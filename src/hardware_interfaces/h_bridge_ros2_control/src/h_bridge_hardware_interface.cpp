#include "h_bridge_ros2_control/h_bridge_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <vector>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace h_bridge_ros2_control
{

// ─── Lifecycle ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn HBridgeHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse per-joint parameters from xacro
  for (const auto & joint : info_.joints) {
    // CAN ID for this joint's H-Bridge controller (hex or decimal)
    joint_can_ids.push_back(
      static_cast<uint32_t>(std::stoul(joint.parameters.at("can_id"), nullptr, 0)));

    // Maximum joint velocity in rad/s — used to scale command [-1, 1] → [-32768, 32767]
    joint_max_velocity.push_back(
      std::abs(std::stod(joint.parameters.at("max_velocity"))));
  }

  num_joints = static_cast<int>(info_.joints.size());
  update_rate = std::stoi(info_.hardware_parameters.at("update_rate"));
  logger_rate = std::stoi(info_.hardware_parameters.at("logger_rate"));
  logger_state = std::stoi(info_.hardware_parameters.at("logger_state"));
  can_interface = info_.hardware_parameters.at("can_interface");

  elapsed_update_time = 0.0;
  elapsed_time = 0.0;
  elapsed_logger_time = 0.0;

  // Initialize state and command vectors
  joint_state_velocity_.assign(num_joints, 0.0);
  joint_command_velocity_.assign(num_joints, 0.0);
  prev_joint_command_velocity_.assign(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HBridgeHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!canBus.open(can_interface,
        std::bind(&HBridgeHardwareInterface::on_can_message, this, std::placeholders::_1)))
  {
    RCLCPP_ERROR(rclcpp::get_logger("HBridgeHardwareInterface"),
      "Failed to open CAN interface %s", can_interface.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"),
    "CAN interface %s opened successfully", can_interface.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HBridgeHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "Activating ...please wait...");

  // Zero all motor commands on activation
  joint_command_velocity_.assign(num_joints, 0.0);
  prev_joint_command_velocity_.assign(num_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HBridgeHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "Deactivating ...please wait...");

  // Send motor OFF (speed = 0) to every joint
  for (int i = 0; i < num_joints; i++) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = joint_can_ids[i];
    can_tx_frame_.dlc = 3;
    can_tx_frame_.data[0] = VELOCITY_CMD;
    can_tx_frame_.data[1] = 0x00;
    can_tx_frame_.data[2] = 0x00;
    canBus.send(can_tx_frame_);
  }

  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"),
    "Successfully deactivated all H-Bridge motors!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HBridgeHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "Cleaning up ...please wait...");

  // Send motor OFF to every joint before closing the bus
  for (int i = 0; i < num_joints; i++) {
    can_tx_frame_ = CANLib::CanFrame();
    can_tx_frame_.id = joint_can_ids[i];
    can_tx_frame_.dlc = 3;
    can_tx_frame_.data[0] = VELOCITY_CMD;
    can_tx_frame_.data[1] = 0x00;
    can_tx_frame_.data[2] = 0x00;
    canBus.send(can_tx_frame_);
  }

  canBus.close();

  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "Cleaning up successful!");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn HBridgeHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  return this->on_cleanup(previous_state);
}


// ─── Interface Exports ───────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
HBridgeHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (int i = 0; i < num_joints; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
HBridgeHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int i = 0; i < num_joints; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  return command_interfaces;
}


// ─── CAN Callback ────────────────────────────────────────────────────────────

void HBridgeHardwareInterface::on_can_message(const CANLib::CanFrame & frame)
{
  can_rx_frame_ = frame;

  // If the H-Bridge PCB sends telemetry back, decode it here.
  // For now this is a placeholder — populate once the response protocol is defined.
}


// ─── Read / Write ────────────────────────────────────────────────────────────

hardware_interface::return_type HBridgeHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // H-Bridge is open-loop velocity; state mirrors the last command sent.
  // Replace with actual CAN feedback once the PCB supports it.
  for (int i = 0; i < num_joints; i++) {
    joint_state_velocity_[i] = joint_command_velocity_[i];
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type HBridgeHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  elapsed_update_time += period.seconds();
  elapsed_time += period.seconds();
  double update_period = 1.0 / update_rate;

  // Logger update
  elapsed_logger_time += period.seconds();
  double logging_period = 1.0 / logger_rate;
  if (elapsed_logger_time > logging_period) {
    elapsed_logger_time = 0.0;
    if (logger_state == 1) {
      logger_function();
    }
  }

  if (elapsed_update_time > update_period) {
    elapsed_update_time = 0.0;

    for (int i = 0; i < num_joints; i++) {
      if (!std::isfinite(joint_command_velocity_[i]) ||
          joint_command_velocity_[i] == prev_joint_command_velocity_[i])
      {
        continue;
      }

      // Clamp to max velocity
      double clamped = std::clamp(
        joint_command_velocity_[i], -joint_max_velocity[i], joint_max_velocity[i]);

      // Scale: joint velocity (rad/s) → int16 where ±max_velocity maps to ±32767
      // Motor OFF:    0x0000 (0)
      // Max CW:       0x7FFF (+32767)
      // Max CCW:      0x8000 (-32768)
      int16_t speed_raw = static_cast<int16_t>(
        std::round((clamped / joint_max_velocity[i]) * 32767.0));

      // Build CAN frame: [VELOCITY_CMD, SPEED_LOW, SPEED_HIGH]
      can_tx_frame_ = CANLib::CanFrame();
      can_tx_frame_.id = joint_can_ids[i];
      can_tx_frame_.dlc = 3;
      can_tx_frame_.data[0] = VELOCITY_CMD;
      can_tx_frame_.data[1] = static_cast<uint8_t>(speed_raw & 0xFF);
      can_tx_frame_.data[2] = static_cast<uint8_t>((speed_raw >> 8) & 0xFF);

      canBus.send(can_tx_frame_);

      prev_joint_command_velocity_[i] = joint_command_velocity_[i];
    }
  }

  return hardware_interface::return_type::OK;
}


// ─── Logger ──────────────────────────────────────────────────────────────────

void HBridgeHardwareInterface::logger_function()
{
  if (num_joints == 0) return;

  std::ostringstream oss;
  oss << "\033[2J\033[H \nH-Bridge Logger"
      << "\n--- HWI Specific ---\n"
      << "CAN Interface: " << can_interface
      << " | HWI Update Rate: " << update_rate
      << " | Logger Update Rate: " << logger_rate << "\n"
      << "Elapsed Time since first update: " << elapsed_time << "\n"
      << "\n--- Joint Specific ---";

  for (int i = 0; i < num_joints; i++) {
    oss << "\nJOINT: " << info_.joints[i].name << "\n"
        << "Parameters: CAN ID: 0x" << std::hex << std::uppercase << joint_can_ids[i]
        << " | Max Velocity: " << std::dec << joint_max_velocity[i] << " rad/s\n"
        << "Command Velocity: " << joint_command_velocity_[i]
        << " | State Velocity: " << joint_state_velocity_[i] << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("HBridgeHardwareInterface"), "%s", oss.str().c_str());
}


}  // namespace h_bridge_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  h_bridge_ros2_control::HBridgeHardwareInterface, hardware_interface::SystemInterface)
