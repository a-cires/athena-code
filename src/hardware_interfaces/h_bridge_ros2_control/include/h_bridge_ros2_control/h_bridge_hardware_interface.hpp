#ifndef H_BRIDGE_ROS2_CONTROL__H_BRIDGE_HARDWARE_INTERFACE_HPP_
#define H_BRIDGE_ROS2_CONTROL__H_BRIDGE_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "umdloop_can_library/SocketCanBus.hpp"
#include "umdloop_can_library/CanFrame.hpp"

namespace h_bridge_ros2_control
{

class HBridgeHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HBridgeHardwareInterface)

  // Initialization
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // State and command interface exports
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Lifecycle
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN message handler
  void on_can_message(const CANLib::CanFrame & frame);

  // Logger
  void logger_function();

  // Hardware Interface Parameters
  int update_rate;
  double elapsed_update_time;
  double elapsed_time;
  double elapsed_logger_time;
  int logger_rate;
  int logger_state;

  int num_joints;

  // State interfaces
  std::vector<double> joint_state_velocity_;

  // Command interfaces
  std::vector<double> joint_command_velocity_;
  std::vector<double> prev_joint_command_velocity_;

  // CAN Library
  CANLib::SocketCanBus canBus;
  CANLib::CanFrame can_tx_frame_;
  CANLib::CanFrame can_rx_frame_;
  std::string can_interface;

  // Per-joint parameters
  std::vector<uint32_t> joint_can_ids;       // CAN arbitration ID per joint (the XXX)
  std::vector<double> joint_max_velocity;    // Max velocity in rad/s per joint

  // CAN Protocol Constants
  // Frame format: 3 bytes — [CMD_BYTE, SPEED_LOW, SPEED_HIGH]
  //   Motor OFF:      XXX#30 00 00  (speed = 0)
  //   Max speed CW:   XXX#30 FF 7F  (speed = +32767)
  //   Max speed CCW:  XXX#30 00 80  (speed = -32768)
  static constexpr uint8_t VELOCITY_CMD = 0x30;
};

}  // namespace h_bridge_ros2_control

#endif  // H_BRIDGE_ROS2_CONTROL__H_BRIDGE_HARDWARE_INTERFACE_HPP_
