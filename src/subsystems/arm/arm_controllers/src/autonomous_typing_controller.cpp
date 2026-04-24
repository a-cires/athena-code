// Copyright (c) 2025, UMDLoop
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "athena_arm_controllers/autonomous_typing_controller.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

using ControllerReferenceMsg = arm_controllers::AutonomousTypingController::ControllerReferenceMsg;

void reset_controller_reference_msg(std::shared_ptr<ControllerReferenceMsg> & msg)
{
  msg->data.clear();
}

}  // namespace

namespace arm_controllers
{
AutonomousTypingController::AutonomousTypingController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn AutonomousTypingController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<autonomous_typing_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AutonomousTypingController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  num_joints_ = static_cast<int>(params_.position_joints.size()) +
                static_cast<int>(params_.velocity_joints.size());
  current_joint_positions_.resize(num_joints_, 0.0);

  // Initialize state machine
  typing_state_ = TypingState::IDLE;
  press_phase_ = PressPhase::PUSH_DOWN;
  current_key_index_ = 0;
  press_timer_ = 0.0;
  sequence_loaded_ = false;
  actuator_neutral_position_ = 0.0;
  actuator_press_position_ = 0.0;
  target_theta_ = 0.0;
  target_r_ = 0.0;
  prev_msg_size_ = 0;

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/typing_command", subscribers_qos,
    std::bind(&AutonomousTypingController::reference_callback, this, std::placeholders::_1));

  // Initialize input reference with empty message
  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  try
  {
    // State publisher
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = params_.position_joints[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "Autonomous Typing Controller configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

void AutonomousTypingController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
}

controller_interface::InterfaceConfiguration AutonomousTypingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(num_joints_);

  // Position-controlled joints first
  for (const auto & joint : params_.position_joints)
  {
    command_interfaces_config.names.push_back(joint + "/position");
  }

  // Velocity-controlled joints second
  for (const auto & joint : params_.velocity_joints)
  {
    command_interfaces_config.names.push_back(joint + "/velocity");
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration AutonomousTypingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(num_joints_);

  // Read position state for all joints (same ordering as command interfaces)
  for (const auto & joint : params_.position_joints)
  {
    state_interfaces_config.names.push_back(joint + "/position");
  }

  for (const auto & joint : params_.velocity_joints)
  {
    state_interfaces_config.names.push_back(joint + "/position");
  }

  return state_interfaces_config;
}

controller_interface::CallbackReturn AutonomousTypingController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset input reference
  reset_controller_reference_msg(*(input_ref_.readFromRT)());

  // Read initial positions from state interfaces
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    current_joint_positions_[i] = state_interfaces_[i].get_value();
  }

  // Capture actuator neutral position
  actuator_neutral_position_ = current_joint_positions_[STATE_ACTUATOR];

  // Reset state machine
  typing_state_ = TypingState::IDLE;
  current_key_index_ = 0;
  sequence_loaded_ = false;
  prev_msg_size_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), "Autonomous Typing Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AutonomousTypingController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Autonomous Typing Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type AutonomousTypingController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // 1. Read current joint positions from state interfaces
  for (size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    current_joint_positions_[i] = state_interfaces_[i].get_value();
  }

  // 2. Check for new message — parse (r, theta) pairs
  auto current_ref = input_ref_.readFromRT();
  const auto & data = (*current_ref)->data;

  if (!data.empty() && data.size() != prev_msg_size_)
  {
    if (data.size() % 2 != 0)
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Received odd-length data (%zu). Expected interleaved [r, theta] pairs. Ignoring.",
        data.size());
    }
    else
    {
      key_r_targets_.clear();
      key_theta_targets_.clear();
      for (size_t i = 0; i + 1 < data.size(); i += 2)
      {
        key_r_targets_.push_back(data[i]);
        key_theta_targets_.push_back(data[i + 1]);
      }
      current_key_index_ = 0;
      sequence_loaded_ = true;
      typing_state_ = TypingState::IDLE;

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Loaded typing sequence with %zu keys", key_r_targets_.size());
    }
    prev_msg_size_ = data.size();
  }

  // 3. Default commands: hold current positions, zero velocities
  double wrist_roll_cmd = current_joint_positions_[STATE_WRIST_ROLL];
  double actuator_cmd = current_joint_positions_[STATE_ACTUATOR];
  double gripper_velocity = 0.0;

  const double tolerance = params_.position_tolerance;
  const double kp = params_.gripper_kp;
  const double max_gripper_vel = params_.max_joint_velocities[2];  // gripper_claw is index 2 in velocity_joints

  // 4. State machine
  switch (typing_state_)
  {
    case TypingState::IDLE:
    {
      if (sequence_loaded_ && current_key_index_ < key_r_targets_.size())
      {
        target_theta_ = key_theta_targets_[current_key_index_];
        target_r_ = key_r_targets_[current_key_index_];
        typing_state_ = TypingState::NAVIGATING;

        RCLCPP_INFO(
          get_node()->get_logger(),
          "Navigating to key %zu: r=%.4f, theta=%.4f rad",
          current_key_index_, target_r_, target_theta_);
      }
      else if (sequence_loaded_ && current_key_index_ >= key_r_targets_.size())
      {
        RCLCPP_INFO(get_node()->get_logger(), "Typing sequence complete");
        sequence_loaded_ = false;
      }
      break;
    }

    case TypingState::NAVIGATING:
    {
      // Position command for wrist_roll (theta)
      wrist_roll_cmd = target_theta_;

      // Velocity P-controller for gripper_claw (r)
      double r_error = target_r_ - current_joint_positions_[STATE_GRIPPER_CLAW];
      gripper_velocity = kp * r_error;
      gripper_velocity = std::clamp(gripper_velocity, -max_gripper_vel, max_gripper_vel);

      // Check if both targets reached
      bool theta_reached =
        std::abs(current_joint_positions_[STATE_WRIST_ROLL] - target_theta_) < tolerance;
      bool r_reached = std::abs(r_error) < tolerance;

      if (theta_reached && r_reached)
      {
        // Capture actuator neutral before pressing
        actuator_neutral_position_ = current_joint_positions_[STATE_ACTUATOR];
        actuator_press_position_ = actuator_neutral_position_ + params_.actuator_press_depth;
        press_phase_ = PressPhase::PUSH_DOWN;
        typing_state_ = TypingState::PRESSING;

        RCLCPP_INFO(
          get_node()->get_logger(),
          "Reached key %zu, pressing (neutral=%.4f, press=%.4f)",
          current_key_index_, actuator_neutral_position_, actuator_press_position_);
      }
      break;
    }

    case TypingState::PRESSING:
    {
      // Maintain wrist_roll and gripper at their targets during press
      wrist_roll_cmd = target_theta_;
      double r_error = target_r_ - current_joint_positions_[STATE_GRIPPER_CLAW];
      gripper_velocity = kp * r_error;
      gripper_velocity = std::clamp(gripper_velocity, -max_gripper_vel, max_gripper_vel);

      double current_actuator = current_joint_positions_[STATE_ACTUATOR];

      switch (press_phase_)
      {
        case PressPhase::PUSH_DOWN:
        {
          actuator_cmd = actuator_press_position_;
          if (std::abs(current_actuator - actuator_press_position_) < tolerance)
          {
            press_phase_ = PressPhase::HOLD;
            press_timer_ = 0.0;
          }
          break;
        }

        case PressPhase::HOLD:
        {
          actuator_cmd = actuator_press_position_;
          press_timer_ += period.seconds();
          if (press_timer_ >= params_.key_press_duration)
          {
            press_phase_ = PressPhase::RETRACT;
          }
          break;
        }

        case PressPhase::RETRACT:
        {
          actuator_cmd = actuator_neutral_position_;
          if (std::abs(current_actuator - actuator_neutral_position_) < tolerance)
          {
            current_key_index_++;
            typing_state_ = TypingState::IDLE;

            RCLCPP_INFO(
              get_node()->get_logger(),
              "Key %zu press complete, advancing to next key",
              current_key_index_ - 1);
          }
          break;
        }
      }
      break;
    }
  }

  // 5. Write commands to interfaces
  command_interfaces_[CMD_WRIST_ROLL].set_value(wrist_roll_cmd);      // position
  command_interfaces_[CMD_ACTUATOR].set_value(actuator_cmd);           // position
  command_interfaces_[CMD_BASE_YAW].set_value(0.0);                    // velocity (hold)
  command_interfaces_[CMD_WRIST_PITCH].set_value(0.0);                 // velocity (hold)
  command_interfaces_[CMD_GRIPPER_CLAW].set_value(gripper_velocity);   // velocity

  // 6. Publish state
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = wrist_roll_cmd;
    state_publisher_->msg_.process_value = current_joint_positions_[STATE_WRIST_ROLL];
    state_publisher_->msg_.error = wrist_roll_cmd - current_joint_positions_[STATE_WRIST_ROLL];
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::AutonomousTypingController, controller_interface::ControllerInterface)
