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

#ifndef ATHENA_ARM_CONTROLLERS__AUTONOMOUS_TYPING_CONTROLLER_HPP_
#define ATHENA_ARM_CONTROLLERS__AUTONOMOUS_TYPING_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <arm_controllers/autonomous_typing_controller_parameters.hpp>
#include "athena_arm_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace arm_controllers
{

// State machine for typing sequence
enum class TypingState : uint8_t
{
  IDLE = 0,
  NAVIGATING = 1,
  PRESSING = 2,
};

// Sub-phases within PRESSING state
enum class PressPhase : uint8_t
{
  PUSH_DOWN = 0,
  HOLD = 1,
  RETRACT = 2,
};

// Command interface indices (position joints first, then velocity joints)
static constexpr size_t CMD_WRIST_ROLL = 0;     // position
static constexpr size_t CMD_ACTUATOR = 1;        // position
static constexpr size_t CMD_BASE_YAW = 2;        // velocity (held at 0)
static constexpr size_t CMD_WRIST_PITCH = 3;     // velocity (held at 0)
static constexpr size_t CMD_GRIPPER_CLAW = 4;    // velocity

// State interface indices (same ordering)
static constexpr size_t STATE_WRIST_ROLL = 0;
static constexpr size_t STATE_ACTUATOR = 1;
static constexpr size_t STATE_BASE_YAW = 2;
static constexpr size_t STATE_WRIST_PITCH = 3;
static constexpr size_t STATE_GRIPPER_CLAW = 4;

class AutonomousTypingController : public controller_interface::ControllerInterface
{
public:
  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  AutonomousTypingController();

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ATHENA_ARM_CONTROLLERS__VISIBILITY_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = std_msgs::msg::Float64MultiArray;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<autonomous_typing_controller::ParamListener> param_listener_;
  autonomous_typing_controller::Params params_;

  int num_joints_;

  // Current joint positions read from state interfaces
  std::vector<double> current_joint_positions_;

  // Typing state machine
  TypingState typing_state_;
  PressPhase press_phase_;
  size_t current_key_index_;
  double press_timer_;
  bool sequence_loaded_;

  // Actuator positions captured during operation
  double actuator_neutral_position_;
  double actuator_press_position_;

  // Current navigation targets
  double target_theta_;
  double target_r_;

  // Parsed key targets from input message
  std::vector<double> key_r_targets_;
  std::vector<double> key_theta_targets_;

  // Track previous message to detect new commands
  size_t prev_msg_size_;

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

private:
  ATHENA_ARM_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace arm_controllers

#endif  // ATHENA_ARM_CONTROLLERS__AUTONOMOUS_TYPING_CONTROLLER_HPP_
