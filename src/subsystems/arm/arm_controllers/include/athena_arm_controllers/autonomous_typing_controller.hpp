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

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
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

// Per-joint command interface index info — built at configure time
struct JointInfo {
  std::string name;
  int pos_cmd_idx = -1;   // index into command_interfaces_, -1 if not claimed
  int vel_cmd_idx = -1;   // index into command_interfaces_, -1 if not claimed
  int state_pos_idx = -1; // index into state_interfaces_
  double max_velocity = 0.0;
};

// Per-joint PID state
struct PidState {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double integral = 0.0;
  double prev_error = 0.0;
};

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
  int num_cmd_interfaces_;

  // Joint info map: joint_name -> JointInfo (built at configure time)
  std::unordered_map<std::string, JointInfo> joint_map_;

  // Ordered list of all joint names (position_only, velocity_only, dual)
  // Used for state interface ordering
  std::vector<std::string> all_joint_names_;

  // PID state map: joint_name -> PidState (only for pid_joints)
  std::unordered_map<std::string, PidState> pid_states_;

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

  // Track new messages via atomic flag (set in callback, cleared in update)
  std::atomic<bool> new_message_received_{false};

  // Command subscribers and Controller State publisher
  rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // Helper: compute PID output for a joint, returns velocity command
  double compute_pid(const std::string & joint_name, double error, double dt);

  // Helper: write a command to a joint by name
  void set_joint_position_cmd(const std::string & name, double value);
  void set_joint_velocity_cmd(const std::string & name, double value);

  // Helper: read current position of a joint by name
  double get_joint_position(const std::string & name) const;

private:
  ATHENA_ARM_CONTROLLERS__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

}  // namespace arm_controllers

#endif  // ATHENA_ARM_CONTROLLERS__AUTONOMOUS_TYPING_CONTROLLER_HPP_
