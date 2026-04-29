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

#include <algorithm>
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

  // ── Build ordered joint list and joint_map_ ──────────────────────────────
  all_joint_names_.clear();
  joint_map_.clear();

  // Ordering: position_only, then velocity_only, then dual
  for (const auto & name : params_.position_only_joints) { all_joint_names_.push_back(name); }
  for (const auto & name : params_.velocity_only_joints) { all_joint_names_.push_back(name); }
  for (const auto & name : params_.dual_joints) { all_joint_names_.push_back(name); }

  num_joints_ = static_cast<int>(all_joint_names_.size());
  current_joint_positions_.resize(num_joints_, 0.0);

  // Assign command interface indices sequentially
  int cmd_idx = 0;
  int state_idx = 0;

  for (const auto & name : params_.position_only_joints) {
    JointInfo ji;
    ji.name = name;
    ji.pos_cmd_idx = cmd_idx++;
    ji.state_pos_idx = state_idx++;
    joint_map_[name] = ji;
  }
  for (const auto & name : params_.velocity_only_joints) {
    JointInfo ji;
    ji.name = name;
    ji.vel_cmd_idx = cmd_idx++;
    ji.state_pos_idx = state_idx++;
    joint_map_[name] = ji;
  }
  for (const auto & name : params_.dual_joints) {
    JointInfo ji;
    ji.name = name;
    ji.pos_cmd_idx = cmd_idx++;
    ji.vel_cmd_idx = cmd_idx++;
    ji.state_pos_idx = state_idx++;
    joint_map_[name] = ji;
  }

  num_cmd_interfaces_ = cmd_idx;

  // Assign max velocities (order: position_only, velocity_only, dual)
  if (static_cast<int>(params_.max_joint_velocities.size()) != num_joints_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_joint_velocities size (%zu) does not match total joint count (%d)",
      params_.max_joint_velocities.size(), num_joints_);
    return controller_interface::CallbackReturn::ERROR;
  }
  for (int i = 0; i < num_joints_; ++i) {
    joint_map_[all_joint_names_[i]].max_velocity = std::abs(params_.max_joint_velocities[i]);
  }

  // ── Build PID state map ──────────────────────────────────────────────────
  pid_states_.clear();
  for (size_t i = 0; i < params_.pid_joints.size(); ++i) {
    const auto & name = params_.pid_joints[i];
    if (joint_map_.find(name) == joint_map_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "PID joint '%s' not found in any joint category", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    PidState ps;
    ps.kp = (i < params_.pid_kp.size()) ? params_.pid_kp[i] : 0.0;
    ps.ki = (i < params_.pid_ki.size()) ? params_.pid_ki[i] : 0.0;
    ps.kd = (i < params_.pid_kd.size()) ? params_.pid_kd[i] : 0.0;
    pid_states_[name] = ps;
  }

  // ── Initialize state machine ─────────────────────────────────────────────
  typing_state_ = TypingState::IDLE;
  press_phase_ = PressPhase::PUSH_DOWN;
  current_key_index_ = 0;
  press_timer_ = 0.0;
  sequence_loaded_ = false;
  actuator_neutral_position_ = 0.0;
  actuator_press_position_ = 0.0;
  target_theta_ = 0.0;
  target_r_ = 0.0;

  // ── Subscriber ───────────────────────────────────────────────────────────
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
    "~/typing_command", subscribers_qos,
    std::bind(&AutonomousTypingController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg);
  input_ref_.writeFromNonRT(msg);

  // ── State publisher ──────────────────────────────────────────────────────
  try
  {
    s_publisher_ =
      get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr,
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  state_publisher_->lock();
  state_publisher_->msg_.header.frame_id = all_joint_names_[0];
  state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(),
    "Autonomous Typing Controller configured: %d joints, %d cmd interfaces, %zu PID joints",
    num_joints_, num_cmd_interfaces_, pid_states_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

void AutonomousTypingController::reference_callback(
  const std::shared_ptr<ControllerReferenceMsg> msg)
{
  input_ref_.writeFromNonRT(msg);
  new_message_received_.store(true);
}


// ─── Interface configuration ─────────────────────────────────────────────────

controller_interface::InterfaceConfiguration
AutonomousTypingController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(num_cmd_interfaces_);

  for (const auto & name : params_.position_only_joints) {
    config.names.push_back(name + "/position");
  }
  for (const auto & name : params_.velocity_only_joints) {
    config.names.push_back(name + "/velocity");
  }
  for (const auto & name : params_.dual_joints) {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
AutonomousTypingController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.reserve(num_joints_);

  // Read position state from all joints (same ordering as all_joint_names_)
  for (const auto & name : all_joint_names_) {
    config.names.push_back(name + "/position");
  }

  return config;
}


// ─── Lifecycle ───────────────────────────────────────────────────────────────

controller_interface::CallbackReturn AutonomousTypingController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset input reference
  reset_controller_reference_msg(*(input_ref_.readFromRT)());

  // Read initial positions from state interfaces
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    current_joint_positions_[i] = state_interfaces_[i].get_value();
  }

  // Capture actuator neutral position
  actuator_neutral_position_ = get_joint_position("actuator");

  // Reset PID states
  for (auto & [name, ps] : pid_states_) {
    ps.integral = 0.0;
    ps.prev_error = 0.0;
  }

  // Reset state machine
  typing_state_ = TypingState::IDLE;
  current_key_index_ = 0;
  sequence_loaded_ = false;
  new_message_received_.store(false);

  RCLCPP_INFO(get_node()->get_logger(), "Autonomous Typing Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn AutonomousTypingController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }

  RCLCPP_INFO(get_node()->get_logger(), "Autonomous Typing Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}


// ─── Helpers ─────────────────────────────────────────────────────────────────

double AutonomousTypingController::get_joint_position(const std::string & name) const
{
  auto it = joint_map_.find(name);
  if (it != joint_map_.end() && it->second.state_pos_idx >= 0) {
    return current_joint_positions_[it->second.state_pos_idx];
  }
  return std::numeric_limits<double>::quiet_NaN();
}

void AutonomousTypingController::set_joint_position_cmd(const std::string & name, double value)
{
  auto it = joint_map_.find(name);
  if (it != joint_map_.end() && it->second.pos_cmd_idx >= 0) {
    command_interfaces_[it->second.pos_cmd_idx].set_value(value);
  }
}

void AutonomousTypingController::set_joint_velocity_cmd(const std::string & name, double value)
{
  auto it = joint_map_.find(name);
  if (it != joint_map_.end() && it->second.vel_cmd_idx >= 0) {
    command_interfaces_[it->second.vel_cmd_idx].set_value(value);
  }
}

double AutonomousTypingController::compute_pid(
  const std::string & joint_name, double error, double dt)
{
  auto it = pid_states_.find(joint_name);
  if (it == pid_states_.end()) {
    return 0.0;
  }

  auto & ps = it->second;
  ps.integral += error * dt;
  double derivative = (dt > 0.0) ? (error - ps.prev_error) / dt : 0.0;
  ps.prev_error = error;

  double output = ps.kp * error + ps.ki * ps.integral + ps.kd * derivative;

  // Clamp to max velocity
  auto jit = joint_map_.find(joint_name);
  if (jit != joint_map_.end()) {
    output = std::clamp(output, -jit->second.max_velocity, jit->second.max_velocity);
  }

  return output;
}


// ─── Update (RT control loop) ────────────────────────────────────────────────

controller_interface::return_type AutonomousTypingController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  // 1. Read current joint positions from state interfaces
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    current_joint_positions_[i] = state_interfaces_[i].get_value();
  }

  // 2. Check for new message — parse (r, theta) pairs
  if (new_message_received_.exchange(false))
  {
    auto current_ref = input_ref_.readFromRT();
    const auto & data = (*current_ref)->data;

    if (data.empty())
    {
      // Empty message — ignore
    }
    else if (data.size() % 2 != 0)
    {
      RCLCPP_WARN(get_node()->get_logger(),
        "Received odd-length data (%zu). Expected interleaved [r, theta] pairs. Ignoring.",
        data.size());
    }
    else
    {
      key_r_targets_.clear();
      key_theta_targets_.clear();
      for (size_t i = 0; i + 1 < data.size(); i += 2) {
        key_r_targets_.push_back(data[i]);
        key_theta_targets_.push_back(data[i + 1]);
      }
      current_key_index_ = 0;
      sequence_loaded_ = true;
      typing_state_ = TypingState::IDLE;

      RCLCPP_INFO(get_node()->get_logger(),
        "Loaded typing sequence with %zu keys", key_r_targets_.size());
    }
  }

  // 3. Default commands: hold current positions, zero velocities
  double wrist_roll_target = get_joint_position("wrist_roll");
  double actuator_target = get_joint_position("actuator");
  double gripper_target_r = get_joint_position("gripper_claw");  // hold in place by default

  const double tolerance = params_.position_tolerance;

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

        RCLCPP_INFO(get_node()->get_logger(),
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
      // Target wrist_roll to theta
      wrist_roll_target = target_theta_;

      // Target gripper_claw to r
      gripper_target_r = target_r_;

      // Check if both targets reached
      bool theta_reached =
        std::abs(get_joint_position("wrist_roll") - target_theta_) < tolerance;
      bool r_reached =
        std::abs(get_joint_position("gripper_claw") - target_r_) < tolerance;

      if (theta_reached && r_reached)
      {
        actuator_neutral_position_ = get_joint_position("actuator");
        actuator_press_position_ = actuator_neutral_position_ + params_.actuator_press_depth;
        press_phase_ = PressPhase::PUSH_DOWN;
        typing_state_ = TypingState::PRESSING;

        RCLCPP_INFO(get_node()->get_logger(),
          "Reached key %zu, pressing (neutral=%.4f, press=%.4f)",
          current_key_index_, actuator_neutral_position_, actuator_press_position_);
      }
      break;
    }

    case TypingState::PRESSING:
    {
      // Maintain wrist_roll and gripper at their targets during press
      wrist_roll_target = target_theta_;
      gripper_target_r = target_r_;

      double current_actuator = get_joint_position("actuator");

      switch (press_phase_)
      {
        case PressPhase::PUSH_DOWN:
        {
          actuator_target = actuator_press_position_;
          if (std::abs(current_actuator - actuator_press_position_) < tolerance) {
            press_phase_ = PressPhase::HOLD;
            press_timer_ = 0.0;
          }
          break;
        }

        case PressPhase::HOLD:
        {
          actuator_target = actuator_press_position_;
          press_timer_ += dt;
          if (press_timer_ >= params_.key_press_duration) {
            press_phase_ = PressPhase::RETRACT;
          }
          break;
        }

        case PressPhase::RETRACT:
        {
          actuator_target = actuator_neutral_position_;
          if (std::abs(current_actuator - actuator_neutral_position_) < tolerance) {
            current_key_index_++;
            typing_state_ = TypingState::IDLE;

            RCLCPP_INFO(get_node()->get_logger(),
              "Key %zu press complete, advancing to next key",
              current_key_index_ - 1);
          }
          break;
        }
      }
      break;
    }
  }

  // 5. Write commands to all joints
  for (const auto & [name, ji] : joint_map_)
  {
    bool has_pid = (pid_states_.find(name) != pid_states_.end());

    if (has_pid)
    {
      // PID joints: compute position error -> PID -> velocity command
      double target_pos = 0.0;
      if (name == "gripper_claw") {
        target_pos = gripper_target_r;
      } else if (name == "wrist_roll") {
        target_pos = wrist_roll_target;
      } else {
        target_pos = get_joint_position(name);  // hold in place
      }

      double error = target_pos - get_joint_position(name);
      double vel_cmd = compute_pid(name, error, dt);

      // Write velocity command
      set_joint_velocity_cmd(name, vel_cmd);

      // If dual joint, also write position target (HWI can use it if it supports it)
      if (ji.pos_cmd_idx >= 0) {
        set_joint_position_cmd(name, target_pos);
      }
    }
    else
    {
      // Non-PID joints: direct position or velocity command
      if (name == "actuator") {
        set_joint_position_cmd(name, actuator_target);
        set_joint_velocity_cmd(name, 0.0);
      } else if (name == "base_yaw" || name == "wrist_pitch") {
        // Hold: zero velocity, NaN position (or current position if dual)
        if (ji.pos_cmd_idx >= 0) {
          set_joint_position_cmd(name, get_joint_position(name));
        }
        set_joint_velocity_cmd(name, 0.0);
      } else {
        // Any other non-PID joint: hold position, zero velocity
        if (ji.pos_cmd_idx >= 0) {
          set_joint_position_cmd(name, get_joint_position(name));
        }
        if (ji.vel_cmd_idx >= 0) {
          set_joint_velocity_cmd(name, 0.0);
        }
      }
    }
  }

  // 6. Publish state
  if (state_publisher_ && state_publisher_->trylock())
  {
    state_publisher_->msg_.header.stamp = time;
    state_publisher_->msg_.set_point = wrist_roll_target;
    state_publisher_->msg_.process_value = get_joint_position("wrist_roll");
    state_publisher_->msg_.error = wrist_roll_target - get_joint_position("wrist_roll");
    state_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace arm_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::AutonomousTypingController, controller_interface::ControllerInterface)
