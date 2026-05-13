// Copyright (c) 2025, UMDLoop
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

// End-to-end watchdog test for ManualArmJointByJointController.
//
// Verifies the safety contract:
//   1. After on_activate, before any reference message arrives, update()
//      commands zero on every velocity interface.
//   2. After a reference message arrives, update() reflects the commanded
//      velocities (proving normal control still works).
//   3. After the configured input_timeout elapses without a new message,
//      update() returns to commanding zero on every velocity interface.

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "athena_arm_controllers/manual_arm_joint_by_joint_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"

using arm_controllers::ManualArmJointByJointController;

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;

// Subclass that exposes input_ref_ for direct injection (avoids needing a
// ROS executor in the unit test).
class TestableManualArmJointByJointController : public ManualArmJointByJointController
{
public:
  using ManualArmJointByJointController::input_ref_;
  using ManualArmJointByJointController::input_watchdog_;
};
}  // namespace

class ManualArmWatchdogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    controller_ = std::make_unique<TestableManualArmJointByJointController>();
    rclcpp::NodeOptions opts;
    // Provide required parameters so on_init() succeeds.
    opts.parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"j1", "j2", "j3"}),
      rclcpp::Parameter(
        "joint_max_velocities",
        std::vector<double>{1.0, 1.0, 1.0}),
      rclcpp::Parameter("virtual_four_bar_coupling_ratio", 0.0),
      rclcpp::Parameter("controller_input_timeout", 0.2),
    });

    ASSERT_EQ(
      controller_->init("test_manual_arm_jbj", "", opts),
      controller_interface::return_type::OK);

    // Wire three velocity command interfaces backed by storage in the test.
    cmd_values_.assign(3, std::numeric_limits<double>::quiet_NaN());
    state_values_.assign(3, 0.0);

    cmd_storage_.clear();
    state_storage_.clear();
    cmd_storage_.reserve(3);
    state_storage_.reserve(3);

    std::vector<hardware_interface::LoanedCommandInterface> cmd_loaned;
    std::vector<hardware_interface::LoanedStateInterface> state_loaned;
    const std::vector<std::string> joints{"j1", "j2", "j3"};
    for (size_t i = 0; i < 3; ++i) {
      cmd_storage_.emplace_back(joints[i], "velocity", &cmd_values_[i]);
      cmd_loaned.emplace_back(cmd_storage_.back());

      state_storage_.emplace_back(joints[i], "velocity", &state_values_[i]);
      state_loaned.emplace_back(state_storage_.back());
    }

    controller_->assign_interfaces(std::move(cmd_loaned), std::move(state_loaned));
  }

  // Inject a reference message directly into the RT buffer and notify the
  // watchdog at `now` -- equivalent to a real subscription callback firing.
  void inject_reference(
    const rclcpp::Time & now,
    const std::vector<double> & axes,
    const std::vector<int32_t> & buttons)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Joy>();
    msg->axes.assign(axes.begin(), axes.end());
    msg->buttons.assign(buttons.begin(), buttons.end());
    controller_->input_ref_.writeFromNonRT(msg);
    controller_->input_watchdog_.notify(now);
  }

  std::unique_ptr<TestableManualArmJointByJointController> controller_;
  std::vector<double> cmd_values_;
  std::vector<double> state_values_;
  std::vector<hardware_interface::CommandInterface> cmd_storage_;
  std::vector<hardware_interface::StateInterface> state_storage_;
};

TEST_F(ManualArmWatchdogTest, ZerosBeforeAnyReferenceArrives)
{
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  ASSERT_EQ(
    controller_->update(t0, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (double v : cmd_values_) {
    EXPECT_DOUBLE_EQ(v, 0.0) << "stale-state should command 0 on velocity interfaces";
  }
}

TEST_F(ManualArmWatchdogTest, NormalControlPropagatesAfterReference)
{
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);

  // axes layout used by the controller: [base_yaw, shoulder, _, elbow, ...]
  // buttons[1] gates base_yaw and shoulder; we keep it 0 so they pass.
  std::vector<double> axes(6, 0.0);
  std::vector<int32_t> buttons(13, 0);
  axes[0] = 0.5;   // base_yaw
  axes[1] = 0.25;  // shoulder
  axes[3] = -0.1;  // elbow

  inject_reference(t0, axes, buttons);

  ASSERT_EQ(
    controller_->update(t0, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  // joint_max_velocities is [1, 1, 1], coupling ratio 0, so commands ==
  // axes-driven values. axes are float32 in sensor_msgs::Joy so use NEAR.
  EXPECT_NEAR(cmd_values_[0], 0.5, 1e-5);
  EXPECT_NEAR(cmd_values_[1], 0.25, 1e-5);
  EXPECT_NEAR(cmd_values_[2], -0.1, 1e-5);
}

TEST_F(ManualArmWatchdogTest, ZerosAgainAfterTimeoutElapses)
{
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);

  std::vector<double> axes(6, 0.0);
  std::vector<int32_t> buttons(13, 0);
  axes[0] = 0.7;
  axes[1] = 0.6;
  axes[3] = 0.5;
  inject_reference(t0, axes, buttons);

  ASSERT_EQ(
    controller_->update(t0, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_NEAR(cmd_values_[0], 0.7, 1e-5);
  EXPECT_NEAR(cmd_values_[1], 0.6, 1e-5);
  EXPECT_NEAR(cmd_values_[2], 0.5, 1e-5);

  // Advance well past the configured 0.2 s timeout without injecting more
  // references. The controller must fall back to safe-stop (zero velocities).
  rclcpp::Time t_stale = t0 + rclcpp::Duration::from_seconds(0.5);
  ASSERT_EQ(
    controller_->update(t_stale, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  for (double v : cmd_values_) {
    EXPECT_DOUBLE_EQ(v, 0.0);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
