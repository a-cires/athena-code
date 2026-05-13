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

#include <chrono>
#include <memory>

#include "gtest/gtest.h"
#include "general_controllers/input_watchdog.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace
{
std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_node(const std::string & name)
{
  rclcpp::NodeOptions opts;
  opts.allow_undeclared_parameters(true);
  opts.automatically_declare_parameters_from_overrides(false);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, opts);
}
}  // namespace

class InputWatchdogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
};

TEST_F(InputWatchdogTest, NeverReceivedReportsStale)
{
  auto node = make_node("watchdog_test_never");
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);

  rclcpp::Time now(1, 0, RCL_ROS_TIME);
  EXPECT_FALSE(wd.is_fresh(now));
}

TEST_F(InputWatchdogTest, FreshAfterNotify)
{
  auto node = make_node("watchdog_test_fresh");
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  wd.notify(t0);
  EXPECT_TRUE(wd.is_fresh(t0));

  // Within the 500ms timeout window
  rclcpp::Time t_within = t0 + rclcpp::Duration::from_seconds(0.4);
  EXPECT_TRUE(wd.is_fresh(t_within));
}

TEST_F(InputWatchdogTest, StaleAfterTimeoutElapses)
{
  auto node = make_node("watchdog_test_stale");
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  wd.notify(t0);

  // Past the 500ms timeout window
  rclcpp::Time t_past = t0 + rclcpp::Duration::from_seconds(0.6);
  EXPECT_FALSE(wd.is_fresh(t_past));

  // A new notify makes it fresh again
  wd.notify(t_past);
  EXPECT_TRUE(wd.is_fresh(t_past));
}

TEST_F(InputWatchdogTest, ResetGoesBackToStale)
{
  auto node = make_node("watchdog_test_reset");
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  wd.notify(t0);
  EXPECT_TRUE(wd.is_fresh(t0));

  wd.reset();
  EXPECT_FALSE(wd.is_fresh(t0));
}

TEST_F(InputWatchdogTest, ZeroTimeoutDisablesWatchdog)
{
  auto node = make_node("watchdog_test_zero");
  // Override parameter to 0 before init.
  node->declare_parameter<double>("controller_input_timeout", 0.0);
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);  // default ignored because parameter is set

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  // Still requires at least one notify.
  EXPECT_FALSE(wd.is_fresh(t0));

  wd.notify(t0);
  rclcpp::Time t_far_future = t0 + rclcpp::Duration::from_seconds(3600.0);
  EXPECT_TRUE(wd.is_fresh(t_far_future));
}

TEST_F(InputWatchdogTest, ParameterOverrideTakesEffect)
{
  auto node = make_node("watchdog_test_override");
  node->declare_parameter<double>("controller_input_timeout", 0.1);

  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);  // default ignored because parameter is set

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  wd.notify(t0);

  EXPECT_TRUE(wd.is_fresh(t0 + rclcpp::Duration::from_seconds(0.05)));
  EXPECT_FALSE(wd.is_fresh(t0 + rclcpp::Duration::from_seconds(0.15)));
}

TEST_F(InputWatchdogTest, ClockJumpBackwardsTreatedAsFresh)
{
  auto node = make_node("watchdog_test_clock_jump");
  general_controllers::InputWatchdog wd;
  wd.init(node, 0.5);

  rclcpp::Time t0(10, 0, RCL_ROS_TIME);
  wd.notify(t0);

  rclcpp::Time t_before = t0 - rclcpp::Duration::from_seconds(5.0);
  EXPECT_TRUE(wd.is_fresh(t_before));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
