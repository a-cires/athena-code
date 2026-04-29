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

#ifndef ARM_AUTO_TYPING__ARM_AUTO_TYPING_NODE_HPP_
#define ARM_AUTO_TYPING__ARM_AUTO_TYPING_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace arm_auto_typing
{

/// @brief Cartesian position of a key on the keyboard (millimeters).
struct CartesianKey
{
  double x;  ///< Horizontal position (mm)
  double y;  ///< Vertical position (mm)
};

/// @brief Polar position of a key relative to the arm origin.
struct PolarKey
{
  double r;      ///< Radial distance from origin (mm)
  double theta;  ///< Angle from the +Y axis, measured clockwise (radians)
};

/// @brief High-level node that accepts a key sequence (as a string) and publishes
///        interleaved [r, theta, r, theta, ...] Float64MultiArray messages to the
///        autonomous_typing_controller's ~/typing_command topic.
///
///        Key positions are defined in Cartesian coordinates (from physical
///        keyboard measurements) and converted to polar at startup using a
///        configurable origin point.
class ArmAutoTypingNode : public rclcpp::Node
{
public:
  explicit ArmAutoTypingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// @brief Build the polar key lookup table from Cartesian parameters + origin.
  void build_key_map();

  /// @brief Convert a single Cartesian point to polar relative to origin_.
  ///        theta uses atan2(dx, dy) so 0 rad = +Y (north), positive = clockwise.
  PolarKey cartesian_to_polar(const CartesianKey & key) const;

  /// @brief Convert a text string to interleaved [r, theta, r, theta, ...] data.
  std::vector<double> text_to_targets(const std::string & text) const;

  /// @brief Callback for incoming key sequence text.
  void text_callback(const std_msgs::msg::String::SharedPtr msg);

  /// @brief Service callback to abort the current typing sequence.
  void abort_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // ── ROS interfaces ─────────────────────────────────────────────────────
  /// Publisher: sends Float64MultiArray to the autonomous_typing_controller
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr typing_command_pub_;

  /// Subscriber: receives key-sequence strings to type
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;

  /// Service: abort current sequence
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;

  // ── Data ───────────────────────────────────────────────────────────────
  /// Origin point for polar conversion (arm wrist pivot projected onto keyboard)
  CartesianKey origin_;

  /// Polar key lookup: single character -> polar position
  std::unordered_map<std::string, PolarKey> key_map_;
};

}  // namespace arm_auto_typing

#endif  // ARM_AUTO_TYPING__ARM_AUTO_TYPING_NODE_HPP_
