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

#include "arm_auto_typing/arm_auto_typing_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

namespace arm_auto_typing
{

ArmAutoTypingNode::ArmAutoTypingNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("arm_auto_typing_node", options)
{
  // ── Declare parameters ─────────────────────────────────────────────────
  this->declare_parameter<std::string>(
    "typing_command_topic",
    "/autonomous_typing_controller/typing_command");
  this->declare_parameter<std::string>("text_input_topic", "~/text_input");

  // Origin point (arm wrist pivot projected onto keyboard plane, in mm)
  this->declare_parameter<double>("origin.x", 0.0);
  this->declare_parameter<double>("origin.y", 0.0);

  // Key layout: parallel arrays of key names and their Cartesian positions (mm)
  this->declare_parameter<std::vector<std::string>>("key_layout.keys", std::vector<std::string>());
  this->declare_parameter<std::vector<double>>("key_layout.x_positions", std::vector<double>());
  this->declare_parameter<std::vector<double>>("key_layout.y_positions", std::vector<double>());

  // ── Read origin ────────────────────────────────────────────────────────
  origin_.x = this->get_parameter("origin.x").as_double();
  origin_.y = this->get_parameter("origin.y").as_double();

  // ── Build polar key map from Cartesian measurements ────────────────────
  build_key_map();

  // ── Publisher: Float64MultiArray -> autonomous_typing_controller ────────
  auto typing_topic = this->get_parameter("typing_command_topic").as_string();
  typing_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    typing_topic, rclcpp::SystemDefaultsQoS());

  // ── Subscriber: String key sequence ────────────────────────────────────
  auto text_topic = this->get_parameter("text_input_topic").as_string();
  text_sub_ = this->create_subscription<std_msgs::msg::String>(
    text_topic, rclcpp::SystemDefaultsQoS(),
    std::bind(&ArmAutoTypingNode::text_callback, this, std::placeholders::_1));

  // ── Abort service ──────────────────────────────────────────────────────
  abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/abort",
    std::bind(
      &ArmAutoTypingNode::abort_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    this->get_logger(),
    "ArmAutoTypingNode started — %zu keys loaded, origin=(%.1f, %.1f) mm",
    key_map_.size(), origin_.x, origin_.y);
}

// ─── Polar conversion ────────────────────────────────────────────────────────

PolarKey ArmAutoTypingNode::cartesian_to_polar(const CartesianKey & key) const
{
  double dx = key.x - origin_.x;
  double dy = key.y - origin_.y;

  double r = std::sqrt(dx * dx + dy * dy);

  // atan2(dx, dy) gives angle from +Y axis (north), positive clockwise.
  // This matches the convention in test_polar_conversion.cpp.
  double theta = std::atan2(dx, dy);

  // Normalize to [0, 2π)
  if (theta < 0.0) {
    theta += 2.0 * M_PI;
  }

  return {r, theta};
}

void ArmAutoTypingNode::build_key_map()
{
  auto keys = this->get_parameter("key_layout.keys").as_string_array();
  auto x_positions = this->get_parameter("key_layout.x_positions").as_double_array();
  auto y_positions = this->get_parameter("key_layout.y_positions").as_double_array();

  if (keys.size() != x_positions.size() || keys.size() != y_positions.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Key layout arrays must have equal length (keys=%zu, x=%zu, y=%zu)",
      keys.size(), x_positions.size(), y_positions.size());
    return;
  }

  key_map_.clear();
  for (size_t i = 0; i < keys.size(); ++i) {
    if (keys[i].empty()) {
      continue;
    }

    CartesianKey cart{x_positions[i], y_positions[i]};
    PolarKey polar = cartesian_to_polar(cart);
    key_map_[keys[i]] = polar;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Key '%s': cart=(%.1f, %.1f) -> polar=(r=%.2f mm, theta=%.4f rad / %.1f deg)",
      keys[i].c_str(), cart.x, cart.y, polar.r, polar.theta,
      polar.theta * 180.0 / M_PI);
  }
}

// ─── Text -> target conversion ───────────────────────────────────────────────

std::vector<double> ArmAutoTypingNode::text_to_targets(const std::string & text) const
{
  std::vector<double> targets;
  targets.reserve(text.size() * 2);

  for (size_t i = 0; i < text.size(); ++i) {
    // Build a single-character string for lookup
    std::string key_str(1, static_cast<char>(
      std::tolower(static_cast<unsigned char>(text[i]))));

    auto it = key_map_.find(key_str);
    if (it != key_map_.end()) {
      // Interleaved [r, theta] — matches what autonomous_typing_controller expects
      targets.push_back(it->second.r);
      targets.push_back(it->second.theta);
    } else {
      RCLCPP_WARN(
        this->get_logger(), "No key mapping for '%s', skipping", key_str.c_str());
    }
  }

  return targets;
}

// ─── Callbacks ───────────────────────────────────────────────────────────────

void ArmAutoTypingNode::text_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty text, ignoring");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received text to type: \"%s\"", msg->data.c_str());

  auto targets = text_to_targets(msg->data);
  if (targets.empty()) {
    RCLCPP_WARN(this->get_logger(), "No valid key targets for input text");
    return;
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = targets;
  typing_command_pub_->publish(cmd);

  RCLCPP_INFO(
    this->get_logger(), "Published typing sequence: %zu keys as %zu doubles",
    targets.size() / 2, targets.size());
}

void ArmAutoTypingNode::abort_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  // Empty message clears the typing sequence in the controller
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data.clear();
  typing_command_pub_->publish(cmd);

  response->success = true;
  response->message = "Typing sequence aborted";
  RCLCPP_INFO(this->get_logger(), "Typing sequence aborted via service call");
}

}  // namespace arm_auto_typing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arm_auto_typing::ArmAutoTypingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
