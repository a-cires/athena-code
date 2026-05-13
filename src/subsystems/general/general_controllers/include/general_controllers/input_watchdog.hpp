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

#ifndef GENERAL_CONTROLLERS__INPUT_WATCHDOG_HPP_
#define GENERAL_CONTROLLERS__INPUT_WATCHDOG_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace general_controllers
{

/**
 * @brief Tracks freshness of reference inputs to a ros2_control controller.
 *
 * Usage:
 *   1. Construct as a member of the controller class.
 *   2. In on_configure(), call init(node, default_timeout_s). This declares
 *      the "controller_input_timeout" parameter on the node and reads it.
 *   3. In on_activate(), call reset() to require a fresh message before any
 *      input is considered valid.
 *   4. In the reference subscription callback (non-RT), call notify(now).
 *   5. In update() (RT), call is_fresh(now) to gate command output.
 *
 * Realtime safety:
 *   - notify() and is_fresh() are lock-free; they only manipulate an
 *     atomic int64 timestamp counter.
 *   - No allocations or logging in the RT path. A throttled WARN/INFO is
 *     emitted via log_state_transition(), which is intended to be called
 *     from update() but uses RCLCPP_*_THROTTLE which is safe enough for our
 *     real-time profile (no blocking, bounded work).
 *
 * Behavior:
 *   - Until notify() has been called at least once after reset(), is_fresh()
 *     returns false (so a freshly-activated controller commands a safe
 *     default until the operator publishes).
 *   - timeout == 0 disables the freshness check entirely (escape hatch for
 *     bringup/testing). In that case is_fresh() returns true as soon as one
 *     message has been received.
 */
class InputWatchdog
{
public:
  InputWatchdog() = default;

  /**
   * @brief Declare the controller_input_timeout parameter on the node and
   * cache its value. Safe to call multiple times across reconfigures.
   *
   * @param node Lifecycle node owning the controller.
   * @param default_timeout_s Default timeout in seconds if the parameter
   *   has not been set externally. Negative defaults are treated as 0.5s.
   */
  void init(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    double default_timeout_s)
  {
    node_ = node;
    if (default_timeout_s < 0.0) {
      default_timeout_s = 0.5;
    }

    constexpr const char * kParamName = "controller_input_timeout";
    if (!node->has_parameter(kParamName)) {
      node->declare_parameter<double>(kParamName, default_timeout_s);
    }

    double timeout_s = node->get_parameter(kParamName).as_double();
    if (timeout_s < 0.0) {
      RCLCPP_WARN(
        node->get_logger(),
        "controller_input_timeout < 0 (%f); treating as 0 (disabled).",
        timeout_s);
      timeout_s = 0.0;
    }

    timeout_ = rclcpp::Duration::from_seconds(timeout_s);
    timeout_ns_.store(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(timeout_s)).count(),
      std::memory_order_relaxed);

    RCLCPP_INFO(
      node->get_logger(),
      "InputWatchdog: timeout = %.3f s (0 disables the watchdog)",
      timeout_s);

    reset();
  }

  /**
   * @brief Reset to "never received" state. Call from on_activate so a
   * stale buffer from a previous activation cannot leak through.
   */
  void reset()
  {
    last_input_ns_.store(0, std::memory_order_relaxed);
    ever_received_.store(false, std::memory_order_relaxed);
    was_fresh_.store(false, std::memory_order_relaxed);
  }

  /**
   * @brief Mark that a fresh reference message has just been received.
   * Safe to call from a ROS subscription callback (non-RT).
   */
  void notify(const rclcpp::Time & now)
  {
    last_input_ns_.store(now.nanoseconds(), std::memory_order_release);
    ever_received_.store(true, std::memory_order_release);
  }

  /**
   * @brief True if a reference message has been received and (if a non-zero
   * timeout is configured) the most recent one is within the timeout window.
   *
   * Safe to call from the RT update loop.
   */
  bool is_fresh(const rclcpp::Time & now) const
  {
    if (!ever_received_.load(std::memory_order_acquire)) {
      return false;
    }

    const int64_t timeout_ns = timeout_ns_.load(std::memory_order_relaxed);
    if (timeout_ns == 0) {
      // Watchdog disabled: any received message is "fresh".
      return true;
    }

    const int64_t last_ns = last_input_ns_.load(std::memory_order_acquire);
    const int64_t now_ns = now.nanoseconds();

    // If the clock jumped backwards (e.g., sim time reset), treat the
    // message as fresh rather than stale to avoid spurious safe-stops.
    if (now_ns < last_ns) {
      return true;
    }

    return (now_ns - last_ns) <= timeout_ns;
  }

  /**
   * @brief Log a transition into/out of the stale state at most once per
   * second per direction. Call from update() after computing is_fresh().
   *
   * This is intentionally not done inside is_fresh() so the caller controls
   * which logger and what message text is used.
   *
   * @return The new fresh state (same as the passed-in value), so callers
   *   can chain it with assignment if desired.
   */
  bool log_state_transition(bool fresh, const rclcpp::Logger & logger) const
  {
    const bool prev = was_fresh_.exchange(fresh, std::memory_order_acq_rel);
    if (prev == fresh) {
      return fresh;
    }
    if (!fresh) {
      RCLCPP_WARN_THROTTLE(
        logger, *rclcpp::Clock::make_shared().get(), 1000,
        "Reference input is stale; commanding safe defaults until input resumes.");
    } else {
      RCLCPP_INFO(
        logger,
        "Reference input recovered; resuming normal control.");
    }
    return fresh;
  }

  /// Configured timeout. Zero means the watchdog is disabled.
  rclcpp::Duration timeout() const { return timeout_; }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

  rclcpp::Duration timeout_{0, 0};
  std::atomic<int64_t> timeout_ns_{0};

  std::atomic<int64_t> last_input_ns_{0};
  std::atomic<bool> ever_received_{false};
  mutable std::atomic<bool> was_fresh_{false};
};

}  // namespace general_controllers

#endif  // GENERAL_CONTROLLERS__INPUT_WATCHDOG_HPP_
