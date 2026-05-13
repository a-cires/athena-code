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

#ifndef GENERAL_CONTROLLERS__SAFE_STOPPER_HPP_
#define GENERAL_CONTROLLERS__SAFE_STOPPER_HPP_

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"

namespace general_controllers
{

/**
 * @brief Applies safe-default commands to a controller's command interfaces
 * when the reference input is stale (or has never arrived).
 *
 * Per-interface safe defaults:
 *   - "velocity" or "effort" interfaces: 0.0
 *   - "position" interface: latch the last commanded position. The latch is
 *     captured on the first apply() call after a reset(); subsequent apply()
 *     calls re-issue the latched values. Falling back to NaN-safe behavior:
 *     if the previous command was NaN (e.g. controller just activated), the
 *     latched value is read directly from the command interface itself.
 *   - Anything else: 0.0 (conservative default).
 *
 * Realtime safety: prepare()/reset() may allocate (called from on_activate
 * or on rare stale-onset transitions); apply() is allocation-free.
 */
class SafeStopper
{
public:
  enum class InterfaceKind : std::uint8_t
  {
    Velocity,
    Position,
    Effort,
    Other,
  };

  /**
   * @brief Cache the kind of each command interface. Call from on_activate
   * after the interfaces have been assigned.
   */
  void prepare(const std::vector<hardware_interface::LoanedCommandInterface> & cmds)
  {
    kinds_.clear();
    kinds_.reserve(cmds.size());
    latched_.assign(cmds.size(), 0.0);
    have_latch_ = false;

    for (const auto & cmd : cmds) {
      const std::string & iface = cmd.get_interface_name();
      if (iface == "velocity") {
        kinds_.push_back(InterfaceKind::Velocity);
      } else if (iface == "position") {
        kinds_.push_back(InterfaceKind::Position);
      } else if (iface == "effort") {
        kinds_.push_back(InterfaceKind::Effort);
      } else {
        kinds_.push_back(InterfaceKind::Other);
      }
    }
  }

  /**
   * @brief Drop any latched positions. Call when the watchdog transitions
   * back to "fresh" so the next stale episode latches a new position.
   */
  void reset() { have_latch_ = false; }

  /**
   * @brief Apply safe defaults to all command interfaces.
   *
   * On the first call after reset() (or after prepare()), this latches the
   * current commanded position for any position interface. Subsequent
   * calls re-issue the latched positions and zero everything else. This
   * matches "hold last commanded position; zero velocity/effort".
   */
  void apply(std::vector<hardware_interface::LoanedCommandInterface> & cmds)
  {
    if (cmds.size() != kinds_.size()) {
      // Interface set changed under us; re-prepare.
      prepare(cmds);
    }

    if (!have_latch_) {
      for (std::size_t i = 0; i < cmds.size(); ++i) {
        if (kinds_[i] == InterfaceKind::Position) {
          double current = cmds[i].get_value();
          // Guard against NaN (e.g., interfaces freshly activated and never
          // commanded). Default to 0 in that case, which is conservative;
          // controllers that need a different fallback can pre-seed the
          // command interface in on_activate.
          if (!std::isfinite(current)) {
            current = 0.0;
          }
          latched_[i] = current;
        }
      }
      have_latch_ = true;
    }

    for (std::size_t i = 0; i < cmds.size(); ++i) {
      switch (kinds_[i]) {
        case InterfaceKind::Position:
          cmds[i].set_value(latched_[i]);
          break;
        case InterfaceKind::Velocity:
        case InterfaceKind::Effort:
        case InterfaceKind::Other:
        default:
          cmds[i].set_value(0.0);
          break;
      }
    }
  }

  std::size_t size() const { return kinds_.size(); }

private:
  std::vector<InterfaceKind> kinds_;
  std::vector<double> latched_;
  bool have_latch_ = false;
};

}  // namespace general_controllers

#endif  // GENERAL_CONTROLLERS__SAFE_STOPPER_HPP_
