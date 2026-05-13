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

#include <cmath>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "general_controllers/safe_stopper.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

class SafeStopperTest : public ::testing::Test
{
protected:
  // Build a vector of LoanedCommandInterface backed by storage owned by the
  // test fixture. Each interface has a (joint, kind, value) triple.
  std::vector<hardware_interface::LoanedCommandInterface> make_loaned(
    const std::vector<std::pair<std::string, std::string>> & joint_iface,
    std::vector<double> & values)
  {
    storage_.clear();
    storage_.reserve(joint_iface.size());
    values.assign(joint_iface.size(), 0.0);

    for (size_t i = 0; i < joint_iface.size(); ++i) {
      storage_.emplace_back(joint_iface[i].first, joint_iface[i].second, &values[i]);
    }

    std::vector<hardware_interface::LoanedCommandInterface> loaned;
    loaned.reserve(storage_.size());
    for (auto & cmd : storage_) {
      loaned.emplace_back(cmd);
    }
    return loaned;
  }

private:
  std::vector<hardware_interface::CommandInterface> storage_;
};

TEST_F(SafeStopperTest, ZeroesVelocityAndEffortInterfaces)
{
  std::vector<double> values;
  auto cmds = make_loaned(
    {{"j1", "velocity"}, {"j2", "effort"}, {"j3", "velocity"}}, values);

  // Set non-zero current values to prove apply() overrides them.
  for (auto & c : cmds) {
    c.set_value(7.5);
  }

  general_controllers::SafeStopper stopper;
  stopper.prepare(cmds);
  stopper.apply(cmds);

  EXPECT_DOUBLE_EQ(values[0], 0.0);
  EXPECT_DOUBLE_EQ(values[1], 0.0);
  EXPECT_DOUBLE_EQ(values[2], 0.0);
}

TEST_F(SafeStopperTest, LatchesPositionInterfacesOnFirstApply)
{
  std::vector<double> values;
  auto cmds = make_loaned(
    {{"j1", "position"}, {"j2", "velocity"}, {"j3", "position"}}, values);

  // Pre-set "last commanded position" for the position interfaces.
  cmds[0].set_value(1.234);
  cmds[1].set_value(99.0);
  cmds[2].set_value(-0.5);

  general_controllers::SafeStopper stopper;
  stopper.prepare(cmds);
  stopper.apply(cmds);

  // Positions latched at their pre-apply values.
  EXPECT_DOUBLE_EQ(values[0], 1.234);
  EXPECT_DOUBLE_EQ(values[2], -0.5);
  // Velocity zeroed.
  EXPECT_DOUBLE_EQ(values[1], 0.0);

  // A subsequent apply re-issues the same latched positions even if
  // something else perturbs the underlying values in between.
  values[0] = 999.0;
  values[2] = -888.0;
  stopper.apply(cmds);
  EXPECT_DOUBLE_EQ(values[0], 1.234);
  EXPECT_DOUBLE_EQ(values[2], -0.5);
  EXPECT_DOUBLE_EQ(values[1], 0.0);
}

TEST_F(SafeStopperTest, ResetReleasesLatchSoNextApplyRecaptures)
{
  std::vector<double> values;
  auto cmds = make_loaned({{"j1", "position"}}, values);

  cmds[0].set_value(2.0);
  general_controllers::SafeStopper stopper;
  stopper.prepare(cmds);
  stopper.apply(cmds);
  EXPECT_DOUBLE_EQ(values[0], 2.0);

  // Simulate normal control writing a new position, then a fresh stale
  // episode after reset(): the new latch should capture that new position.
  cmds[0].set_value(3.5);
  stopper.reset();
  stopper.apply(cmds);
  EXPECT_DOUBLE_EQ(values[0], 3.5);
}

TEST_F(SafeStopperTest, NaNPositionFallsBackToZero)
{
  std::vector<double> values;
  auto cmds = make_loaned({{"j1", "position"}}, values);

  cmds[0].set_value(std::numeric_limits<double>::quiet_NaN());

  general_controllers::SafeStopper stopper;
  stopper.prepare(cmds);
  stopper.apply(cmds);

  EXPECT_DOUBLE_EQ(values[0], 0.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
