/*******************************************************************************
 *
 * Copyright 2023 Boseong Felipe Jeon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *******************************************************************************/

#include "my_robotics_library/frontend/wrapper.h"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>

using namespace my_robotics_library;

TEST(CommandEvent, HoveringRequestTest) {
  Wrapper wrapper;
  auto control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kIdle);

  // Assume we got hovering request
  wrapper.OnHoveringCommandCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHovering);

  // Assume drone is hovered a little bit, which needs a less actuation
  wrapper.SetPosition({0.1, 0, 0, 0.2});
  EXPECT_LT(wrapper.GetControl().input, control.input);
}

TEST(MonitorEvent, ShouldLandWhenBatteryLow) {
  Wrapper wrapper;

  wrapper.OnHoveringCommandCallback();
  auto control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHovering);

  wrapper.SetBatteryLevel(0);
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kLanding);
}

TEST(MonitorEvent, ShouldHoldStopWhenNotSafe) {
  Wrapper wrapper;

  wrapper.OnHoveringCommandCallback();
  auto control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHovering);

  TimedPosition dangerous_position{0, 2, 0, 0};
  wrapper.SetPosition(dangerous_position);
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHolding);

  TimedPosition safe_position{1, 0, 0, 0};
  wrapper.SetPosition(safe_position);
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHovering);
}

TEST(MonitorEvent, ChasingAndExploration) {
  Wrapper wrapper;

  wrapper.OnHoveringCommandCallback();
  auto control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kHovering);

  // target position is not set. Chasing command does not take effect
  wrapper.OnChasingCommandCallback();
  control = wrapper.GetControl();
  EXPECT_NE(control.phase, MotionPhase::kChasing);

  // Chasing command takes effect after receiving target position
  wrapper.SetTargetPosition(TimedPosition{1, 0, 0, 0});
  wrapper.OnChasingCommandCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kChasing);
  EXPECT_EQ(control.input, 0.0);

  // Chasing should re-planned after 0.2 sec
  wrapper.SetTargetPosition(TimedPosition{2, 1, 0, 0});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kChasing);
  EXPECT_EQ(control.input, 1.0);

  // When target is lost, change to exploration phase
  wrapper.SetTargetPosition(std::nullopt);
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kExploration);

  // When target is re-detected, switch to exploration
  wrapper.SetTargetPosition(TimedPosition{3, 2, 0, 0});
  wrapper.OnTimerCallback();
  control = wrapper.GetControl();
  EXPECT_EQ(control.phase, MotionPhase::kChasing);
  EXPECT_EQ(control.input, 2.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}