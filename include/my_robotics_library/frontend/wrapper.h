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

#ifndef SIMPLE_ROBOTICS_FRONTEND_INCLUDE_FRONTEND_WRAPPER_H_
#define SIMPLE_ROBOTICS_FRONTEND_INCLUDE_FRONTEND_WRAPPER_H_

#include "cmath"

#include "my_robotics_library/backend/obstacle_manager.h"
#include "my_robotics_library/backend/planners/chasing_planner.h"
#include "my_robotics_library/backend/planners/height_planner.h"

#include <memory>
#include <optional>

#include "my_robotics_library/backend/types.h"

namespace my_robotics_library {
struct Monitor {
  bool is_planning_visible{false};
  bool is_safe_for_short_horizon{true};
  bool is_battery_enough{true};
};

struct SensorInformation {
  TimedVelocity velocity;
  TimedPosition position;
  std::optional<TimedPosition> target_position;
  int battery_level{1};
};

enum MonitorEvent { kHover, kLand, kHoldStop, kExplore, kChaseReplan, kNone };

struct State {
  MotionPhase motion_phase{MotionPhase::kIdle};
};

class Wrapper {
public:
  Wrapper();

  void SetVelocity(const TimedVelocity &velocity);
  void SetPosition(const TimedPosition &position);
  void SetTargetPosition(const std::optional<TimedPosition> &target_position);
  void SetBatteryLevel(int level);

  Control GetControl() const;

  void OnTimerCallback();
  void OnHoveringCommandCallback();
  void OnChasingCommandCallback();

private:
  Parameter parameter_;
  SensorInformation sensor_information_;
  Monitor monitor_;
  std::vector<State> state_history_;
  std::shared_ptr<MotionPlanningResult> motion_planning_result_;

  backend::HeightPlanner height_planner_;
  backend::ChasingPlanner chasing_planner_;

  void UpdateMonitor();
  MonitorEvent ReadMonitorEvent() const;

  State ProcessEvent(const State &state, const MonitorEvent &event);

  State HandleLanding(const State &state);
  State HandleHoldStop(const State &state);
  State HandleExploration(const State &state);
  State HandleChasingPlan(const State &state);
  State HandleHovering(const State &state);
};
} // namespace my_robotics_library

#endif // SIMPLE_ROBOTICS_FRONTEND_INCLUDE_FRONTEND_WRAPPER_H_
