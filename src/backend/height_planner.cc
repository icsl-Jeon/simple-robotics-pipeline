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
#include "my_robotics_library/backend/planners/height_planner.h"

using namespace my_robotics_library;
using namespace my_robotics_library::backend;
using namespace std::chrono;

void HeightPlanner::SetRobotPosition(
    const my_robotics_library::TimedPosition &robot_position) {
  robot_position_ = robot_position;
}

HeightMotionPlanningResult HeightPlanner::ComputeHeightMotion(
    const my_robotics_library::backend::HeightPlannerInput &planner_input) {

  return HeightMotionPlanningResult(planner_input.target_height,
                                    &robot_position_);
}

HeightMotionPlanningResult::HeightMotionPlanningResult(
    double planned_height, TimedPosition *position_ptr)
    : MotionPlanningResult(
          planned_height == 0 ? MotionPhase::kLanding : MotionPhase::kHovering,
          duration<double>(system_clock::now().time_since_epoch()).count()),
      planned_height_(planned_height), robot_position_ptr_(position_ptr) {}

Control HeightMotionPlanningResult::GenerateControl(double t) const {
  Control control;
  control.phase = GetMotionType();
  control.t = t;

  double error = planned_height_ - robot_position_ptr_->z;
  if (error < 0)
    return control;
  control.input = error;
  return control;
}

std::vector<TimedPosition>
HeightMotionPlanningResult::GetPlanningTrajectory(double t0, double tf) const {
  const int N = 30;
  double dt = (tf - t0) / N;
  std::vector<TimedPosition> trajectory;
  for (int n = 0; n < N; n++) {
    trajectory.push_back({t0 + (tf - t0) / N * n, 0, 0, planned_height_});
  }
  return trajectory;
}