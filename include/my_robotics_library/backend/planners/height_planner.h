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

#ifndef SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_Height_PLANNER_H_
#define SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_Height_PLANNER_H_

#include "chrono"
#include "my_robotics_library/backend/types.h"

namespace my_robotics_library {
namespace backend {

struct HeightPlannerInput {
  double target_height{1.0};
};

class HeightMotionPlanningResult : public MotionPlanningResult {
public:
  HeightMotionPlanningResult(double planned_height,
                             TimedPosition *position_ptr);
  Control GenerateControl(double t) const;
  std::vector<TimedPosition> GetPlanningTrajectory(double t0, double tf) const;

private:
  double planned_height_{0.0};
  TimedPosition *robot_position_ptr_;
};

class HeightPlanner {
public:
  HeightPlanner() = default;
  void SetRobotPosition(const TimedPosition &robot_position);
  HeightMotionPlanningResult
  ComputeHeightMotion(const HeightPlannerInput &planner_input);

private:
  TimedPosition robot_position_;
};

} // namespace backend
} // namespace my_robotics_library

#endif // SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_Height_PLANNER_H_
