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

#ifndef SIMPLE_ROBOTICS_FRONTEND_TYPES_H
#define SIMPLE_ROBOTICS_FRONTEND_TYPES_H
#include <vector>
namespace my_robotics_library {

struct Parameter {
  double hovering_height{1.0};
};

enum MotionPhase {
  kLanding,
  kHolding,
  kExploration,
  kChasing,
  kHovering,
  kIdle
};

struct TimedPosition {
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TimedVelocity {
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Control {
  MotionPhase phase{kIdle};
  double t{0.0};
  double input{0.0};
};

class MotionPlanningResult {

public:
  MotionPlanningResult(MotionPhase motion_phase, double t_request)
      : motion_type_(motion_phase), t_request_(t_request){};
  virtual Control GenerateControl(double t) const = 0;
  virtual std::vector<TimedPosition> GetPlanningTrajectory(double t0,
                                                           double tf) const = 0;
  MotionPhase GetMotionType() const { return motion_type_; };
  double GetRequestTime() const { return t_request_; }

private:
  MotionPhase motion_type_;
  double t_request_;
};

} // namespace my_robotics_library

#endif // SIMPLE_ROBOTICS_FRONTEND_TYPES_H
