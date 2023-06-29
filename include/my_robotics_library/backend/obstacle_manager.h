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

#ifndef SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_TARGET_PREDICTOR_H_
#define SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_TARGET_PREDICTOR_H_
#include "my_robotics_library/backend/types.h"
namespace my_robotics_library {
namespace backend {
class ObstacleManager {
public:
  ObstacleManager();
  double GetDistanceToObstacle(const TimedPosition &position) const;
};

} // namespace backend
} // namespace my_robotics_library

#endif // SIMPLE_ROBOTICS_FRONTEND_INCLUDE_CORE_TARGET_PREDICTOR_H_
