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

using namespace my_robotics_library;

Wrapper::Wrapper() { state_history_.push_back({MotionPhase::kIdle}); }

void Wrapper::SetPosition(const my_robotics_library::TimedPosition &position) {
  sensor_information_.position = position;
  height_planner_.SetRobotPosition(position);
}

void Wrapper::SetTargetPosition(
    const std::optional<TimedPosition> &target_position) {
  sensor_information_.target_position = target_position;
}

void Wrapper::SetBatteryLevel(int level) {
  sensor_information_.battery_level = level;
}

void Wrapper::UpdateMonitor() {
  auto current_motion_phase = state_history_.back().motion_phase;

  if ((current_motion_phase != MotionPhase::kChasing &&
       current_motion_phase != MotionPhase::kExploration) ||
      !sensor_information_.target_position.has_value())
    monitor_.is_planning_visible = false;
  else
    monitor_.is_planning_visible = true;

  if (state_history_.back().motion_phase == MotionPhase::kHolding)
    monitor_.is_safe_for_short_horizon = true;
  else if (std::abs(sensor_information_.position.x) > 1)
    monitor_.is_safe_for_short_horizon = false;

  monitor_.is_battery_enough = sensor_information_.battery_level > 0;
}

MonitorEvent Wrapper::ReadMonitorEvent() const {
  if (!monitor_.is_battery_enough &&
      state_history_.back().motion_phase != MotionPhase::kLanding)
    return MonitorEvent::kLand;

  if (!monitor_.is_safe_for_short_horizon)
    return MonitorEvent::kHoldStop;

  switch (state_history_.back().motion_phase) {
  case MotionPhase::kChasing: {
    if (!monitor_.is_planning_visible)
      return kExplore;

    using namespace std::chrono;
    auto elapse_since_planning =
        duration<double>(system_clock::now().time_since_epoch()).count() -
        motion_planning_result_->GetRequestTime();
    if (elapse_since_planning > 0.2)
      return MonitorEvent::kChaseReplan;
  }
  case MotionPhase::kExploration: {
    if (monitor_.is_planning_visible)
      return kChaseReplan;
  }
  case MotionPhase::kHolding: {
    auto previous_state = state_history_[state_history_.size() - 2];
    if (monitor_.is_safe_for_short_horizon) {
      switch (previous_state.motion_phase) {
      case MotionPhase::kExploration:
        return MonitorEvent::kExplore;
      case MotionPhase::kChasing:
        return MonitorEvent::kChaseReplan;
      case MotionPhase::kHovering:
        return MonitorEvent::kHover;
      case MotionPhase::kLanding:
        return MonitorEvent::kLand;
      }
    }
  }
  }
  return MonitorEvent::kNone;
}

void Wrapper::OnTimerCallback() {
  UpdateMonitor();
  auto event_type = ReadMonitorEvent();
  if (event_type != MonitorEvent::kNone)
    state_history_.push_back(ProcessEvent(state_history_.back(), event_type));
}

void Wrapper::OnHoveringCommandCallback() {
  state_history_.push_back(HandleHovering(state_history_.back()));
}

void Wrapper::OnChasingCommandCallback() {
  state_history_.push_back(HandleChasingPlan(state_history_.back()));
}

Control Wrapper::GetControl() const {
  double current_time = 0.0;
  auto current_motion_phase = state_history_.back().motion_phase;

  if (current_motion_phase == MotionPhase::kIdle)
    return Control();
  else if (current_motion_phase == MotionPhase::kHolding)
    return Control{MotionPhase::kHolding, 0, 0};
  else if (current_motion_phase == MotionPhase::kExploration)
    return Control{MotionPhase::kExploration, 0, 0};
  else
    return motion_planning_result_->GenerateControl(current_time);
}

State Wrapper::ProcessEvent(const State &state, const MonitorEvent &event) {
  State new_state = state;
  switch (event) {
  case kHover:
    new_state = HandleHovering(state);
    break;
    
  case kLand:
    new_state = HandleLanding(state);
    break;

  case kHoldStop:
    new_state = HandleHoldStop(state);
    break;

  case kExplore:
    new_state = HandleExploration(state);
    break;

  case kChaseReplan:
    new_state = HandleChasingPlan(state);
    break;
  }
  return new_state;
};

State Wrapper::HandleLanding(const my_robotics_library::State &state) {
  auto new_state = state;
  new_state.motion_phase = MotionPhase::kLanding;

  backend::HeightPlannerInput input;
  input.target_height = 0.0;
  auto height_plan = height_planner_.ComputeHeightMotion(input);
  motion_planning_result_.reset(
      new backend::HeightMotionPlanningResult(height_plan));
  return new_state;
}

State Wrapper::HandleHoldStop(const my_robotics_library::State &state) {
  auto new_state = state;
  new_state.motion_phase = MotionPhase::kHolding;
  return new_state;
}

State Wrapper::HandleExploration(const my_robotics_library::State &state) {
  auto new_state = state;
  new_state.motion_phase = MotionPhase::kExploration;
  return new_state;
}

State Wrapper::HandleChasingPlan(const my_robotics_library::State &state) {
  if (!sensor_information_.target_position.has_value())
    return state;

  auto new_state = state;
  new_state.motion_phase = MotionPhase::kChasing;
  backend::ChasingPlannerInput input;
  input.target_position = sensor_information_.target_position.value();
  auto chasing_plan = chasing_planner_.ComputeChasingMotion(input);
  motion_planning_result_.reset(
      new backend::ChasingMotionPlanningResult(chasing_plan));
  return new_state;
}

State Wrapper::HandleHovering(const my_robotics_library::State &state) {
  auto new_state = state;
  new_state.motion_phase = MotionPhase::kHovering;

  backend::HeightPlannerInput input;
  input.target_height = parameter_.hovering_height;
  auto height_plan = height_planner_.ComputeHeightMotion(input);
  motion_planning_result_.reset(
      new backend::HeightMotionPlanningResult(height_plan));
  return new_state;
}
