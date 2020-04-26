/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#ifndef MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_DESTINATION_H_
#define MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_DESTINATION_H_

#include <string>

#include "modules/common/proto/geometry.pb.h"
#include "modules/planning/tasks/traffic_decider/traffic_rule.h"

namespace apollo {
namespace planning {

/**
 * This class decides whether we should stop for destination.
 * situation.
 */
class Destination : public TrafficRule {
 public:
  explicit Destination(const TrafficRuleConfig& config);
  virtual ~Destination() = default;

  common::Status ApplyRule(Frame* const frame,
                 ReferenceLineInfo* const reference_line_info);

 private:
  void MakeDecisions(Frame* const frame,
                     ReferenceLineInfo* const reference_line_info);
  int BuildStopDecision(Frame* const frame,
                         ReferenceLineInfo* const reference_line_info);
  int Stop(Frame* const frame,
           ReferenceLineInfo* const reference_line_info,
           const std::string lane_id,
           const double lane_s);
  bool CheckPullOver(ReferenceLineInfo* const reference_line_info,
                     const std::string lane_id,
                     const double lane_s,
                     common::PointENU* dest_point);
  int PullOver(common::PointENU* const dest_point);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TASKS_TRAFFIC_DECIDER_DESTINATION_H_
