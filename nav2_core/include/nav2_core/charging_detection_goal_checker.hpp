// Copyright (c) 2026 zpy
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

#ifndef NAV2_CORE__CHARGING_DETECTION_GOAL_CHECKER_HPP_
#define NAV2_CORE__CHARGING_DETECTION_GOAL_CHECKER_HPP_

namespace nav2_core
{

// 充电电流感知 GoalChecker 可选接口：实现方在检测到新鲜充电电流时返回 true。
// Controller Server 在到点判定的入口短路完成，接触上电后不再等待几何容差与停稳复核。
class ChargingDetectionGoalChecker
{
public:
  virtual ~ChargingDetectionGoalChecker() = default;
  virtual bool chargingCurrentDetected() const = 0;
};

}
// namespace nav2_core

#endif  // NAV2_CORE__CHARGING_DETECTION_GOAL_CHECKER_HPP_