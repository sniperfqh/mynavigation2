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

#ifndef NAV2_CORE__TERMINAL_AWARE_CONTROLLER_HPP_
#define NAV2_CORE__TERMINAL_AWARE_CONTROLLER_HPP_

namespace nav2_core
{

class TerminalAwareController
{
public:
  virtual ~TerminalAwareController() = default;
  virtual bool isTerminalStopLatched() = 0;
  virtual bool isTerminalPositionAccurate() = 0;
};

}
// namespace nav2_core

#endif  // NAV2_CORE__TERMINAL_AWARE_CONTROLLER_HPP_
