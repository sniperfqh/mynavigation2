// Copyright (c) 2023, Samsung Research America
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

#ifndef NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
#define NAV2_CORE__ROUTE_EXCEPTIONS_HPP_


#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

/**
 * @class RouteException
 * @brief Base runtime exception for all route-processing failures.
 */
class RouteException : public std::runtime_error
{
public:
  explicit RouteException(const std::string & description)
  : std::runtime_error(description) {}
};

/**
 * @class OperationFailed
 * @brief Failure raised while applying a runtime route operation.
 */
class OperationFailed : public RouteException
{
public:
  explicit OperationFailed(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class NoValidRouteCouldBeFound
 * @brief The graph is valid but no route satisfies the request.
 */
class NoValidRouteCouldBeFound : public RouteException
{
public:
  explicit NoValidRouteCouldBeFound(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class TimedOut
 * @brief Route search exceeded its configured work or iteration budget.
 */
class TimedOut : public RouteException
{
public:
  explicit TimedOut(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class RouteTFError
 * @brief TF lookup or pose transformation required by routing failed.
 */
class RouteTFError : public RouteException
{
public:
  explicit RouteTFError(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class NoValidGraph
 * @brief The loaded route graph is missing, malformed, or unusable.
 */
class NoValidGraph : public RouteException
{
public:
  explicit NoValidGraph(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class IndeterminantNodesOnGraph
 * @brief Start or goal intent cannot be mapped unambiguously onto graph nodes.
 */
class IndeterminantNodesOnGraph : public RouteException
{
public:
  explicit IndeterminantNodesOnGraph(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class InvalidEdgeScorerUse
 * @brief An edge-cost scorer was invoked with an unsupported state or request.
 */
class InvalidEdgeScorerUse : public RouteException
{
public:
  explicit InvalidEdgeScorerUse(const std::string & description)
  : RouteException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
