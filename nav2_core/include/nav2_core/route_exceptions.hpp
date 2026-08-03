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

// 中文：本文件定义 nav2_route 计算、图加载、TF 和运行操作的分层异常类型，供 Route Server 精确映射结果。

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

/**
 * @class RouteException
 * @brief Base runtime exception for all route-processing failures.
 * 中文：所有 Route 专用异常的共同基类；未被更具体 catch 匹配的路由错误可由 Route Server 统一收口。
 */
class RouteException : public std::runtime_error
{
public:
  // 中文：保存底层失败描述，供服务器日志和 Action／Service 错误结果读取。
  explicit RouteException(const std::string & description)
  : std::runtime_error(description) {}
};

/**
 * @class OperationFailed
 * @brief Failure raised while applying a runtime route operation.
 * 中文：路线运行操作失败，例如碰撞监控或外部操作客户端无法完成当前处理。
 */
class OperationFailed : public RouteException
{
public:
  // 中文：沿用 RouteException 的文本载荷，同时保留可被独立捕获的具体类型。
  explicit OperationFailed(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class NoValidRouteCouldBeFound
 * @brief The graph is valid but no route satisfies the request.
 * 中文：图本身可用，但在给定起终点和约束下没有找到有效 Route。
 */
class NoValidRouteCouldBeFound : public RouteException
{
public:
  // 中文：把无路径原因上交 Route Server，由其映射为专用错误码。
  explicit NoValidRouteCouldBeFound(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class TimedOut
 * @brief Route search exceeded its configured work or iteration budget.
 * 中文：路由搜索达到时间、迭代或其他工作预算，尚未得到可接受结果。
 */
class TimedOut : public RouteException
{
public:
  // 中文：保留超时上下文，便于上层区分“无路可走”和“计算未完成”。
  explicit TimedOut(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class RouteTFError
 * @brief TF lookup or pose transformation required by routing failed.
 * 中文：起点、目标或机器人 Pose 无法变换到 Route 图坐标系时抛出。
 */
class RouteTFError : public RouteException
{
public:
  // 中文：错误文本通常包含目标 Frame 或具体 TF 查询失败原因。
  explicit RouteTFError(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class NoValidGraph
 * @brief The loaded route graph is missing, malformed, or unusable.
 * 中文：Route 图不存在、结构无效、边节点引用错误或不能用于搜索。
 */
class NoValidGraph : public RouteException
{
public:
  // 中文：将图验证失败原因交给服务器，避免继续进入搜索或跟踪阶段。
  explicit NoValidGraph(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class IndeterminantNodesOnGraph
 * @brief Start or goal intent cannot be mapped unambiguously onto graph nodes.
 * 中文：起点／目标在图上的候选节点无法确定。类名保留上游现有拼写以维持公开接口兼容。
 */
class IndeterminantNodesOnGraph : public RouteException
{
public:
  // 中文：保存节点意图提取阶段的具体歧义或缺失说明。
  explicit IndeterminantNodesOnGraph(const std::string & description)
  : RouteException(description) {}
};

/**
 * @class InvalidEdgeScorerUse
 * @brief An edge-cost scorer was invoked with an unsupported state or request.
 * 中文：边评分器的调用条件、输入状态或使用位置不符合其契约。
 */
class InvalidEdgeScorerUse : public RouteException
{
public:
  // 中文：错误描述由具体评分器生成，Route Server 可据类型返回明确失败原因。
  explicit InvalidEdgeScorerUse(const std::string & description)
  : RouteException(description) {}
};

}  // namespace nav2_core

#endif  // NAV2_CORE__ROUTE_EXCEPTIONS_HPP_
