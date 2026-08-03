/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  Copyright (c) 2019, Intel Corporation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NAV2_CORE__EXCEPTIONS_HPP_
#define NAV2_CORE__EXCEPTIONS_HPP_

// 中文：本文件提供规划、平滑和控制插件共享的运行时异常基类，便于服务器统一终止当前任务。

#include <stdexcept>
#include <string>
#include <memory>

namespace nav2_core
{

/**
 * @class PlannerException
 * @brief Generic exception propagated by Nav2 planning and path-processing plugins.
 * 中文：虽然名称包含 Planner，该异常也被 Controller、Smoother 和路径变换辅助逻辑复用。
 * 中文：服务器通常按引用捕获它，把 `what()` 文本写入日志并映射为对应 Action 失败或重试分支。
 */
class PlannerException : public std::runtime_error
{
public:
  // 中文：把调用方提供的错误描述交给 std::runtime_error 保存，what() 将返回同一说明文本。
  explicit PlannerException(const std::string description)
  : std::runtime_error(description) {}
  // 中文：保留共享指针别名供需要长期保存异常对象的代码使用；常规错误路径通常直接按值抛出。
  using Ptr = std::shared_ptr<PlannerException>;
};

}  // namespace nav2_core

#endif  // NAV2_CORE__EXCEPTIONS_HPP_
