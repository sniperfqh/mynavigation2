#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2022 Afif Swaidan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a Python3 API for a line iterator.

It provides the ability to iterate
through the points of a line.

中文说明：
这个文件提供二维线段采样器，给 footprint 碰撞检测逐点扫描边界使用。
它不依赖 ROS，只负责在两点之间按 step_size 推进当前采样点。
"""

from cmath import sqrt


class LineIterator():
    """
    LineIterator.

    LineIterator Python3 API for iterating along the points of a given line
    """

    def __init__(self, x0, y0, x1, y1, step_size=1.0):
        """
        Initialize the LineIterator.

        Args
        ----
            x0 (float): Abscissa of the initial point
            y0 (float): Ordinate of the initial point
            x1 (float): Abscissa of the final point
            y1 (float): Ordinate of the final point
            step_size (float): Optional, Increments' resolution, defaults to 1

        Raises
        ------
            TypeError: When one (or more) of the inputs is not a number
            ValueError: When step_size is not a positive number

        """
        # Keep input validation strict to avoid passing ROS messages or strings by mistake.
        # 中文注解：输入校验保持严格，避免碰撞检测时把 ROS message 或字符串误传进来。
        if type(x0) not in [int, float]:
            raise TypeError("x0 must be a number (int or float)")

        if type(y0) not in [int, float]:
            raise TypeError("y0 must be a number (int or float)")

        if type(x1) not in [int, float]:
            raise TypeError("x1 must be a number (int or float)")

        if type(y1) not in [int, float]:
            raise TypeError("y1 must be a number (int or float)")

        if type(step_size) not in [int, float]:
            raise TypeError("step_size must be a number (int or float)")

        if step_size <= 0:
            raise ValueError("step_size must be a positive number")

        # Store endpoints, the current point, and step size; iteration starts at the origin.
        # 中文注解：保存端点、当前点和步长；当前点从起点开始。
        self.x0_ = x0
        self.y0_ = y0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0
        self.step_size_ = step_size

        # Non-vertical lines store slope and intercept; vertical lines only advance in y.
        # 中文注解：非竖直线保存斜率和截距；竖直线只沿 y 推进，不需要 m/b。
        if x1 != x0 and y1 != y0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        else:
            self.valid_ = False
            raise ValueError(
                "Line has zero length (All 4 points have same coordinates)")

    def isValid(self):
        """Check if line is valid."""
        return self.valid_

    def advance(self):
        """Advance to the next point in the line."""
        # x 方向可推进时优先推进 x，并用直线方程重新计算 y。
        if self.x1_ > self.x0_:
            if self.x_ < self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ + self.step_size_, self.x0_, self.x1_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        elif self.x1_ < self.x0_:
            if self.x_ > self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ - self.step_size_, self.x1_, self.x0_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        else:
            # Vertical segments keep x fixed and advance only along y.
            # 中文注解：竖直线段 x 不变，只按 y 的方向推进。
            if self.y1_ > self.y0_:
                if self.y_ < self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ + self.step_size_, self.y0_, self.y1_), 5)
                else:
                    self.valid_ = False
            elif self.y1_ < self.y0_:
                if self.y_ > self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ - self.step_size_, self.y1_, self.y0_), 5)
                else:
                    self.valid_ = False
            else:
                self.valid_ = False

    def getX(self):
        """Get the abscissa of the current point."""
        return self.x_

    def getY(self):
        """Get the ordinate of the current point."""
        return self.y_

    def getX0(self):
        """Get the abscissa of the initial point."""
        return self.x0_

    def getY0(self):
        """Get the ordinate of the intial point."""
        return self.y0_

    def getX1(self):
        """Get the abscissa of the final point."""
        return self.x1_

    def getY1(self):
        """Get the ordinate of the final point."""
        return self.y1_

    def get_line_length(self):
        """Get the length of the line."""
        # Return cmath.sqrt result to preserve the original implementation behavior.
        # 中文注解：返回复数 sqrt 的结果，保留原实现行为；测试中会按该类型断言。
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def clamp(self, n, min_n, max_n):
        """
        Clamp n to be between min_n and max_n.

        Args
        ----
            n (float): input value
            min_n (float): minimum value
            max_n (float): maximum value

        Returns
        -------
            n (float): input value clamped between given min and max

        """
        # clamp 保证最后一步不会越过线段终点。
        if n < min_n:
            return min_n
        elif n > max_n:
            return max_n
        else:
            return n
