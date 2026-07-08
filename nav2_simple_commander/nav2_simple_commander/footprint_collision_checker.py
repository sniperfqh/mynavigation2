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
This is a Python3 API for a Footprint Collision Checker.

It provides the needed methods to manipulate the coordinates
and calculate the cost of a Footprint

中文说明：
这个文件用当前 costmap 检查机器人 footprint 是否与障碍物相交。
核心思路是把 footprint 多边形边界采样成栅格点，读取每个点的代价值，
如果任意点达到 LETHAL_OBSTACLE，就认为该位姿发生碰撞。
"""

from math import cos, sin

from geometry_msgs.msg import Point32, Polygon
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator

NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
FREE_SPACE = 0


class FootprintCollisionChecker:
    """
    FootprintCollisionChecker.

    FootprintCollisionChecker Class for getting the cost
    and checking the collisions of a Footprint
    """

    def __init__(self):
        """Initialize the FootprintCollisionChecker Object."""
        # costmap_ 必须由 setCostmap() 注入；不在构造函数中绑定，便于复用 checker。
        self.costmap_ = None
        pass

    def footprintCost(self, footprint: Polygon):
        """
        Iterate over all the points in a footprint and check for collision.

        Args
        ----
            footprint (Polygon): The footprint to calculate the collision cost for

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points

        """
        # Track the maximum cost seen on the footprint boundary; 254 returns early.
        # 中文注解：记录 footprint 边界上见过的最大代价；254 直接提前返回。
        footprint_cost = 0.0
        x1 = 0.0
        y1 = 0.0

        # The first point is the closed polygon start and the last edge connects back to it.
        # 中文注解：第一个点作为闭合多边形的起点，最后还会连接回这个点。
        x0, y0 = self.worldToMapValidated(footprint.points[0].x, footprint.points[0].y)

        if x0 is None or y0 is None:
            return LETHAL_OBSTACLE

        xstart = x0
        ystart = y0

        for i in range(len(footprint.points) - 1):
            # Convert each edge endpoint to map cell coordinates; out-of-bounds is lethal.
            # 中文注解：逐条边转换到 map cell 坐标，越界时按 lethal 处理。
            x1, y1 = self.worldToMapValidated(
                footprint.points[i + 1].x, footprint.points[i + 1].y
            )

            if x1 is None or y1 is None:
                return LETHAL_OBSTACLE

            # Check the maximum cost among all sampled points on the current boundary edge.
            # 中文注解：检查当前边界线段上所有采样点的最大 cost。
            footprint_cost = max(float(self.lineCost(x0, x1, y0, y1)), footprint_cost)
            x0 = x1
            y0 = y1

            if footprint_cost == LETHAL_OBSTACLE:
                return footprint_cost

        # Close the final edge from the last point back to the first point.
        # 中文注解：闭合最后一条边：最后一个点 -> 第一个点。
        return max(float(self.lineCost(xstart, x1, ystart, y1)), footprint_cost)

    def lineCost(self, x0, x1, y0, y1, step_size=0.5):
        """
        Iterate over all the points along a line and check for collision.

        Args
        ----
            x0 (float): Abscissa of the initial point in map coordinates
            y0 (float): Ordinate of the initial point in map coordinates
            x1 (float): Abscissa of the final point in map coordinates
            y1 (float): Ordinate of the final point in map coordinates
            step_size (float): Optional, Increments' resolution, defaults to 0.5

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            line_cost (float): The maximum cost found in the line points

        """
        line_cost = 0.0
        point_cost = -1.0
        # LineIterator 负责按 step_size 在 map 坐标中沿边界采样。
        line_iterator = LineIterator(x0, y0, x1, y1, step_size)

        while line_iterator.isValid():
            # Convert the sampled point to an integer cell before reading its cost.
            # 中文注解：采样点转换成整数 cell 后读取代价。
            point_cost = self.pointCost(
                int(line_iterator.getX()), int(line_iterator.getY())
            )

            if point_cost == LETHAL_OBSTACLE:
                return point_cost

            if line_cost < point_cost:
                line_cost = point_cost

            line_iterator.advance()

        return line_cost

    def worldToMapValidated(self, wx: float, wy: float):
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float): world coordinate X
            wy (float): world coordinate Y

        Returns
        -------
            None: if coordinates are invalid
            tuple of int: mx, my (if coordinates are valid)
            mx (int): map coordinate X
            my (int): map coordinate Y

        """
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        # Reuse PyCostmap2D.worldToMap; callers are expected to pass in-map coordinates.
        # 中文注解：这里沿用 PyCostmap2D 的 worldToMap；调用方负责传入地图范围内坐标。
        return self.costmap_.worldToMap(wx, wy)

    def pointCost(self, x: int, y: int):
        """
        Get the cost of a point in the costmap using map coordinates XY.

        Args
        ----
            mx (int): map coordinate X
            my (int): map coordinate Y

        Returns
        -------
            np.uint8: cost of a point

        """
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap_.getCostXY(x, y)

    def setCostmap(self, costmap: PyCostmap2D):
        """
        Specify which costmap to use.

        Args
        ----
            costmap (PyCostmap2D): costmap to use in the object's methods

        Returns
        -------
            None

        """
        self.costmap_ = costmap
        return None

    def footprintCostAtPose(self, x: float, y: float, theta: float, footprint: Polygon):
        """
        Get the cost of a footprint at a specific Pose in map coordinates.

        Args
        ----
            x (float): map coordinate X
            y (float): map coordinate Y
            theta (float): absolute rotation angle of the footprint
            footprint (Polygon): the footprint to calculate its cost at the given Pose

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points

        """
        # Rotate local footprint coordinates by theta, then translate them to pose (x, y).
        # 中文注解：先按 theta 旋转 footprint 局部坐标，再平移到目标位姿 (x, y)。
        cos_th = cos(theta)
        sin_th = sin(theta)
        oriented_footprint = Polygon()

        for i in range(len(footprint.points)):
            new_pt = Point32()
            new_pt.x = x + (
                footprint.points[i].x * cos_th - footprint.points[i].y * sin_th
            )
            new_pt.y = y + (
                footprint.points[i].x * sin_th + footprint.points[i].y * cos_th
            )
            oriented_footprint.points.append(new_pt)

        return self.footprintCost(oriented_footprint)
