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

import unittest
from cmath import sqrt
from nav2_simple_commander.line_iterator import LineIterator


class TestLineIterator(unittest.TestCase):
    """Unit tests for the geometry-only LineIterator helper.

    中文注解：LineIterator 是 footprint 碰撞检测的基础工具，这里验证类型校验、步进和边界钳制。
    """

    def test_type_error(self):
        # Test if a type error raised when passing invalid arguements types
        # 中文注解：传入非数字参数时必须抛出 TypeError。
        self.assertRaises(TypeError, LineIterator, 0, 0, '10', 10, '1')

    def test_value_error(self):
        # Test if a value error raised when passing negative or zero step_size
        # 中文注解：step_size 小于等于 0 时必须抛出 ValueError。
        self.assertRaises(ValueError, LineIterator, 0, 0, 10, 10, -2)
        # Test if a value error raised when passing zero length line
        # 中文注解：起点和终点完全相同的零长度线段也必须抛出 ValueError。
        self.assertRaises(ValueError, LineIterator, 2, 2, 2, 2, 1)

    def test_get_xy(self):
        # Test if the initial and final coordinates are returned correctly
        # 中文注解：验证 getter 返回的起点和终点坐标与构造参数一致。
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.getX0(), 0)
        self.assertEqual(lt.getY0(), 0)
        self.assertEqual(lt.getX1(), 5)
        self.assertEqual(lt.getY1(), 5)

    def test_line_length(self):
        # Test if the line length is calculated correctly
        # 中文注解：验证线段长度计算保持与 cmath.sqrt 结果一致。
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.get_line_length(), sqrt(pow(5, 2) + pow(5, 2)))

    def test_straight_line(self):
        # Test if the calculations are correct for y = x
        # 中文注解：验证正斜率 1 的线段按 x/y 同步递增。
        lt = LineIterator(0, 0, 5, 5, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

        # Test if the calculations are correct for y = 2x (positive slope)
        # 中文注解：验证更大的正斜率会按直线方程更新 y。
        lt = LineIterator(0, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (i*2))
            lt.advance()
            i += 1

        # Test if the calculations are correct for y = -2x (negative slope)
        # 中文注解：验证负斜率会按直线方程递减 y。
        lt = LineIterator(0, 0, 5, -10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (-i*2))
            lt.advance()
            i += 1

    def test_hor_line(self):
        # Test if the calculations are correct for y = 0x+b (horizontal line)
        # 中文注解：验证水平线 y 保持不变，只推进 x。
        lt = LineIterator(0, 10, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0())
            lt.advance()
            i += 1

    def test_ver_line(self):
        # Test if the calculations are correct for x = n (vertical line)
        # 中文注解：验证竖直线 x 保持不变，只推进 y。
        lt = LineIterator(5, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0())
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

    def test_clamp(self):
        # Test if the increments are clamped to avoid crossing the final points
        # 中文注解：验证步长大于线段长度时不会越过终点。
        # when step_size is large with respect to line length
        # 中文注解：最后一次推进会被 clamp 到终点坐标。
        lt = LineIterator(0, 0, 5, 5, 10)
        self.assertEqual(lt.getX(), 0)
        self.assertEqual(lt.getY(), 0)
        lt.advance()
        while lt.isValid():
            self.assertEqual(lt.getX(), 5)
            self.assertEqual(lt.getY(), 5)
            lt.advance()


if __name__ == '__main__':
    unittest.main()
