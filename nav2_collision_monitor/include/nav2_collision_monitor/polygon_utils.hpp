// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__POLYGON_UTILS_HPP_
#define NAV2_COLLISION_MONITOR__POLYGON_UTILS_HPP_

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"

#include "nav2_util/array_parser.hpp"

#include "nav2_collision_monitor/types.hpp"

namespace nav2_collision_monitor
{

//ooii 官方 polygon_utils.hpp：几何辅助函数，供 Polygon/Circle/ExclusionZone 共用

/**
 * @brief Parses a VVF string ("[[x1, y1], [x2, y2], ...]") into polygon vertices.
 */
inline bool parsePolygonPoints(
  const std::string & points_str,
  std::size_t min_vertices,
  std::vector<Point> & out,
  std::string & error_msg)
{
  std::string parse_error;
  const std::vector<std::vector<float>> vvf = nav2_util::parseVVF(points_str, parse_error);
  if (!parse_error.empty()) {
    error_msg = "error parsing points '" + points_str + "': " + parse_error;
    return false;
  }
  if (vvf.size() < min_vertices) {
    error_msg = "polygon must have at least " + std::to_string(min_vertices) + " vertices";
    return false;
  }

  std::vector<Point> parsed;
  parsed.reserve(vvf.size());
  for (const std::vector<float> & v : vvf) {
    if (v.size() != 2) {
      error_msg = "each point must be a pair of numbers [x, y]";
      return false;
    }
    Point point;
    point.x = v[0];
    point.y = v[1];
    parsed.push_back(point);
  }

  out = std::move(parsed);
  return true;
}

/**
 * @brief Transforms polygon vertices by a tf2 transform on the z = 0 plane.
 */
inline void transformPolygonPoints(
  const tf2::Transform & tf,
  const std::vector<Point> & in,
  std::vector<Point> & out)
{
  out.resize(in.size());
  for (std::size_t i = 0; i < in.size(); ++i) {
    const tf2::Vector3 p_b = tf * tf2::Vector3(in[i].x, in[i].y, 0.0);
    out[i].x = p_b.x();
    out[i].y = p_b.y();
  }
}

/**
 * @brief Ray-crossing test: is (x, y) inside the polygon formed by `poly`?
 * @note 用户 nav2_util 没有 isPointInsidePolygon，这里内联实现
 *       （算法同 Polygon::isPointInside，Shimrat A112）。
 */
inline bool isPointInsidePolygon(
  double x, double y,
  const std::vector<Point> & poly)
{
  const int poly_size = static_cast<int>(poly.size());
  if (poly_size < 3) {
    return false;
  }

  bool res = false;  // Final result, initialized with already inverted value
  int i = poly_size - 1;
  for (int j = 0; j < poly_size; j++) {
    if ((y <= poly[i].y) == (y > poly[j].y)) {
      const double x_inter = poly[i].x +
        (y - poly[i].y) * (poly[j].x - poly[i].x) / (poly[j].y - poly[i].y);
      if (x_inter > x) {
        res = !res;
      }
    }
    i = j;
  }
  return res;
}

/**
 * @brief Rasterises a circle centred at the origin into an N-gon approximation.
 */
inline std::vector<Point> circleToPolygon(double radius, int edges = 16)
{
  std::vector<Point> poly;
  poly.reserve(static_cast<std::size_t>(edges));
  const double angle_increment = 2.0 * M_PI / edges;
  for (int i = 0; i < edges; ++i) {
    const double angle = angle_increment * i;
    Point p;
    p.x = radius * std::cos(angle);
    p.y = radius * std::sin(angle);
    poly.push_back(p);
  }
  return poly;
}

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__POLYGON_UTILS_HPP_
