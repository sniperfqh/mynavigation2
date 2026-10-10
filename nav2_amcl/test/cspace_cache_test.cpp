#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"

map_t * makeMap(int width, int height)
{
  map_t * map = map_alloc();
  map->size_x = width;
  map->size_y = height;
  map->scale = 0.01;
  map->cells = static_cast<map_cell_t *>(std::calloc(width * height, sizeof(map_cell_t)));
  for (int i = 0; i < width * height; ++i)
  {
    map->cells[i].occ_state = -1;
  }
  return map;
}

int main(int argc, char ** argv)
{
  if (argc == 3)
  {
    std::ifstream image(argv[1], std::ios::binary);
    std::string magic;
    int width, height, max_value;
    image >> magic >> width >> height >> max_value;
    image.get();
    if (magic != "P5" || max_value != 255 || width <= 0 || height <= 0)
    {
      return 1;
    }
    map_t * map = makeMap(width, height);
    for (int row = 0; row < height; ++row)
    {
      for (int column = 0; column < width; ++column)
      {
        const int pixel = image.get();
        if (pixel < 0)
        {
          map_free(map);
          return 2;
        }
        map->cells[MAP_INDEX(map, column, height - 1 - row)].occ_state = pixel < 100 ? 1 : -1;
      }
    }
    const auto begin = std::chrono::steady_clock::now();
    map_update_cspace(map, 2.0);
    const double duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    std::ofstream snapshot(argv[2], std::ios::binary);
    for (int i = 0; i < width * height; ++i)
    {
      const double value = map->cells[i].occ_dist;
      snapshot.write(reinterpret_cast<const char *>(&value), sizeof(value));
    }
    std::cout << "cells=" << width * height << ", compute_s=" << duration << std::endl;
    map_free(map);
    return snapshot.good() ? 0 : 3;
  }
  map_t * map = makeMap(7, 7);
  map->cells[MAP_INDEX(map, 3, 3)].occ_state = 1;
  nav2_amcl::LikelihoodFieldModel first(0.8, 0.05, 0.2, 0.03, 180, map);
  for (int y = 0; y < 7; ++y)
  {
    for (int x = 0; x < 7; ++x)
    {
      const double expected = std::min(0.03, std::hypot(x - 3, y - 3) * 0.01);
      if (std::abs(map->cells[MAP_INDEX(map, x, y)].occ_dist - expected) > 1e-12)
      {
        map_free(map);
        return 4;
      }
    }
  }
  nav2_amcl::LikelihoodFieldModel changed(0.8, 0.05, 0.2, 0.05, 180, map);
  const bool updated = std::abs(map->cells[0].occ_dist - std::sqrt(18.0) * 0.01) < 1e-12;
  map_free(map);
  return updated ? 0 : 5;
}
