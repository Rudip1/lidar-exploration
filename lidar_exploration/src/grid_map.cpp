#include "lidar_exploration/grid_map.hpp"

#include <cmath>

namespace lidar_exploration {

Cell GridMap::world_to_cell(const Eigen::Vector2d& p) const {
    const Eigen::Vector2d g = (p - origin) / resolution;
    return {static_cast<int>(std::floor(g.x())), static_cast<int>(std::floor(g.y()))};
}

double GridMap::area(CellState s) const {
    std::size_t n = 0;
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x) n += state({x, y}) == s;
    return static_cast<double>(n) * resolution * resolution;
}

GridMap pad(const GridMap& map, double margin) {
    const int m = static_cast<int>(std::ceil(margin / map.resolution));
    GridMap out(map.width + 2 * m, map.height + 2 * m, map.resolution,
                map.origin - Eigen::Vector2d::Constant(m * map.resolution));
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x) out.data[out.index({x + m, y + m})] = map.data[map.index({x, y})];
    return out;
}

motion_planning::OccupancyGrid obstacle_grid(const GridMap& map, bool unknown_is_obstacle) {
    motion_planning::OccupancyGrid grid(map.width, map.height, map.resolution, map.origin);
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x) {
            const CellState s = map.state({x, y});
            grid.set({x, y}, s == CellState::Occupied || (unknown_is_obstacle && s == CellState::Unknown));
        }
    return grid;
}

}  // namespace lidar_exploration
