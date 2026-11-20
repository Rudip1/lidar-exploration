// Three-state occupancy map in the ROS convention (nav_msgs/OccupancyGrid): -1 unknown, 0..100 occupancy
// probability, row-major with cell (x, y) at index y * width + x and cell (0, 0) starting at `origin`.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "motion_planning/grid.hpp"

namespace lidar_exploration {

using motion_planning::Cell;

enum class CellState { Unknown, Free, Occupied };

struct GridMap {
    int width = 0, height = 0;
    double resolution = 0.05;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    std::vector<std::int8_t> data;  // -1 unknown, 0..100

    static constexpr int kOccupiedThreshold = 50;  // occupancy >= 50 is an obstacle

    GridMap() = default;
    GridMap(int w, int h, double res, const Eigen::Vector2d& o)
        : width(w), height(h), resolution(res), origin(o), data(static_cast<std::size_t>(w) * h, -1) {}

    bool in_bounds(const Cell& c) const { return c.x >= 0 && c.y >= 0 && c.x < width && c.y < height; }
    std::size_t index(const Cell& c) const { return static_cast<std::size_t>(c.y) * width + c.x; }
    // Outside the map counts as unknown.
    CellState state(const Cell& c) const {
        if (!in_bounds(c)) return CellState::Unknown;
        const int v = data[index(c)];
        return v < 0 ? CellState::Unknown : (v >= kOccupiedThreshold ? CellState::Occupied : CellState::Free);
    }
    Cell world_to_cell(const Eigen::Vector2d& p) const;
    Eigen::Vector2d cell_center(const Cell& c) const {
        return origin + resolution * Eigen::Vector2d(c.x + 0.5, c.y + 0.5);
    }
    // Area [m^2] of the cells in the given state.
    double area(CellState s) const;
};

// The map with `margin` metres of unknown cells added on every side. SLAM maps (slam_toolbox) end where the
// observations end, so free space can touch the map edge; after padding, that edge is a frontier like any other.
GridMap pad(const GridMap& map, double margin);

// Obstacle grid for planning and clearance: occupied cells, and unknown cells too if `unknown_is_obstacle`.
motion_planning::OccupancyGrid obstacle_grid(const GridMap& map, bool unknown_is_obstacle);

}  // namespace lidar_exploration
