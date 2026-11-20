// Test fixture: the office floor of lidar_exploration_gazebo/worlds/office_floor.sdf as a ground-truth grid, a
// simulated 360-degree lidar, and a mapper that writes what the lidar sees into a GridMap (perfect poses).
#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <vector>

#include "lidar_exploration/grid_map.hpp"
#include "motion_planning/grid.hpp"

namespace floor_plan {

using lidar_exploration::Cell;
using lidar_exploration::CellState;
using lidar_exploration::GridMap;

// Wall centre lines (ax, ay, bx, by), identical to scripts/make_office_floor.py; walls are 0.1 m thick.
inline const std::vector<std::array<double, 4>>& walls() {
    static const std::vector<std::array<double, 4>> w = {
        // outer walls, 20 m x 14 m
        {-10, -7, 10, -7}, {10, -7, 10, 7}, {10, 7, -10, 7}, {-10, 7, -10, -7},
        // corridor y in [-1, 1], doors 1.4 m wide (between centre lines)
        {-10, 1, -7.45, 1}, {-6.05, 1, -0.7, 1}, {0.7, 1, 6.05, 1}, {7.45, 1, 10, 1},
        {-10, -1, -6.2, -1}, {-4.8, -1, 0.8, -1}, {2.2, -1, 6.8, -1}, {8.2, -1, 10, -1},
        // room dividers; the two north-east rooms share a door
        {-3.5, 1, -3.5, 7}, {3.5, 1, 3.5, 4}, {3.5, 5.4, 3.5, 7}, {-3.5, -7, -3.5, -1}, {3.5, -7, 3.5, -1},
        // furniture
        {-8, 4.5, -6, 4.5}, {-1.5, 3.5, 0, 5}, {7, 3, 7, 5.5},
        {-8.5, -4, -7, -5.5}, {-1, -4.5, 1.5, -4.5}, {5.5, -3, 6.5, -3}, {8.5, -5, 8.5, -6.2}};
    return w;
}
constexpr double kHalfThickness = 0.05;
inline const Eigen::Vector2d kStart(-8.5, 0.0);

inline double distance_to_segment(const Eigen::Vector2d& p, const std::array<double, 4>& s) {
    const Eigen::Vector2d a(s[0], s[1]), b(s[2], s[3]);
    const double u = std::clamp((p - a).dot(b - a) / (b - a).squaredNorm(), 0.0, 1.0);
    return (p - a - u * (b - a)).norm();
}

// Ground truth at 5 cm: 0 free, 100 occupied, nothing unknown.
inline GridMap truth_map(double resolution = 0.05) {
    GridMap m(static_cast<int>(std::lround(21.0 / resolution)), static_cast<int>(std::lround(15.0 / resolution)),
              resolution, Eigen::Vector2d(-10.5, -7.5));
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < m.width; ++x) {
            const Eigen::Vector2d p = m.cell_center({x, y});
            bool occ = false;
            for (const auto& w : walls()) occ = occ || distance_to_segment(p, w) <= kHalfThickness + 0.5 * resolution;
            m.data[m.index({x, y})] = occ ? 100 : 0;
        }
    return m;
}

// Free cells reachable from the start by a 4-connected flood fill: the area a perfect exploration could map as free.
inline std::vector<std::uint8_t> reachable_free(const GridMap& truth) {
    std::vector<std::uint8_t> r(truth.data.size(), 0);
    const Cell s = truth.world_to_cell(kStart);
    std::deque<Cell> q{s};
    r[truth.index(s)] = 1;
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    while (!q.empty()) {
        const Cell c = q.front();
        q.pop_front();
        for (int k = 0; k < 4; ++k) {
            const Cell n{c.x + dx[k], c.y + dy[k]};
            if (truth.state(n) != CellState::Free || r[truth.index(n)]) continue;
            r[truth.index(n)] = 1;
            q.push_back(n);
        }
    }
    return r;
}

// Writes one 360-beam scan from `p` into `map` (same geometry as `truth`): free along each beam, occupied at the hit.
// Like slam_toolbox, a beam that hits nothing within `range` is dropped and clears nothing.
inline void integrate_scan(const GridMap& truth, GridMap& map, const Eigen::Vector2d& p, double range = 12.0,
                           int beams = 360) {
    const double step = 0.5 * truth.resolution;
    std::vector<Cell> ray;
    for (int k = 0; k < beams; ++k) {
        const double a = 2.0 * M_PI * k / beams;
        const Eigen::Vector2d dir(std::cos(a), std::sin(a));
        ray.clear();
        for (double r = 0.0; r <= range; r += step) {
            const Cell c = truth.world_to_cell(p + r * dir);
            if (!truth.in_bounds(c)) break;
            if (truth.state(c) == CellState::Occupied) {
                for (const Cell& f : ray)
                    if (map.data[map.index(f)] != 100) map.data[map.index(f)] = 0;
                map.data[map.index(c)] = 100;
                break;
            }
            ray.push_back(c);
        }
    }
}

// Fraction of the reachable free cells that the map knows to be free.
inline double coverage(const GridMap& map, const std::vector<std::uint8_t>& reachable) {
    std::size_t total = 0, known = 0;
    for (std::size_t i = 0; i < reachable.size(); ++i) {
        total += reachable[i];
        known += reachable[i] && map.data[i] >= 0 && map.data[i] < GridMap::kOccupiedThreshold;
    }
    return static_cast<double>(known) / static_cast<double>(total);
}

}  // namespace floor_plan
