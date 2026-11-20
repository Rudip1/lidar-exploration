#include "lidar_exploration/frontier.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>

namespace lidar_exploration {

namespace {
constexpr int kDx4[4] = {1, -1, 0, 0}, kDy4[4] = {0, 0, 1, -1};

bool is_frontier(const GridMap& map, const Cell& c) {
    if (map.state(c) != CellState::Free) return false;
    for (int k = 0; k < 4; ++k) {
        const Cell n{c.x + kDx4[k], c.y + kDy4[k]};
        if (map.in_bounds(n) && map.state(n) == CellState::Unknown) return true;  // the map edge is no frontier
    }
    return false;
}
}  // namespace

std::vector<Frontier> find_frontiers(const GridMap& map, int min_cells) {
    std::vector<std::uint8_t> seen(map.data.size(), 0);
    std::vector<Frontier> out;
    for (int y = 0; y < map.height; ++y)
        for (int x = 0; x < map.width; ++x) {
            const Cell c0{x, y};
            if (seen[map.index(c0)] || !is_frontier(map, c0)) continue;
            Frontier f;
            std::deque<Cell> queue{c0};
            seen[map.index(c0)] = 1;
            while (!queue.empty()) {  // 8-connected flood fill over frontier cells
                const Cell c = queue.front();
                queue.pop_front();
                f.cells.push_back(c);
                for (int dy = -1; dy <= 1; ++dy)
                    for (int dx = -1; dx <= 1; ++dx) {
                        const Cell n{c.x + dx, c.y + dy};
                        if (!map.in_bounds(n) || seen[map.index(n)] || !is_frontier(map, n)) continue;
                        seen[map.index(n)] = 1;
                        queue.push_back(n);
                    }
            }
            if (static_cast<int>(f.cells.size()) < min_cells) continue;
            for (const Cell& c : f.cells) f.centroid += map.cell_center(c);
            f.centroid /= static_cast<double>(f.cells.size());
            out.push_back(std::move(f));
        }
    return out;
}

double visible_unknown_area(const GridMap& map, const Eigen::Vector2d& p, double range, int rays) {
    std::vector<std::size_t> cells;  // unknown cells crossed by any ray
    const double step = 0.5 * map.resolution;
    for (int k = 0; k < rays; ++k) {
        const double a = 2.0 * M_PI * k / rays;
        const Eigen::Vector2d dir(std::cos(a), std::sin(a));
        for (double r = 0.0; r <= range; r += step) {
            const Cell c = map.world_to_cell(p + r * dir);
            if (!map.in_bounds(c)) break;  // nothing is known to exist beyond the map
            const CellState s = map.state(c);
            if (s == CellState::Occupied) break;
            if (s == CellState::Unknown) cells.push_back(map.index(c));
        }
    }
    std::sort(cells.begin(), cells.end());
    const auto n = std::unique(cells.begin(), cells.end()) - cells.begin();
    return static_cast<double>(n) * map.resolution * map.resolution;
}

std::vector<Viewpoint> select_viewpoints(const GridMap& map, const std::vector<Frontier>& frontiers,
                                         const Eigen::Vector2d& robot, const ViewpointOptions& opt,
                                         const std::vector<Eigen::Vector2d>& blocked, double blocked_radius) {
    // Viewpoints are known free cells away from obstacles.
    const motion_planning::FieldArray clearance = motion_planning::distance_transform(obstacle_grid(map, false));

    std::vector<Cell> pool;
    for (const Frontier& f : frontiers) pool.insert(pool.end(), f.cells.begin(), f.cells.end());
    std::mt19937 rng(opt.seed);
    std::shuffle(pool.begin(), pool.end(), rng);
    if (static_cast<int>(pool.size()) > opt.max_candidates) pool.resize(opt.max_candidates);

    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<Viewpoint> out;
    for (const Cell& fc : pool) {
        const Eigen::Vector2d target = map.cell_center(fc);
        for (int t = 0; t < opt.tries_per_candidate; ++t) {
            const double r = opt.search_radius * std::sqrt(unit(rng)), a = 2.0 * M_PI * unit(rng);
            const Eigen::Vector2d p = target + r * Eigen::Vector2d(std::cos(a), std::sin(a));
            const Cell c = map.world_to_cell(p);
            if (!map.in_bounds(c) || map.state(c) != CellState::Free || clearance(c.y, c.x) < opt.clearance) continue;
            const bool is_blocked = std::any_of(blocked.begin(), blocked.end(), [&](const Eigen::Vector2d& b) {
                return (b - p).norm() < blocked_radius;
            });
            if (is_blocked) continue;
            Viewpoint v;
            v.gain = visible_unknown_area(map, p, opt.sensor_range, opt.rays);
            if (v.gain < opt.min_gain) break;  // this frontier cell is nearly explored from here; next one
            const Eigen::Vector2d d = target - p;
            v.pose = motion_planning::Pose(p.x(), p.y(), std::atan2(d.y(), d.x()));
            v.distance = (p - robot).norm();
            v.score = v.gain * std::exp(-opt.distance_decay * v.distance);
            out.push_back(v);
            break;
        }
    }
    std::sort(out.begin(), out.end(), [](const Viewpoint& a, const Viewpoint& b) { return a.score > b.score; });
    return out;
}

}  // namespace lidar_exploration
