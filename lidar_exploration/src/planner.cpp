#include "lidar_exploration/planner.hpp"

#include <cmath>
#include <stdexcept>

#include "motion_planning/dubins.hpp"
#include "motion_planning/sampling.hpp"

namespace lidar_exploration {

namespace mp = motion_planning;

mp::OccupancyGrid planning_grid(const GridMap& map, const Eigen::Vector2d& start, const PlannerOptions& opt) {
    mp::OccupancyGrid grid = mp::inflate(obstacle_grid(map, !opt.plan_through_unknown), opt.inflation);
    const int r = static_cast<int>(std::ceil(opt.inflation / map.resolution));
    const Cell c0 = map.world_to_cell(start);
    for (int dy = -r; dy <= r; ++dy)
        for (int dx = -r; dx <= r; ++dx) {
            const Cell c{c0.x + dx, c0.y + dy};
            if (map.in_bounds(c) && (map.cell_center(c) - start).norm() <= opt.inflation &&
                map.state(c) != CellState::Occupied)
                grid.set(c, false);
        }
    return grid;
}

std::optional<mp::Trajectory> plan_path(const GridMap& map, const mp::Pose& start, const mp::Pose& goal,
                                        const PlannerOptions& opt) {
    const mp::OccupancyGrid grid = planning_grid(map, start.head<2>(), opt);
    if (!grid.point_free(start.head<2>()) || !grid.point_free(goal.head<2>())) return std::nullopt;
    mp::PlanningProblem2D problem = mp::problem_from_grid(grid, start.head<2>(), goal.head<2>());
    const Eigen::Vector2d map_lower = problem.lower, map_upper = problem.upper;

    std::vector<mp::DubinsPath> segments;
    const mp::DubinsPath direct = mp::shortest_dubins_path(start, goal, opt.turning_radius);
    if (mp::dubins_path_valid(direct, problem, 0.5 * map.resolution)) {
        segments.push_back(direct);  // nothing in the way: no need to grow a tree
    } else {
        mp::DubinsPlannerOptions o;
        o.max_iterations = opt.max_iterations;
        o.radius = opt.turning_radius;
        o.step = opt.step;
        o.goal_bias = opt.goal_bias;
        o.collision_step = 0.5 * map.resolution;
        o.seed = opt.seed;
        // RRT* samples uniformly in the box, so a box much larger than the detour needed wastes the iterations and
        // returns long, winding paths. Grow it from the start-goal bounding box until a path is found.
        const Eigen::Vector2d lo = start.head<2>().cwiseMin(goal.head<2>());
        const Eigen::Vector2d hi = start.head<2>().cwiseMax(goal.head<2>());
        for (double margin : opt.box_margins) {
            problem.lower = (lo.array() - margin).max(map_lower.array()).matrix();
            problem.upper = (hi.array() + margin).min(map_upper.array()).matrix();
            mp::DubinsTreeResult tree;
            try {
                tree = mp::dubins_rrt_star(problem, start, goal, o);
            } catch (const std::invalid_argument&) {
                return std::nullopt;
            }
            if (tree.found) {
                segments = tree.segments;
                break;
            }
        }
        if (segments.empty()) return std::nullopt;
    }

    std::vector<mp::Pose> poses;
    for (const mp::DubinsPath& seg : segments) {
        const mp::Trajectory t = seg.sample(opt.sample_step);
        for (int i = poses.empty() ? 0 : 1; i < t.rows(); ++i) poses.push_back(t.row(i).transpose());
    }
    mp::Trajectory out(static_cast<int>(poses.size()), 3);
    for (int i = 0; i < out.rows(); ++i) out.row(i) = poses[i].transpose();
    return out;
}

bool path_free(const mp::OccupancyGrid& grid, const mp::Trajectory& path, int from) {
    for (int i = std::max(0, from); i < path.rows(); ++i)
        if (!grid.point_free(path.row(i).head<2>().transpose())) return false;
    return true;
}

}  // namespace lidar_exploration
