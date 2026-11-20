// Curvature-limited paths on the current map with learn-motion-planning's Dubins RRT*: the obstacles are the occupied
// cells (and optionally the unknown ones), inflated by the robot radius plus a margin.
//
// Planning through unknown space is the default: a SLAM map is full of unknown speckles and of unmapped strips where
// long beams found no return, and treating them as walls blocks almost every path. The explorer re-checks the path
// against every new map, so a wall that appears in unknown space ahead causes a replan before the robot gets there.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "lidar_exploration/grid_map.hpp"
#include "motion_planning/kinematics.hpp"

namespace lidar_exploration {

struct PlannerOptions {
    double inflation = 0.45;       // robot radius (0.36 m to the body corners) + margin [m]
    bool plan_through_unknown = true;
    double turning_radius = 0.30;  // [m]; the robot could turn in place, but smooth paths are faster to follow
    int max_iterations = 1500;
    double step = 1.5;             // longest extension [m]
    double goal_bias = 0.2;
    double sample_step = 0.05;     // spacing of the returned path [m]
    // RRT* sampling boxes tried in turn: the start-goal bounding box grown by each margin [m] (clipped to the map).
    std::vector<double> box_margins = {2.0, 6.0, 1e9};
    std::uint32_t seed = 1;
};

// The planning grid: inflated obstacles, with the disc of radius `inflation` around `start` cleared of everything
// except truly occupied cells, so that a robot that has drifted close to an obstacle can still leave.
motion_planning::OccupancyGrid planning_grid(const GridMap& map, const Eigen::Vector2d& start,
                                             const PlannerOptions& options);

// Poses every sample_step from start to goal, or nothing if no path was found within max_iterations.
std::optional<motion_planning::Trajectory> plan_path(const GridMap& map, const motion_planning::Pose& start,
                                                     const motion_planning::Pose& goal,
                                                     const PlannerOptions& options);

// True if the path from index `from` on is still free on the (inflated) current map.
bool path_free(const motion_planning::OccupancyGrid& grid, const motion_planning::Trajectory& path, int from);

}  // namespace lidar_exploration
