// Frontiers (Yamauchi 1997): known free cells next to unknown space, grouped into 8-connected clusters, and
// viewpoint candidates scored by the unknown area a 360-degree lidar would see from them.
#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "lidar_exploration/grid_map.hpp"
#include "motion_planning/kinematics.hpp"

namespace lidar_exploration {

struct Frontier {
    std::vector<Cell> cells;
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
};

// A free cell is a frontier cell if one of its 4-neighbours inside the map is unknown. Clusters smaller than
// `min_cells` are dropped (single cells along a wall are mostly noise of the scan matcher).
std::vector<Frontier> find_frontiers(const GridMap& map, int min_cells);

// Unknown area [m^2] inside the map visible from p: `rays` rays out to `range`, each stopping at the first occupied
// cell or the map edge.
// Unknown cells do not block a ray, so this is an optimistic estimate, as is usual for frontier exploration.
double visible_unknown_area(const GridMap& map, const Eigen::Vector2d& p, double range, int rays);

struct ViewpointOptions {
    double clearance = 0.50;         // from occupied cells [m]; more than the planner's inflation
    double sensor_range = 12.0;      // [m], as the simulated lidar
    int rays = 120;
    double min_gain = 1.0;           // viewpoints that would reveal less unknown area [m^2] are not worth a trip
    double distance_decay = 0.15;    // score = gain * exp(-decay * distance) [1/m]
    double search_radius = 1.5;      // viewpoints are sought within this distance of a frontier cell [m]
    int max_candidates = 150;        // frontier cells examined per selection
    int tries_per_candidate = 12;
    std::uint32_t seed = 1;
};

struct Viewpoint {
    motion_planning::Pose pose = motion_planning::Pose::Zero();  // heading towards the frontier cell it serves
    double gain = 0.0;      // visible unknown area [m^2]
    double distance = 0.0;  // straight-line distance from the robot [m]
    double score = 0.0;
};

// Viewpoints with gain >= min_gain, best score first. `blocked` positions (e.g. goals that could not be reached)
// exclude candidates within `blocked_radius`.
std::vector<Viewpoint> select_viewpoints(const GridMap& map, const std::vector<Frontier>& frontiers,
                                         const Eigen::Vector2d& robot, const ViewpointOptions& options,
                                         const std::vector<Eigen::Vector2d>& blocked = {},
                                         double blocked_radius = 0.5);

}  // namespace lidar_exploration
