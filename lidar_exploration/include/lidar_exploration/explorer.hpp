// The exploration loop, free of ROS: choose a viewpoint at a frontier, plan a Dubins path to it, follow it, and
// choose again when it is reached, when a map update blocks the path or makes the viewpoint uninformative, or after
// a timeout. Exploration is finished when no frontier offers a reachable viewpoint any more.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "lidar_exploration/frontier.hpp"
#include "lidar_exploration/grid_map.hpp"
#include "lidar_exploration/path_follower.hpp"
#include "lidar_exploration/planner.hpp"

namespace lidar_exploration {

struct ExplorerOptions {
    ViewpointOptions viewpoints;
    PlannerOptions planner;
    FollowerOptions follower;
    int min_frontier_cells = 5;
    int plans_per_selection = 5;      // best viewpoints tried before the selection counts as failed
    int failed_selections_to_finish = 3;
    double reselect_period = 1.0;     // wait this long after a failed selection [s]
    double timeout_factor = 4.0;      // abandon a goal after timeout_factor * path length / speed + 10 s
    double blocked_radius = 0.6;      // goals that failed are avoided within this radius [m] ...
    double blocked_time = 30.0;       // ... for this long [s] (the map may have opened a way meanwhile) ...
    int failures_to_give_up = 3;      // ... and for good after this many failures there
};

enum class ExplorerState { WaitingForMap, Selecting, Following, Finished };
std::string to_string(ExplorerState s);

class Explorer {
public:
    explicit Explorer(const ExplorerOptions& options = {});

    // New map from the mapper. The path is re-checked against it on the next update().
    void set_map(GridMap map);
    // One control step at time t [s] with the robot at pose q (map frame).
    Command update(const motion_planning::Pose& q, double t);

    ExplorerState state() const { return state_; }
    const GridMap& map() const { return map_; }
    const std::vector<Frontier>& frontiers() const { return frontiers_; }
    const std::optional<Viewpoint>& goal() const { return goal_; }
    const motion_planning::Trajectory& path() const { return path_; }
    std::vector<Eigen::Vector2d> blocked_goals() const { return blocked_positions(last_t_); }
    int goals_reached() const { return goals_reached_; }
    int replans() const { return replans_; }
    const std::string& last_event() const { return event_; }  // why the state last changed, for logging

private:
    struct Blocked {
        Eigen::Vector2d p;
        int failures = 0;
        double until = 0.0;
    };
    void select(const motion_planning::Pose& q, double t);
    void abandon(const std::string& why);
    void block(const Eigen::Vector2d& p, double t);
    std::vector<Eigen::Vector2d> blocked_positions(double t) const;
    bool waiting_for_unblock(double t) const;

    ExplorerOptions opt_;
    GridMap map_;
    bool have_map_ = false, map_changed_ = false;
    bool fresh_map_ = false;  // a map update arrived since the last goal was reached or the last selection failed
    ExplorerState state_ = ExplorerState::WaitingForMap;
    std::vector<Frontier> frontiers_;
    std::optional<Viewpoint> goal_;
    motion_planning::Trajectory path_;
    std::unique_ptr<PathFollower> follower_;
    std::vector<Blocked> blocked_;
    double goal_deadline_ = 0.0, next_selection_ = 0.0, last_t_ = 0.0;
    int failed_selections_ = 0, goals_reached_ = 0, replans_ = 0;
    std::uint32_t plan_seed_ = 1;
    std::string event_;
};

}  // namespace lidar_exploration
