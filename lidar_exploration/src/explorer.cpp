#include "lidar_exploration/explorer.hpp"

#include <algorithm>
#include <cstdio>

namespace lidar_exploration {

namespace mp = motion_planning;

std::string to_string(ExplorerState s) {
    switch (s) {
        case ExplorerState::WaitingForMap: return "waiting for map";
        case ExplorerState::Selecting: return "selecting";
        case ExplorerState::Following: return "following";
        case ExplorerState::Finished: return "finished";
    }
    return "?";
}

Explorer::Explorer(const ExplorerOptions& options) : opt_(options) {
    // A viewpoint on the boundary of the inflated obstacles would be a valid goal that the planner rejects.
    opt_.viewpoints.clearance = std::max(opt_.viewpoints.clearance, opt_.planner.inflation + 0.05);
}

void Explorer::set_map(GridMap map) {
    map_ = std::move(map);
    have_map_ = map_changed_ = true;
}

Command Explorer::update(const mp::Pose& q, double t) {
    last_t_ = t;
    if (state_ == ExplorerState::Finished || !have_map_) return {};
    if (state_ == ExplorerState::WaitingForMap) state_ = ExplorerState::Selecting;

    if (map_changed_) {
        map_changed_ = false;
        fresh_map_ = true;
        frontiers_ = find_frontiers(map_, opt_.min_frontier_cells);
        if (state_ == ExplorerState::Following) {
            const mp::OccupancyGrid grid = planning_grid(map_, q.head<2>(), opt_.planner);
            if (!path_free(grid, path_, follower_->next_index())) {
                ++replans_;
                abandon("path blocked by the new map");
            } else if (visible_unknown_area(map_, goal_->pose.head<2>(), opt_.viewpoints.sensor_range,
                                            opt_.viewpoints.rays) < opt_.viewpoints.min_gain) {
                abandon("goal area already explored");
            }
        }
    }

    if (state_ == ExplorerState::Following && t > goal_deadline_) {
        block(goal_->pose.head<2>(), t);
        abandon("goal timed out");
    }
    if (state_ == ExplorerState::Selecting && fresh_map_ && t >= next_selection_) select(q, t);
    if (state_ != ExplorerState::Following) return {};

    const Command cmd = follower_->step(q);
    if (follower_->done()) {
        ++goals_reached_;
        abandon("goal reached");
        fresh_map_ = false;  // choose the next goal on a map that includes what was seen from this one
    }
    return cmd;
}

void Explorer::select(const mp::Pose& q, double t) {
    const std::vector<Viewpoint> candidates = select_viewpoints(map_, frontiers_, q.head<2>(), opt_.viewpoints,
                                                                blocked_positions(t), opt_.blocked_radius);
    int tried = 0;
    for (const Viewpoint& v : candidates) {
        if (tried++ == opt_.plans_per_selection) break;
        PlannerOptions po = opt_.planner;
        po.seed = plan_seed_++;
        std::optional<mp::Trajectory> path = plan_path(map_, q, v.pose, po);
        if (!path || path->rows() < 2) {
            block(v.pose.head<2>(), t);
            continue;
        }
        goal_ = v;
        path_ = std::move(*path);
        follower_ = std::make_unique<PathFollower>(mp::Points2D(path_.leftCols<2>()), opt_.follower);
        goal_deadline_ = t + opt_.timeout_factor * follower_->length() / opt_.follower.speed + 10.0;
        state_ = ExplorerState::Following;
        failed_selections_ = 0;
        char buf[160];
        std::snprintf(buf, sizeof(buf), "new goal (%.2f, %.2f): gain %.1f m^2, path %.2f m", v.pose.x(),
                      v.pose.y(), v.gain, follower_->length());
        event_ = buf;
        return;
    }
    next_selection_ = t + opt_.reselect_period;
    if (!candidates.empty()) {
        event_ = "no path to the best viewpoints; they are blocked";
        return;
    }
    if (waiting_for_unblock(t)) {  // what is left was only blocked for a while: try it again later
        event_ = "only temporarily blocked viewpoints left; waiting";
        return;
    }
    fresh_map_ = false;  // look again only after the next map update
    if (++failed_selections_ >= opt_.failed_selections_to_finish) {
        state_ = ExplorerState::Finished;
        event_ = "no informative viewpoint left: exploration finished";
    } else {
        event_ = "no informative viewpoint";
    }
}

void Explorer::block(const Eigen::Vector2d& p, double t) {
    for (Blocked& b : blocked_)
        if ((b.p - p).norm() < opt_.blocked_radius) {
            ++b.failures;
            b.until = t + opt_.blocked_time;
            return;
        }
    blocked_.push_back({p, 1, t + opt_.blocked_time});
}

std::vector<Eigen::Vector2d> Explorer::blocked_positions(double t) const {
    std::vector<Eigen::Vector2d> out;
    for (const Blocked& b : blocked_)
        if (b.failures >= opt_.failures_to_give_up || t < b.until) out.push_back(b.p);
    return out;
}

bool Explorer::waiting_for_unblock(double t) const {
    for (const Blocked& b : blocked_)
        if (b.failures < opt_.failures_to_give_up && t < b.until) return true;
    return false;
}

void Explorer::abandon(const std::string& why) {
    goal_.reset();
    follower_.reset();
    path_ = mp::Trajectory();
    state_ = ExplorerState::Selecting;
    next_selection_ = 0.0;
    event_ = why;
}

}  // namespace lidar_exploration
