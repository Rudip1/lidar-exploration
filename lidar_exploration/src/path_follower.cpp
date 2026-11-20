#include "lidar_exploration/path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace lidar_exploration {

namespace mp = motion_planning;

namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
}  // namespace

PathFollower::PathFollower(const mp::Points2D& path, const FollowerOptions& options)
    : path_(path), opt_(options) {}

Command PathFollower::step(const mp::Pose& q) {
    if (done_) return {};
    const Eigen::Vector2d p = q.head<2>();
    const double length = path_.length();
    // The reached arc length only moves forward, and by at most one look-ahead per step, so that a path passing
    // close to itself cannot make the robot skip a loop.
    const double s = path_.project(p).s;
    if (s > s_ && s < s_ + 2.0 * opt_.lookahead) s_ = s;

    if ((path_.point_at(length) - p).norm() < opt_.goal_tolerance && s_ >= length - opt_.lookahead) {
        done_ = true;
        return {};
    }
    const Eigen::Vector2d target = path_.point_at(std::min(s_ + opt_.lookahead, length));
    const Eigen::Vector2d d = target - p;
    const double alpha = wrap(std::atan2(d.y(), d.x()) - q[2]);
    Command cmd;
    if (std::abs(alpha) > opt_.turn_in_place_angle) {
        cmd.omega = std::clamp(2.0 * alpha, -opt_.max_angular_speed, opt_.max_angular_speed);
        return cmd;
    }
    const double kappa = mp::pure_pursuit_curvature(q, target);
    cmd.v = opt_.speed;
    cmd.omega = cmd.v * kappa;
    if (std::abs(cmd.omega) > opt_.max_angular_speed) {  // too tight at full speed: same arc, slower
        cmd.omega = std::copysign(opt_.max_angular_speed, cmd.omega);
        cmd.v = opt_.max_angular_speed / std::abs(kappa);
    }
    return cmd;
}

int PathFollower::next_index() const {
    const mp::Points2D& pts = path_.points();
    double s = 0.0;
    for (int i = 1; i < pts.rows(); ++i) {
        s += (pts.row(i) - pts.row(i - 1)).norm();
        if (s >= s_) return i;
    }
    return static_cast<int>(pts.rows()) - 1;
}

}  // namespace lidar_exploration
