// Pure pursuit for a differential-drive robot (learn-motion-planning chapter 7): steer on the arc through a point
// one look-ahead ahead on the path; turn in place first when that point is far off the heading.
#pragma once

#include <Eigen/Core>

#include "motion_planning/kinematics.hpp"
#include "motion_planning/tracking.hpp"

namespace lidar_exploration {

struct FollowerOptions {
    double speed = 0.35;              // [m/s]
    double lookahead = 0.30;          // [m]
    double max_angular_speed = 1.0;   // [rad/s]
    double turn_in_place_angle = 1.0; // heading error to the look-ahead point above which the robot turns in place
    double goal_tolerance = 0.15;     // [m]
};

struct Command {
    double v = 0.0, omega = 0.0;
};

class PathFollower {
public:
    PathFollower(const motion_planning::Points2D& path, const FollowerOptions& options);

    Command step(const motion_planning::Pose& q);
    bool done() const { return done_; }
    double progress() const { return s_; }  // arc length reached [m]
    double length() const { return path_.length(); }
    // Index of the first path sample at or after the reached arc length (for re-checking the rest of the path).
    int next_index() const;

private:
    motion_planning::Path2D path_;
    FollowerOptions opt_;
    double s_ = 0.0;
    bool done_ = false;
};

}  // namespace lidar_exploration
