// The whole exploration loop on the office floor with an ideal robot, a simulated lidar and a perfect mapper: the
// explorer must map nearly all of the reachable free space, stop by itself, and never come close to a wall.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "floor_plan.hpp"
#include "lidar_exploration/explorer.hpp"

using namespace lidar_exploration;
namespace mp = motion_planning;

namespace {

struct RunResult {
    bool finished = false;
    double time = 0.0, distance = 0.0, coverage = 0.0, min_clearance = INFINITY;
    mp::Pose closest = mp::Pose::Zero();  // where the robot came closest to a wall
    int goals = 0, replans = 0;
};

// Optional CSV of (t, x, y, coverage) for plotting, written when `csv` is non-empty.
RunResult explore(std::uint32_t seed, const std::string& csv = "") {
    const GridMap truth = floor_plan::truth_map();
    const auto reachable = floor_plan::reachable_free(truth);
    const auto wall_distance = mp::distance_transform(obstacle_grid(truth, false));
    GridMap map(truth.width, truth.height, truth.resolution, truth.origin);  // all unknown

    ExplorerOptions o;
    o.viewpoints.seed = seed;
    o.planner.seed = seed;
    Explorer ex(o);
    mp::Pose q(floor_plan::kStart.x(), floor_plan::kStart.y(), 0.0);
    const double dt = 0.05, scan_period = 0.1, map_period = 1.0;  // lidar at 10 Hz, map published at 1 Hz
    std::ofstream out;
    if (!csv.empty()) {
        out.open(csv);
        out << "t,x,y,coverage\n";
    }
    const bool verbose = std::getenv("EXPLORATION_VERBOSE") != nullptr;
    std::string last_event;
    RunResult r;
    double t = 0.0;
    for (int k = 0; t < 1800.0; ++k, t = k * dt) {
        if (k % static_cast<int>(std::lround(scan_period / dt)) == 0)
            floor_plan::integrate_scan(truth, map, q.head<2>());
        if (k % static_cast<int>(std::lround(map_period / dt)) == 0) {
            ex.set_map(map);
            if (out) out << t << "," << q.x() << "," << q.y() << "," << floor_plan::coverage(map, reachable) << "\n";
        }
        const Command c = ex.update(q, t);
        if (verbose && ex.last_event() != last_event) {
            last_event = ex.last_event();
            std::printf("  t=%6.1f (%.2f, %.2f) cov %.3f  %s\n", t, q.x(), q.y(),
                        floor_plan::coverage(map, reachable), last_event.c_str());
        }
        if (ex.state() == ExplorerState::Finished) {
            r.finished = true;
            break;
        }
        q.x() += c.v * std::cos(q.z()) * dt;
        q.y() += c.v * std::sin(q.z()) * dt;
        q.z() += c.omega * dt;
        r.distance += std::abs(c.v) * dt;
        const Cell cell = truth.world_to_cell(q.head<2>());
        if (wall_distance(cell.y, cell.x) < r.min_clearance) {
            r.min_clearance = wall_distance(cell.y, cell.x);
            r.closest = q;
        }
    }
    r.time = t;
    r.coverage = floor_plan::coverage(map, reachable);
    r.goals = ex.goals_reached();
    r.replans = ex.replans();
    return r;
}

}  // namespace

class Exploration : public ::testing::TestWithParam<std::uint32_t> {};

TEST_P(Exploration, MapsTheOfficeFloorAndStopsByItself) {
    const RunResult r = explore(GetParam());
    std::printf("seed %u: finished %d after %.0f s, %.1f m, %d goals, %d replans, coverage %.3f, "
                "closest wall %.2f m at (%.2f, %.2f)\n",
                GetParam(), r.finished, r.time, r.distance, r.goals, r.replans, r.coverage, r.min_clearance,
                r.closest.x(), r.closest.y());
    EXPECT_TRUE(r.finished);
    EXPECT_GT(r.coverage, 0.97);
    // The robot's body reaches 0.36 m from its centre; the distance is to the centre of the nearest wall cell.
    EXPECT_GT(r.min_clearance, 0.36 + 0.05);
}

INSTANTIATE_TEST_SUITE_P(Seeds, Exploration, ::testing::Values(1u, 2u, 3u));

// Writes the trajectory of seed 1 for the README figure when EXPLORATION_CSV is set (not a test of its own).
TEST(ExplorationLog, WritesCsvIfRequested) {
    if (const char* path = std::getenv("EXPLORATION_CSV")) {
        explore(1, path);
        SUCCEED() << "wrote " << path;
    } else {
        GTEST_SKIP() << "set EXPLORATION_CSV=<file> to write the run";
    }
}
