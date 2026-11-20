// Frontiers, information gain, planner and path follower, each against a hand-checkable reference.
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

#include "floor_plan.hpp"
#include "lidar_exploration/frontier.hpp"
#include "lidar_exploration/path_follower.hpp"
#include "lidar_exploration/planner.hpp"
#include "motion_planning/dubins.hpp"

using namespace lidar_exploration;
namespace mp = motion_planning;

namespace {
// 10 m x 6 m at 0.1 m; cells with x < 5 m are free, the rest unknown.
GridMap half_known() {
    GridMap m(100, 60, 0.1, Eigen::Vector2d::Zero());
    for (int y = 0; y < m.height; ++y)
        for (int x = 0; x < 50; ++x) m.data[m.index({x, y})] = 0;
    return m;
}

// The robot drives exactly as commanded (unicycle, explicit Euler with a small step).
mp::Pose drive(mp::Pose q, const Command& c, double dt) {
    q.x() += c.v * std::cos(q.z()) * dt;
    q.y() += c.v * std::sin(q.z()) * dt;
    q.z() += c.omega * dt;
    return q;
}
}  // namespace

TEST(Frontier, BoundaryBetweenFreeAndUnknownIsOneFrontier) {
    const GridMap m = half_known();
    const auto f = find_frontiers(m, 5);
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].cells.size(), 60u);  // the column x = 49, full height
    EXPECT_NEAR(f[0].centroid.x(), 4.95, 1e-9);
    EXPECT_NEAR(f[0].centroid.y(), 3.0, 1e-9);
}

TEST(Frontier, PaddingTurnsTheMapEdgeIntoAFrontier) {
    GridMap m(30, 20, 0.1, Eigen::Vector2d(1.0, 2.0));
    for (auto& v : m.data) v = 0;  // all free, as a SLAM map cropped to what was seen
    EXPECT_TRUE(find_frontiers(m, 1).empty());
    const GridMap p = pad(m, 0.5);
    EXPECT_EQ(p.width, 40);
    EXPECT_EQ(p.height, 30);
    EXPECT_NEAR(p.origin.x(), 0.5, 1e-12);
    EXPECT_NEAR(p.origin.y(), 1.5, 1e-12);
    // Same world point, same value.
    EXPECT_EQ(p.state(p.world_to_cell({1.25, 2.35})), CellState::Free);
    EXPECT_EQ(p.state(p.world_to_cell({0.95, 2.35})), CellState::Unknown);
    const auto f = find_frontiers(p, 1);
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].cells.size(), 2u * (30 + 20) - 4);  // the ring of former edge cells
}

TEST(Frontier, SmallClustersAreDropped) {
    GridMap m(20, 20, 0.1, Eigen::Vector2d::Zero());
    for (auto& v : m.data) v = 0;
    m.data[m.index({10, 10})] = -1;  // one unknown cell: its 4 free neighbours are frontier cells
    EXPECT_EQ(find_frontiers(m, 1).size(), 1u);
    EXPECT_EQ(find_frontiers(m, 1)[0].cells.size(), 4u);
    EXPECT_TRUE(find_frontiers(m, 5).empty());
}

TEST(InformationGain, OpenUnknownSpaceIsTheSensorDisc) {
    GridMap m(200, 200, 0.05, Eigen::Vector2d(-5, -5));  // all unknown
    const double gain = visible_unknown_area(m, Eigen::Vector2d::Zero(), 3.0, 360);
    EXPECT_NEAR(gain, M_PI * 9.0, 0.03 * M_PI * 9.0);
}

TEST(InformationGain, WallsBlockTheView) {
    GridMap m(200, 200, 0.05, Eigen::Vector2d(-5, -5));
    for (int y = 0; y < m.height; ++y) m.data[m.index({110, y})] = 100;  // wall at x = 0.5 .. 0.55 m
    const double gain = visible_unknown_area(m, Eigen::Vector2d::Zero(), 3.0, 360);
    // Disc minus the circular segment beyond the wall: r^2 acos(d/r) - d sqrt(r^2 - d^2), d = 0.5 m
    const double r = 3.0, d = 0.5;
    const double segment = r * r * std::acos(d / r) - d * std::sqrt(r * r - d * d);
    EXPECT_NEAR(gain, M_PI * r * r - segment, 0.05 * (M_PI * r * r - segment));
}

TEST(Viewpoints, AreKnownFreeClearOfObstaclesAndNearTheFrontier) {
    const GridMap m = half_known();
    ViewpointOptions o;
    const auto f = find_frontiers(m, 5);
    const auto vps = select_viewpoints(m, f, Eigen::Vector2d(1.0, 3.0), o);
    ASSERT_FALSE(vps.empty());
    const auto clearance = mp::distance_transform(obstacle_grid(m, false));
    for (const auto& v : vps) {
        const Cell c = m.world_to_cell(v.pose.head<2>());
        EXPECT_EQ(m.state(c), CellState::Free);
        EXPECT_GE(clearance(c.y, c.x), o.clearance);
        EXPECT_LE(v.pose.x(), 5.0);
        EXPECT_GE(v.gain, o.min_gain);
    }
    for (size_t i = 1; i < vps.size(); ++i) EXPECT_GE(vps[i - 1].score, vps[i].score);
    // Nothing left to see: no viewpoints.
    GridMap all_free = m;
    for (auto& v : all_free.data) v = 0;
    EXPECT_TRUE(select_viewpoints(all_free, find_frontiers(all_free, 5), Eigen::Vector2d(1, 3), o).empty());
}

TEST(Planner, FindsAPathThroughADoorAndKeepsTheInflationDistance) {
    GridMap m(100, 60, 0.1, Eigen::Vector2d::Zero());
    for (auto& v : m.data) v = 0;
    for (int y = 0; y < 60; ++y)
        if (y < 25 || y > 38) m.data[m.index({50, y})] = 100;  // wall at x = 5 m with a 1.3 m door
    PlannerOptions o;
    const auto path = plan_path(m, mp::Pose(2, 1.5, 0), mp::Pose(8, 4.5, 0), o);
    ASSERT_TRUE(path.has_value());
    EXPECT_LT((path->row(0).head<2>().transpose() - Eigen::Vector2d(2, 1.5)).norm(), 1e-9);
    EXPECT_LT((path->row(path->rows() - 1).head<2>().transpose() - Eigen::Vector2d(8, 4.5)).norm(), 1e-6);
    const auto d = mp::distance_transform(obstacle_grid(m, false));
    for (int i = 0; i < path->rows(); ++i) {
        const Cell c = m.world_to_cell(path->row(i).head<2>().transpose());
        EXPECT_GT(d(c.y, c.x), o.inflation) << "sample " << i;
    }
    // Close the door: no path.
    for (int y = 25; y <= 38; ++y) m.data[m.index({50, y})] = 100;
    EXPECT_FALSE(plan_path(m, mp::Pose(2, 1.5, 0), mp::Pose(8, 4.5, 0), o).has_value());
}

TEST(Planner, UnknownSpaceIsTraversableOnlyIfAllowed) {
    GridMap m = half_known();
    // A free pocket at x in [7, 8] m inside the unknown half.
    for (int y = 25; y < 35; ++y)
        for (int x = 70; x < 80; ++x) m.data[m.index({x, y})] = 0;
    PlannerOptions o;
    EXPECT_TRUE(plan_path(m, mp::Pose(2.0, 3.0, 0), mp::Pose(7.5, 3.0, 0), o).has_value());
    o.plan_through_unknown = false;
    EXPECT_FALSE(plan_path(m, mp::Pose(2.0, 3.0, 0), mp::Pose(7.5, 3.0, 0), o).has_value());
}

TEST(Planner, StartInsideTheInflationIsStillPlannable) {
    GridMap m(100, 60, 0.1, Eigen::Vector2d::Zero());
    for (auto& v : m.data) v = 0;
    for (int x = 0; x < 100; ++x) m.data[m.index({x, 30})] = 100;  // wall at y = 3.0 .. 3.1 m
    PlannerOptions o;
    // 0.25 m below the wall: inside the 0.45 m inflation, but the start disc is cleared.
    EXPECT_TRUE(plan_path(m, mp::Pose(5.0, 2.75, M_PI), mp::Pose(1.0, 1.0, M_PI), o).has_value());
}

TEST(PathFollower, TracksADubinsPathToItsEnd) {
    const mp::DubinsPath dp = mp::shortest_dubins_path(mp::Pose(0, 0, 0), mp::Pose(3, 2, M_PI), 0.3);
    const mp::Trajectory t = dp.sample(0.05);
    FollowerOptions o;
    PathFollower f(mp::Points2D(t.leftCols<2>()), o);
    mp::Pose q(0, 0, 0);
    mp::Path2D ref(mp::Points2D(t.leftCols<2>()));
    double max_cross_track = 0.0;
    int steps = 0;
    for (; steps < 2000 && !f.done(); ++steps) {
        q = drive(q, f.step(q), 0.02);
        max_cross_track = std::max(max_cross_track, std::abs(ref.project(q.head<2>()).lateral));
    }
    EXPECT_TRUE(f.done());
    EXPECT_LT((q.head<2>() - Eigen::Vector2d(3, 2)).norm(), o.goal_tolerance + 1e-9);
    EXPECT_LT(max_cross_track, 0.12);
    // At 0.35 m/s it should not take much longer than length / speed.
    EXPECT_LT(steps * 0.02, 1.5 * dp.length() / o.speed);
}

TEST(PathFollower, TurnsInPlaceWhenThePathStartsBehind) {
    mp::Points2D pts(2, 2);
    pts << 0, 0, -2, 0;
    PathFollower f(pts, FollowerOptions{});
    const Command c = f.step(mp::Pose(0, 0, 0));
    EXPECT_DOUBLE_EQ(c.v, 0.0);
    EXPECT_NE(c.omega, 0.0);
}

// The tests' floor plan must be the Gazebo world's: compare with the wall list make_office_floor.py writes.
TEST(FloorPlan, MatchesTheGazeboWorld) {
    std::ifstream in(WALLS_JSON);
    ASSERT_TRUE(in) << "cannot read " << WALLS_JSON;
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string json = ss.str();
    const std::string list = json.substr(json.find("\"walls\""));
    const std::regex number(R"(-?[0-9]+(\.[0-9]+)?)");
    std::vector<double> values;
    for (auto it = std::sregex_iterator(list.begin(), list.end(), number); it != std::sregex_iterator(); ++it)
        values.push_back(std::stod(it->str()));
    const auto& walls = floor_plan::walls();
    ASSERT_EQ(values.size(), 4 * walls.size());
    for (size_t i = 0; i < walls.size(); ++i)
        for (int k = 0; k < 4; ++k) EXPECT_DOUBLE_EQ(values[4 * i + k], walls[i][k]) << "wall " << i;
}
