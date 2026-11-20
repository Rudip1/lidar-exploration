// ROS 2 wrapper around lidar_exploration::Explorer: the map from the SLAM node in, the robot pose from TF, velocity
// commands out. Publishes the frontiers, the goal and the path for RViz, and `exploration/finished` when done.
//
// Subscribes: map (nav_msgs/OccupancyGrid)          TF: map -> base_footprint
// Publishes:  cmd_vel (geometry_msgs/Twist), exploration/path (nav_msgs/Path),
//             exploration/markers (visualization_msgs/MarkerArray), exploration/finished (std_msgs/Empty, latched)
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lidar_exploration/explorer.hpp"

namespace le = lidar_exploration;
namespace mp = motion_planning;
using namespace std::chrono_literals;

class ExplorerNode : public rclcpp::Node {
public:
    ExplorerNode() : rclcpp::Node("explorer") {
        le::ExplorerOptions o;
        auto& v = o.viewpoints;
        v.clearance = declare_parameter("viewpoint_clearance", v.clearance);
        v.sensor_range = declare_parameter("sensor_range", v.sensor_range);
        v.min_gain = declare_parameter("min_gain", v.min_gain);
        v.distance_decay = declare_parameter("distance_decay", v.distance_decay);
        v.search_radius = declare_parameter("viewpoint_search_radius", v.search_radius);
        v.max_candidates = static_cast<int>(declare_parameter("max_candidates", v.max_candidates));
        auto& p = o.planner;
        p.inflation = declare_parameter("inflation", p.inflation);
        p.turning_radius = declare_parameter("turning_radius", p.turning_radius);
        p.max_iterations = static_cast<int>(declare_parameter("rrt_iterations", p.max_iterations));
        p.plan_through_unknown = declare_parameter("plan_through_unknown", p.plan_through_unknown);
        auto& f = o.follower;
        f.speed = declare_parameter("speed", f.speed);
        f.lookahead = declare_parameter("lookahead", f.lookahead);
        f.max_angular_speed = declare_parameter("max_angular_speed", f.max_angular_speed);
        f.goal_tolerance = declare_parameter("goal_tolerance", f.goal_tolerance);
        o.min_frontier_cells = static_cast<int>(declare_parameter("min_frontier_cells", o.min_frontier_cells));
        o.failed_selections_to_finish =
            static_cast<int>(declare_parameter("failed_selections_to_finish", o.failed_selections_to_finish));
        map_frame_ = declare_parameter("map_frame", map_frame_);
        base_frame_ = declare_parameter("base_frame", base_frame_);
        const double rate = declare_parameter("control_rate", 20.0);
        // A band of unknown cells around the SLAM map, so that free space at its edge forms a frontier.
        pad_margin_ = declare_parameter("map_padding", pad_margin_);
        explorer_ = std::make_unique<le::Explorer>(o);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("exploration/path", rclcpp::QoS(1).transient_local());
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("exploration/markers", 10);
        done_pub_ = create_publisher<std_msgs::msg::Empty>("exploration/finished", rclcpp::QoS(1).transient_local());
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", rclcpp::QoS(1).reliable(), [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { on_map(*m); });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
    }

private:
    void on_map(const nav_msgs::msg::OccupancyGrid& msg) {
        le::GridMap m(static_cast<int>(msg.info.width), static_cast<int>(msg.info.height), msg.info.resolution,
                      Eigen::Vector2d(msg.info.origin.position.x, msg.info.origin.position.y));
        m.data.assign(msg.data.begin(), msg.data.end());
        explorer_->set_map(le::pad(m, pad_margin_));
    }

    void tick() {
        if (finished_) return;
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
        } catch (const tf2::TransformException&) {
            return;  // no map yet
        }
        const auto& r = tf.transform.rotation;
        const mp::Pose q(tf.transform.translation.x, tf.transform.translation.y,
                         std::atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z)));
        const double t = now().seconds();
        const le::Command c = explorer_->update(q, t);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = c.v;
        cmd.angular.z = c.omega;
        cmd_pub_->publish(cmd);

        if (explorer_->last_event() != last_event_) {
            last_event_ = explorer_->last_event();
            RCLCPP_INFO(get_logger(), "[%s] %s", le::to_string(explorer_->state()).c_str(), last_event_.c_str());
            publish_path();
        }
        if (t - last_markers_ > 0.5) {
            publish_markers();
            last_markers_ = t;
        }
        if (explorer_->state() == le::ExplorerState::Finished) {
            finished_ = true;
            cmd_pub_->publish(geometry_msgs::msg::Twist{});
            done_pub_->publish(std_msgs::msg::Empty{});
            RCLCPP_INFO(get_logger(), "exploration finished: %d goals reached, %d replans",
                        explorer_->goals_reached(), explorer_->replans());
        }
    }

    void publish_path() {
        nav_msgs::msg::Path path;
        path.header.frame_id = map_frame_;
        path.header.stamp = now();
        const mp::Trajectory& p = explorer_->path();
        for (int i = 0; i < p.rows(); ++i) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = p(i, 0);
            ps.pose.position.y = p(i, 1);
            ps.pose.orientation.z = std::sin(0.5 * p(i, 2));
            ps.pose.orientation.w = std::cos(0.5 * p(i, 2));
            path.poses.push_back(ps);
        }
        path_pub_->publish(path);
    }

    void publish_markers() {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker clear;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);
        auto marker = [this](const std::string& ns, int type, double scale, double r, double g, double b) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp = now();
            m.ns = ns;
            m.type = type;
            m.scale.x = m.scale.y = m.scale.z = scale;
            m.color.r = r;
            m.color.g = g;
            m.color.b = b;
            m.color.a = 1.0;
            m.pose.orientation.w = 1.0;
            return m;
        };
        auto point = [](const Eigen::Vector2d& p) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = 0.05;
            return pt;
        };

        const le::GridMap& map = explorer_->map();
        auto frontiers = marker("frontiers", visualization_msgs::msg::Marker::POINTS, map.resolution, 1.0, 0.5, 0.0);
        for (const auto& f : explorer_->frontiers())
            for (const auto& c : f.cells) frontiers.points.push_back(point(map.cell_center(c)));
        arr.markers.push_back(frontiers);

        auto blocked = marker("blocked goals", visualization_msgs::msg::Marker::SPHERE_LIST, 0.15, 0.8, 0.1, 0.1);
        for (const auto& b : explorer_->blocked_goals()) blocked.points.push_back(point(b));
        if (!blocked.points.empty()) arr.markers.push_back(blocked);

        if (const auto& goal = explorer_->goal()) {
            auto arrow = marker("goal", visualization_msgs::msg::Marker::ARROW, 0.1, 0.1, 0.7, 0.2);
            arrow.scale.x = 0.5;
            arrow.scale.y = arrow.scale.z = 0.1;
            arrow.pose.position = point(goal->pose.head<2>());
            arrow.pose.orientation.z = std::sin(0.5 * goal->pose.z());
            arrow.pose.orientation.w = std::cos(0.5 * goal->pose.z());
            arr.markers.push_back(arrow);
        }
        marker_pub_->publish(arr);
    }

    std::unique_ptr<le::Explorer> explorer_;
    std::string map_frame_ = "map", base_frame_ = "base_footprint", last_event_;
    bool finished_ = false;
    double last_markers_ = 0.0, pad_margin_ = 1.0;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExplorerNode>());
    rclcpp::shutdown();
    return 0;
}
