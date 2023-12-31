// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cmath>
#include <vector>
#include <random>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/Point.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define ROUND(a, b) (round(a / b) * b)
#define EUCLIDEAN_DISTANCE(x_delt, y_delt) (sqrt(pow(x_delt,2) + pow(y_delt,2)))

using namespace std;
using namespace std::chrono_literals;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent = 0; // index of parent node in the tree vector
    bool is_root = false;
    double r_value;    
    RRT_Node(){}
    RRT_Node(double x, double y, bool is_root): x(x), y(y), is_root(is_root){}
    RRT_Node(double x, double y, bool is_root, double r_value): x(x), y(y), is_root(is_root), r_value(r_value){}

} RRT_Node;

typedef struct Waypoint {
    std::vector<double> position = {0.0,0.0};
    Waypoint(){}
    Waypoint(double x, double y): position({x, y}){}

} Waypoint;

namespace {
    //String parameter names
    std::string POSE_TOPIC = "pose_topic";
    std::string SCAN_TOPIC = "scan_topic";
    std::string DRIVE_TOPIC = "drive_topic";
    std::string OCCUPANCY_GRID_VISUALIZATION_TOPIC = "occupancy_grid_visualization_topic";
    std::string RRT_TREE_NODES_VISUALIZATION_TOPIC = "rrt_tree_nodes_visualization_topic";
    std::string RRT_TREE_EDGES_VISUALIZATION_TOPIC = "rrt_tree_edges_visualization_topic";
    std::string WAYPOINTS_VISUALIZATION_TOPIC = "waypoints_visualization_topic";
    std::string TARGET_WAYPOINT_VISUALIZATION_TOPIC = "target_waypoint_visualization_topic";
    std::string WAYPOINTS_FILENAME = "waypoints_filename";

    //Int parameter names
    std::string OCCUPANCY_GRID_OBSTACLE_PADDING = "occupancy_grid_obstacle_padding";

    //Double parameter names
    std::string OCCUPANCY_GRID_DENSITY = "occupancy_grid_density";
    std::string OCCUPANCY_GRID_MAX_RANGE = "occupancy_grid_max_range";
    std::string OCCUPANCY_GRID_THRESHOLD = "occupancy_grid_threshold";
    std::string RRT_GOAL_DIST_THRESHOLD = "rrt_goal_dist_threshold";
    std::string RRT_MAX_EXPANSION_DIST = "rrt_max_expansion_dist";
    std::string RRT_VISUALIZATION_POINT_SCALE = "rrt_tree_visualization_point_scale";
    std::string RRT_VISUALIZATION_EDGE_SCALE = "rrt_tree_visualization_edge_scale";
    std::string GLOBAL_LOOKAHEAD = "global_lookahead";
    std::string LOCAL_LOOKAHEAD = "local_lookahead";
    std::string SPEED_GAIN = "speed_gain";

    const int ALONG_X = 1;
    const int ALONG_Y = 2;
}

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    std::string map_frame_id = "map";
    std::string car_frame_id = "car";

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupancy_grid_visualization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_tree_nodes_visualization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_tree_edges_visualization_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr waypoints_visualization_publisher_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr rrt_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr drive_publisher_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr occupancy_grid_visualization_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr rrt_tree_visualization_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr waypoints_visualization_timer_{nullptr};

    std::vector<Waypoint> global_waypoints = std::vector<Waypoint>(); //global waypoints in map frame
    std::vector<Waypoint> local_waypoints = std::vector<Waypoint>(); //local waypoints in car frame (from RRT)

    Waypoint global_target_waypoint = Waypoint(); //In car frame
    Waypoint local_target_waypoint = Waypoint(); //In car frame

    int target_global_waypoint_idx = 0;
    int target_local_waypoint_idx = 0;
    
    //Parameters
    std::string pose_topic = std::string();
    std::string scan_topic = std::string();
    std::string drive_topic = std::string();
    std::string occupancy_grid_visualization_topic = std::string();
    std::string rrt_tree_nodes_visualization_topic = std::string();
    std::string rrt_tree_edges_visualization_topic = std::string();
    std::string waypoints_visualization_topic = std::string();
    std::string target_waypoint_visualization_topic = std::string();
    std::string waypoints_filename = std::string();

    int occupancy_grid_obstacle_padding = int();

    bool RRT_ACTIVE = true;
    bool DRIVE = false;

    double occupancy_grid_density = double();
    double occupancy_grid_max_range = double();
    double occupancy_grid_threshold = double();
    double rrt_goal_dist_threshold = double();
    double rrt_max_expansion_dist = double();
    double rrt_tree_visualization_point_scale = double();
    double rrt_tree_visualization_edge_scale = double();
    double global_lookahead = double();
    double local_lookahead = double();
    double speed_gain = double();

    std::map<double,std::map<double,bool>> occupancy_grid = std::map<double,std::map<double,bool>>();
    double occupancy_grid_point_spacing;

    std::vector<RRT_Node> tree = std::vector<RRT_Node>();
    std::vector<RRT_Node> sample_tree = std::vector<RRT_Node>();
    std::vector<RRT_Node> nearest_tree = std::vector<RRT_Node>();

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    //Initialization/helper functions
    void initialize_parameters();
    void initialize_string_parameter(std::string parameter_name, std::string &parameter);
    void initialize_int_parameter(std::string parameter_name, int &parameter);
    void initialize_double_parameter(std::string parameter_name, double &parameter);

    void load_waypoints(std::string filename);
    void update_target_waypoint(double x, double y, const std::vector<Waypoint> &waypoints, int &target_index, double lookahead);
    tf2::Vector3 interpolate_target_waypoint(const std::vector<Waypoint> &waypoints, const int &target_waypoint_index, double lookahead);
    void update_target_waypoints(double x, double y);

    // callbacks
    // updates transform for rrt_timer_callback_
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    void occupancy_grid_visualization_timer_callback_();
    void rrt_tree_visualization_timer_callback_();
    void global_waypoints_visualization_timer_callback_();
    void rrt_timer_callback_();
    void drive_controller_timer_callback_();

    //Visualization
    void publish_occupancy_grid(std::map<double,std::map<double,bool>> &occupancy_grid, std::string frame_id);
    void publish_rrt_tree_nodes(std::vector<RRT_Node> &tree, std::string frame_id);
    void publish_rrt_tree_edges(std::vector<RRT_Node> &tree, std::string frame_id);
    void publish_global_waypoints(std::vector<Waypoint> &waypoints, std::string frame_id);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<Waypoint> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};

