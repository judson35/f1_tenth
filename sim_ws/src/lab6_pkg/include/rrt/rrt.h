// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
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
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>

#define ROUND(a, b) ((int)(a / b) * b)

using namespace std;
using namespace std::chrono_literals;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

namespace {
    //String parameter names
    std::string POSE_TOPIC = "pose_topic";
    std::string SCAN_TOPIC = "scan_topic";
    std::string DRIVE_TOPIC = "drive_topic";
    std::string OCCUPANCY_GRID_TOPIC = "occupancy_grid_topic";

    //Int parameter names
    std::string OCCUPANCY_GRID_OBSTACLE_PADDING = "occupancy_grid_obstacle_padding";

    //Double parameter names
    std::string OCCUPANCY_GRID_DENSITY = "occupancy_grid_density";
    std::string OCCUPANCY_GRID_MAX_RANGE = "occupancy_grid_max_range";
    std::string OCCUPANCY_GRID_THRESHOLD = "occupancy_grid_threshold";
}


class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // TODO: add the publishers and subscribers you need

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupancy_grid_publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::TimerBase::SharedPtr occupancy_grid_visualization_timer_;
    
    //Parameters
    std::string pose_topic;
    std::string scan_topic;
    std::string drive_topic;
    std::string occupancy_grid_topic;

    int occupancy_grid_obstacle_padding;

    double occupancy_grid_density;
    double occupancy_grid_max_range;
    double occupancy_grid_threshold;

    std::map<double,std::map<double,bool>> occupancy_grid;
    double occupancy_grid_point_spacing;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    //Initialization/helper functions
    void initialize_parameters();
    void initialize_string_parameter(std::string parameter_name, std::string *parameter);
    void initialize_int_parameter(std::string parameter_name, int *parameter);
    void initialize_double_parameter(std::string parameter_name, double *parameter);

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void occupancy_grid_visualization_timer_callback_();

    //Visualization
    void publish_occupancy_grid(std::map<double,std::map<double,bool>> occupancy_grid, std::string frame_id);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};

