// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
    param_desc.description = "Pure Pursuit Parameters";

    this->initialize_parameters();
    this->occupancy_grid_point_spacing = (2*this->occupancy_grid_max_range)/(this->occupancy_grid_density-1);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    occupancy_grid_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->occupancy_grid_topic, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      this->pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      this->scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    occupancy_grid_visualization_timer_ = this->create_wall_timer(1ms, std::bind(&RRT::occupancy_grid_visualization_timer_callback_, this));

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::initialize_parameters() {
    this->initialize_string_parameter(POSE_TOPIC, &this->pose_topic);
    this->initialize_string_parameter(SCAN_TOPIC, &this->scan_topic);
    this->initialize_string_parameter(DRIVE_TOPIC, &this->drive_topic);
    this->initialize_string_parameter(OCCUPANCY_GRID_TOPIC, &this->occupancy_grid_topic);

    this->initialize_int_parameter(OCCUPANCY_GRID_OBSTACLE_PADDING, &this->occupancy_grid_obstacle_padding);

    this->initialize_double_parameter(OCCUPANCY_GRID_DENSITY, &this->occupancy_grid_density);
    this->initialize_double_parameter(OCCUPANCY_GRID_MAX_RANGE, &this->occupancy_grid_max_range);
    this->initialize_double_parameter(OCCUPANCY_GRID_THRESHOLD, &this->occupancy_grid_threshold);
}

void RRT::initialize_string_parameter(std::string parameter_name, std::string *parameter) {
    this->declare_parameter(parameter_name, "");
    *parameter = this->get_parameter(parameter_name).as_string();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": " + *parameter);
    return;
}

void RRT::initialize_int_parameter(std::string parameter_name, int *parameter) {
    this->declare_parameter(parameter_name, 0);
    *parameter = this->get_parameter(parameter_name).as_int();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", *parameter);
    return;
}

void RRT::initialize_double_parameter(std::string parameter_name, double *parameter) {
    this->declare_parameter(parameter_name, 0.0);
    *parameter = this->get_parameter(parameter_name).as_double();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", *parameter);
    return;
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    //Update occupancy grid based on lidar scan
    double theta = scan_msg->angle_min;
    double x,y;
    std::map<double,std::map<double,bool>> occupancy_grid;
    for (int i = 0; i < (int)scan_msg->ranges.size(); i++) {
        x = ROUND(scan_msg->ranges[i]*cos(theta), this->occupancy_grid_density);
        y = ROUND(scan_msg->ranges[i]*sin(theta), this->occupancy_grid_density);
        if (scan_msg->ranges[i] < this->occupancy_grid_threshold) {
            for (double j=x-(this->occupancy_grid_obstacle_padding*this->occupancy_grid_density); j <= x+(this->occupancy_grid_obstacle_padding*this->occupancy_grid_density); j+=this->occupancy_grid_density) {
                for (double k=y-(this->occupancy_grid_obstacle_padding*this->occupancy_grid_density); k <= y+(this->occupancy_grid_obstacle_padding*this->occupancy_grid_density); k+=this->occupancy_grid_density) {
                    occupancy_grid[j][k] = 1.0;
                }
            }
        }
        theta += scan_msg->angle_increment;
    }
    this->occupancy_grid = occupancy_grid;
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

void RRT::occupancy_grid_visualization_timer_callback_() {
    this->publish_occupancy_grid(this->occupancy_grid, "ego_racecar/base_link");
}

void RRT::publish_occupancy_grid(std::map<double,std::map<double,bool>> occupancy_grid, std::string frame_id) {
    auto marker = visualization_msgs::msg::Marker();

    marker.type = 6;
    marker.action = 0;
    marker.header.frame_id = frame_id;

    marker.ns = "occupancy_grid";

    marker.scale.x = this->occupancy_grid_density;
    marker.scale.y = this->occupancy_grid_density;
    marker.scale.z = this->occupancy_grid_density;

    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;

    std::map<double,std::map<double,bool>>::iterator x;
    for (x = this->occupancy_grid.begin(); x != this->occupancy_grid.end(); x++) {
        std::map<double,bool>::iterator y;
        for (y = x->second.begin(); y != x->second.end(); y++) {
            auto point = geometry_msgs::msg::Point();
            auto color = std_msgs::msg::ColorRGBA();

            point.x = x->first;
            point.y = y->first;
            point.z = 0.0;

            color.r = (float)y->second;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 1.0;

            marker.points.push_back(point);
            marker.colors.push_back(color);
        }
    }

    marker.lifetime.sec = 0.01;
    this->occupancy_grid_publisher_->publish(marker);
}