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
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), x_dist(-4.0,4.0), y_dist(0.0,5.0) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
    param_desc.description = "Pure Pursuit Parameters";

    this->initialize_parameters();
    this->occupancy_grid_point_spacing = (2*this->occupancy_grid_max_range)/(this->occupancy_grid_density-1);

    load_waypoints(this->waypoints_filename);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    this->occupancy_grid_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->occupancy_grid_visualization_topic, 1);
    rrt_tree_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(this->rrt_tree_visualization_topic, 1);
    this->waypoints_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->waypoints_visualization_topic, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->occupancy_grid_visualization_timer_ = this->create_wall_timer(1ms, std::bind(&RRT::occupancy_grid_visualization_timer_callback_, this));
    this->waypoints_visualization_timer_ = this->create_wall_timer(1ms, std::bind(&RRT::waypoints_visualization_timer_callback_, this));
    // this->rrt_timer_ = this->create_wall_timer(1ms, std::bind(&RRT::rrt_timer_callback_, this));

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::initialize_parameters() {
    this->initialize_string_parameter(POSE_TOPIC, &this->pose_topic);
    this->initialize_string_parameter(SCAN_TOPIC, &this->scan_topic);
    this->initialize_string_parameter(DRIVE_TOPIC, &this->drive_topic);
    this->initialize_string_parameter(OCCUPANCY_GRID_VISUALIZATION_TOPIC, &this->occupancy_grid_visualization_topic);
    this->initialize_string_parameter(RRT_TREE_VISUALIZATION_TOPIC, &this->rrt_tree_visualization_topic);
    this->initialize_string_parameter(WAYPOINTS_VISUALIZATION_TOPIC, &this->waypoints_visualization_topic);
    this->initialize_string_parameter(TARGET_WAYPOINT_VISUALIZATION_TOPIC, &this->target_waypoint_visualization_topic);
    this->initialize_string_parameter(WAYPOINTS_FILENAME, &this->waypoints_filename);

    this->initialize_int_parameter(OCCUPANCY_GRID_OBSTACLE_PADDING, &this->occupancy_grid_obstacle_padding);

    this->initialize_double_parameter(OCCUPANCY_GRID_DENSITY, &this->occupancy_grid_density);
    this->initialize_double_parameter(OCCUPANCY_GRID_MAX_RANGE, &this->occupancy_grid_max_range);
    this->initialize_double_parameter(OCCUPANCY_GRID_THRESHOLD, &this->occupancy_grid_threshold);
    this->initialize_double_parameter(RRT_GOAL_DIST_THRESHOLD, &this->rrt_goal_dist_threshold);
    this->initialize_double_parameter(RRT_MAX_EXPANSION_DIST, &this->rrt_max_expansion_dist);
    this->initialize_double_parameter(RRT_VISUALIZATION_POINT_SCALE, &this->rrt_tree_visualization_point_scale);
    this->initialize_double_parameter(RRT_VISUALIZATION_EDGE_SCALE, &this->rrt_tree_visualization_edge_scale);
    this->initialize_double_parameter(LOOKAHEAD, &this->L);
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

void RRT::load_waypoints(std::string filename) {
        RCLCPP_INFO(this->get_logger(), "Loading Waypoints");
        
        std::fstream fin;
        fin.open(filename, std::ios::in);

        std::string line, value;

        while (!fin.eof()) {
            std::getline(fin, line);
            std::stringstream s(line);
            
            std::vector<double> row;
            while (std::getline(s, value, ',')) {
                row.push_back(std::stod(value));
            }

            this->global_waypoints.push_back(Waypoint(row));
        }

        this->global_waypoints[0].is_target_waypoint = true;
        RCLCPP_INFO(this->get_logger(), "Waypoints Successfully Loaded");
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
    this->update_target_waypoint(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = map_frame_id;
    t.child_frame_id = car_frame_id;

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = pose_msg->pose.pose.position.x;
    t.transform.translation.y = pose_msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.x = pose_msg->pose.pose.orientation.x;
    t.transform.rotation.y = pose_msg->pose.pose.orientation.y;
    t.transform.rotation.z = pose_msg->pose.pose.orientation.z;
    t.transform.rotation.w = pose_msg->pose.pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

}

void RRT::rrt_timer_callback_() {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    geometry_msgs::msg::TransformStamped tf_result;

    try {// Parent --> child frame transformation
        tf_result = this->tf_buffer_->lookupTransform(this->map_frame_id, this->car_frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", this->car_frame_id.c_str(), this->map_frame_id.c_str(), ex.what());
        return;
    }

    tf2::Quaternion q(tf_result.transform.rotation.x, tf_result.transform.rotation.y, tf_result.transform.rotation.z, tf_result.transform.rotation.w);
    tf2::Vector3 p(tf_result.transform.translation.x, tf_result.transform.translation.y, tf_result.transform.translation.z);

    tf2::Transform transform(q, p);
    tf2::Vector3 point_in_parent_coordinates(this->global_waypoints[this->target_waypoint_idx].x, this->global_waypoints[this->target_waypoint_idx].y, 0.0);
    tf2::Vector3 point_in_child_coordinates = transform.inverse() * point_in_parent_coordinates;

    double x_goal = point_in_child_coordinates[0];
    double y_goal = point_in_child_coordinates[1];

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop
    RRT_Node node(0.0, 0.0, true);
    tree.push_back(node);

    while (!is_goal(node, x_goal, y_goal)) {
        std::vector<double> x_sample = this->sample();
        int nearest_idx = this->nearest(tree, x_sample);
        RRT_Node x_new = this->steer(tree[nearest_idx], x_sample);
        if (!this->check_collision(tree[nearest_idx], x_new)) {
            x_new.parent = nearest_idx;
            tree.push_back(x_new);
        }
    }

    this->tree = tree;
    // std::vector<RRT_Node> path = this->find_path(tree, current_node);
}

void RRT::update_target_waypoint(double x, double y) { //return the index of the closest waypoint to the car

    for (auto i = this->target_waypoint_idx; i < global_waypoints.size(); i++) {
        double dist = sqrt(pow((x - this->global_waypoints[i].x), 2) + pow((y - this->global_waypoints[i].y), 2)); //Calculate distance to point

        if (dist < this->L) { //If point is too close --> continue to next point
            this->target_waypoint_idx = i;

        } else if (dist >= this->L) { //If point is too far --> interpolate and return point
            return;

        }
    }
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    double x_samp = ROUND(x_dist(gen), this->occupancy_grid_density);
    double y_samp = ROUND(y_dist(gen), this->occupancy_grid_density);

    std::vector<double> sampled_point = {x_samp, y_samp};
    
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
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < tree.size(); i++) {
        double dist = EUCLIDEAN_DISTANCE((tree[i].x - sampled_point[0]), (tree[i].y - sampled_point[1]));
        nearest_node = (dist >= min_dist) * nearest_node + (dist < min_dist) * i;
    }

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
    //Travel from current node out to sampled point by max expansion dist (if the distance to the sampled point is less than max expansion dist)

    double x_delt = sampled_point[0] - nearest_node.x;
    double y_delt = sampled_point[1] - nearest_node.y;

    double dist = EUCLIDEAN_DISTANCE((nearest_node.x - sampled_point[0]), (nearest_node.y - sampled_point[1]));
    bool CHECK_MAX_DIST = dist > this->rrt_max_expansion_dist;
    new_node.x = ROUND((sampled_point[0] / dist) * this->rrt_max_expansion_dist, this->occupancy_grid_density) * CHECK_MAX_DIST + ROUND(sampled_point[0], this->occupancy_grid_density);
    new_node.y = ROUND((sampled_point[1] / dist) * this->rrt_max_expansion_dist, this->occupancy_grid_density) * CHECK_MAX_DIST + ROUND(sampled_point[1], this->occupancy_grid_density);
    
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

    double delta_x = new_node.x - nearest_node.x;
    double delta_y = new_node.y - nearest_node.y;
    double slope = delta_y / delta_x;

    double check_dist = delta_x * (delta_x < delta_y) + delta_y * (delta_y <= delta_x);
    for (double check_x = ROUND(nearest_node.x,this->occupancy_grid_density); check_x <= new_node.x; check_x += this->occupancy_grid_density) {
        double check_y = ROUND(check_x * slope + nearest_node.y, this->occupancy_grid_density);
        try {
            this->occupancy_grid[check_x][check_y];
            return true; //x and y coordinates of line are in occupancy grid so return collision true
        } catch (...) {
            continue;
        }
    }

    return true;
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
    close_enough = EUCLIDEAN_DISTANCE((goal_x - latest_added_node.x), (goal_y - latest_added_node.y)) <= this->rrt_goal_dist_threshold;

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
    found_path.push_back(latest_added_node);

    int check_idx = latest_added_node.parent;

    while (!tree[check_idx].is_root) {
        found_path.push_back(tree[check_idx]);
        check_idx = tree[check_idx].parent;
    }

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

//Visualization
void RRT::occupancy_grid_visualization_timer_callback_() {
    this->publish_occupancy_grid(this->occupancy_grid, "ego_racecar/base_link");
}

void RRT::rrt_tree_visualization_timer_callback_() {
    this->publish_rrt_tree(this->tree, "ego_racecar/base_link");
}

void RRT::waypoints_visualization_timer_callback_() {
    this->publish_waypoints(this->global_waypoints, "map");
}

void RRT::publish_occupancy_grid(std::map<double,std::map<double,bool>> &occupancy_grid, std::string frame_id) {
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
    this->occupancy_grid_visualization_publisher_->publish(marker);
}

void RRT::publish_rrt_tree(std::vector<RRT_Node> &tree, std::string frame_id) {
    auto tree_vis_array = visualization_msgs::msg::MarkerArray();
    std::vector<int> closed_nodes;
    for (RRT_Node node : tree) {
        auto marker = visualization_msgs::msg::Marker();
    
        marker.type = 2;
        marker.action = 0;
        marker.header.frame_id = frame_id;

        marker.ns = "rrt_tree";

        marker.scale.x = this->rrt_tree_visualization_point_scale;
        marker.scale.y = this->rrt_tree_visualization_point_scale;
        marker.scale.z = this->rrt_tree_visualization_point_scale;

        marker.pose.position.x = node.x;
        marker.pose.position.y = node.y;
        marker.pose.position.z = 0.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.0;

        marker.lifetime.sec = 1;
        tree_vis_array.markers.push_back(marker);

        if (!node.is_root) {
            auto edge = visualization_msgs::msg::Marker();
    
            marker.type = 4;
            marker.action = 0;
            marker.header.frame_id = frame_id;

            marker.ns = "rrt_tree";

            marker.scale.x = this->rrt_tree_visualization_edge_scale;
            marker.scale.y = this->rrt_tree_visualization_edge_scale;
            marker.scale.z = this->rrt_tree_visualization_edge_scale;

            marker.pose.position.x = node.x;
            marker.pose.position.y = node.y;
            marker.pose.position.z = 0.0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            //Add node to list
            {
                auto point = geometry_msgs::msg::Point();
                auto color = std_msgs::msg::ColorRGBA();

                point.x = node.x;
                point.y = node.y;
                point.z = 0.0;

                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = 1.0;

                marker.points.push_back(point);
                marker.colors.push_back(color);
            }

            //Add parent to list
            {
                auto point = geometry_msgs::msg::Point();
                auto color = std_msgs::msg::ColorRGBA();

                point.x = tree[node.parent].x;
                point.y = tree[node.parent].y;
                point.z = 0.0;

                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = 1.0;

                marker.points.push_back(point);
                marker.colors.push_back(color);
            }

            marker.lifetime.sec = 1;
            tree_vis_array.markers.push_back(edge);
        }
    }

    this->rrt_tree_visualization_publisher_->publish(tree_vis_array);
}

void RRT::publish_waypoints(std::vector<Waypoint> &waypoints, std::string frame_id) {
    auto waypoints_msg = visualization_msgs::msg::Marker();
    waypoints_msg.type = 8;
    waypoints_msg.action = 0;
    waypoints_msg.header.frame_id = frame_id;

    waypoints_msg.ns = "waypoints";

    waypoints_msg.scale.x = this->rrt_tree_visualization_point_scale;
    waypoints_msg.scale.y = this->rrt_tree_visualization_point_scale;
    waypoints_msg.scale.z = this->rrt_tree_visualization_point_scale;

    waypoints_msg.pose.position.x = 0.0;
    waypoints_msg.pose.position.y = 0.0;
    waypoints_msg.pose.position.z = 0.0;

    waypoints_msg.pose.orientation.x = 0.0;
    waypoints_msg.pose.orientation.y = 0.0;
    waypoints_msg.pose.orientation.z = 0.0;
    waypoints_msg.pose.orientation.w = 1.0;

    for (Waypoint waypoint : waypoints) {
        auto point = geometry_msgs::msg::Point();
        auto color = std_msgs::msg::ColorRGBA();

        point.x = waypoint.x;
        point.y = waypoint.y;
        point.z = 0.0;

        // RCLCPP_INFO(this->get_logger(), "x: %f\t y: %f", waypoint.x, waypoint.x);

        color.r = !waypoint.is_target_waypoint;
        color.g = waypoint.is_target_waypoint;
        color.b = 0.0;
        color.a = 1.0;

        waypoints_msg.points.push_back(point);
        waypoints_msg.colors.push_back(color);
    }

    waypoints_msg.lifetime.sec = 1;
    this->waypoints_visualization_publisher_->publish(waypoints_msg);
}