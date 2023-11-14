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
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), x_dist(0.0, 4.0), y_dist(-1.5,1.5) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
    param_desc.description = "Pure Pursuit Parameters";

    this->initialize_parameters();
    // this->declare_parameter(POSE_TOPIC, "");
    // this->declare_parameter(SCAN_TOPIC, "");
    // this->declare_parameter(DRIVE_TOPIC, "");
    // this->declare_parameter(OCCUPANCY_GRID_VISUALIZATION_TOPIC, "");
    // this->declare_parameter(RRT_TREE_NODES_VISUALIZATION_TOPIC, "");
    // this->declare_parameter(RRT_TREE_EDGES_VISUALIZATION_TOPIC, "");
    // this->declare_parameter(WAYPOINTS_VISUALIZATION_TOPIC, "");
    // this->declare_parameter(TARGET_WAYPOINT_VISUALIZATION_TOPIC, "");
    // this->declare_parameter(WAYPOINTS_FILENAME, "");

    // this->pose_topic = this->get_parameter(POSE_TOPIC).as_string();
    // this->scan_topic = this->get_parameter(SCAN_TOPIC).as_string();
    // this->drive_topic = this->get_parameter(DRIVE_TOPIC).as_string();
    // this->occupancy_grid_visualization_topic = this->get_parameter(OCCUPANCY_GRID_VISUALIZATION_TOPIC).as_string();
    // this->rrt_tree_nodes_visualization_topic = this->get_parameter(RRT_TREE_NODES_VISUALIZATION_TOPIC).as_string();
    // this->rrt_tree_edges_visualization_topic = this->get_parameter(RRT_TREE_EDGES_VISUALIZATION_TOPIC).as_string();
    // this->waypoints_visualization_topic = this->get_parameter(WAYPOINTS_VISUALIZATION_TOPIC).as_string();
    // this->target_waypoint_visualization_topic = this->get_parameter(TARGET_WAYPOINT_VISUALIZATION_TOPIC).as_string();
    // this->waypoints_filename = this->get_parameter(WAYPOINTS_FILENAME).as_string();

    // RCLCPP_INFO(this->get_logger(), "%s: %s", SCAN_TOPIC, this->scan_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", POSE_TOPIC, this->pose_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", DRIVE_TOPIC, this->drive_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", OCCUPANCY_GRID_VISUALIZATION_TOPIC, this->occupancy_grid_visualization_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", RRT_TREE_NODES_VISUALIZATION_TOPIC, this->rrt_tree_nodes_visualization_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", RRT_TREE_EDGES_VISUALIZATION_TOPIC, this->rrt_tree_edges_visualization_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", WAYPOINTS_VISUALIZATION_TOPIC, this->waypoints_visualization_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", TARGET_WAYPOINT_VISUALIZATION_TOPIC, this->target_waypoint_visualization_topic);
    // RCLCPP_INFO(this->get_logger(), "%s: %s", WAYPOINTS_FILENAME, this->waypoints_filename);


    this->occupancy_grid_point_spacing = (2*this->occupancy_grid_max_range)/(this->occupancy_grid_density-1);

    load_waypoints(this->waypoints_filename);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    this->occupancy_grid_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->occupancy_grid_visualization_topic, 1);
    this->rrt_tree_nodes_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->rrt_tree_nodes_visualization_topic, 1);
    this->rrt_tree_edges_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->rrt_tree_edges_visualization_topic, 1);
    this->waypoints_visualization_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(this->waypoints_visualization_topic, 1);
    this->drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 1);

    // ROS subscribers
    // TODO: create subscribers as you need
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(this->pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->occupancy_grid_visualization_timer_ = this->create_wall_timer(500ms, std::bind(&RRT::occupancy_grid_visualization_timer_callback_, this));
    this->rrt_tree_visualization_timer_ = this->create_wall_timer(500ms, std::bind(&RRT::rrt_tree_visualization_timer_callback_, this));
    this->waypoints_visualization_timer_ = this->create_wall_timer(500ms, std::bind(&RRT::global_waypoints_visualization_timer_callback_, this));
    this->rrt_timer_ = this->create_wall_timer(100ms, std::bind(&RRT::rrt_timer_callback_, this));
    this->drive_publisher_timer_ = this->create_wall_timer(10ms, std::bind(&RRT::drive_controller_timer_callback_, this));

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::initialize_parameters() {
    this->initialize_string_parameter(POSE_TOPIC, this->pose_topic);
    this->initialize_string_parameter(SCAN_TOPIC, this->scan_topic);
    this->initialize_string_parameter(DRIVE_TOPIC, this->drive_topic);
    this->initialize_string_parameter(OCCUPANCY_GRID_VISUALIZATION_TOPIC, this->occupancy_grid_visualization_topic);
    this->initialize_string_parameter(RRT_TREE_NODES_VISUALIZATION_TOPIC, this->rrt_tree_nodes_visualization_topic);
    this->initialize_string_parameter(RRT_TREE_EDGES_VISUALIZATION_TOPIC, this->rrt_tree_edges_visualization_topic);
    this->initialize_string_parameter(WAYPOINTS_VISUALIZATION_TOPIC, this->waypoints_visualization_topic);
    this->initialize_string_parameter(TARGET_WAYPOINT_VISUALIZATION_TOPIC, this->target_waypoint_visualization_topic);
    this->initialize_string_parameter(WAYPOINTS_FILENAME, this->waypoints_filename);

    this->initialize_int_parameter(OCCUPANCY_GRID_OBSTACLE_PADDING, this->occupancy_grid_obstacle_padding);

    this->initialize_double_parameter(OCCUPANCY_GRID_DENSITY, this->occupancy_grid_density);
    this->initialize_double_parameter(OCCUPANCY_GRID_MAX_RANGE, this->occupancy_grid_max_range);
    this->initialize_double_parameter(OCCUPANCY_GRID_THRESHOLD, this->occupancy_grid_threshold);
    this->initialize_double_parameter(RRT_GOAL_DIST_THRESHOLD, this->rrt_goal_dist_threshold);
    this->initialize_double_parameter(RRT_MAX_EXPANSION_DIST, this->rrt_max_expansion_dist);
    this->initialize_double_parameter(RRT_VISUALIZATION_POINT_SCALE, this->rrt_tree_visualization_point_scale);
    this->initialize_double_parameter(RRT_VISUALIZATION_EDGE_SCALE, this->rrt_tree_visualization_edge_scale);
    this->initialize_double_parameter(GLOBAL_LOOKAHEAD, this->global_lookahead);
    this->initialize_double_parameter(LOCAL_LOOKAHEAD, this->local_lookahead);
    this->initialize_double_parameter(SPEED_GAIN, this->speed_gain);
}

void RRT::initialize_string_parameter(std::string parameter_name, std::string &parameter) {
    this->declare_parameter(parameter_name, "");
    parameter = this->get_parameter(parameter_name).as_string();
    RCLCPP_INFO(this->get_logger(), "%s: %s", parameter_name.c_str(), parameter.c_str());
    return;
}

void RRT::initialize_int_parameter(std::string parameter_name, int &parameter) {
    this->declare_parameter(parameter_name, 0);
    parameter = this->get_parameter(parameter_name).as_int();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", parameter);
    return;
}

void RRT::initialize_double_parameter(std::string parameter_name, double &parameter) {
    this->declare_parameter(parameter_name, 0.0);
    parameter = this->get_parameter(parameter_name).as_double();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", parameter);
    return;
}

void RRT::load_waypoints(std::string filename) {
        RCLCPP_INFO(this->get_logger(), "Loading Waypoints");
        RCLCPP_INFO(this->get_logger(), "Filename: %s", filename.c_str());
        
        std::fstream fin;
        fin.open(filename, std::ios::in);

        std::string line, value;

        int idx = 0;

        while (!fin.eof()) {
            std::getline(fin, line);
            std::stringstream s(line);
            
            if (idx % 10 == 0) {
                std::vector<double> row;
                while (std::getline(s, value, ',')) {
                    row.push_back(std::stod(value));
                }
                this->global_waypoints.push_back(Waypoint(row[0], row[1]));
            }
            idx++;
        }
        this->target_global_waypoint_idx = 0;
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
                    occupancy_grid[j][k] = true;
                }
            }
        }
        theta += scan_msg->angle_increment;
    }
    this->occupancy_grid = occupancy_grid;
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {

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

    this->update_target_waypoints(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y); //Update target waypoints and put into car frame
    if (!this->DRIVE) this->DRIVE = true;
}

void RRT::rrt_timer_callback_() {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop
    RRT_Node latest_added_node(0.2, 0.0, true);
    tree.push_back(latest_added_node);

    // RCLCPP_INFO(this->get_logger(), "Searching for Goal");
    for (int i = 0; i < 200; i++) {
        std::vector<double> x_sample = this->sample();
        int nearest_idx = this->nearest(tree, x_sample);
        RRT_Node check_node = this->steer(tree[nearest_idx], x_sample);
        check_node.parent = nearest_idx;
        if (!this->check_collision(tree[nearest_idx], check_node)) {
            latest_added_node = check_node;
            tree.push_back(check_node);
            if (is_goal(check_node, this->global_target_waypoint.position[0], this->global_target_waypoint.position[1])) break;
        }
    }

    this->tree = tree;

    if (latest_added_node.parent > 0) {
        int index = this->nearest(tree, this->global_target_waypoint.position);
        // RCLCPP_INFO(this->get_logger(), "Check Index: %i", index);
        // RCLCPP_INFO(this->get_logger(), "Goal found. Finding Path");
        this->local_waypoints = this->find_path(tree, tree[index]);
        this->target_local_waypoint_idx = 0;
        if (!this->RRT_ACTIVE) this->RRT_ACTIVE = true;
    }
}

void RRT::drive_controller_timer_callback_() {
    if (this->DRIVE) {
        // RCLCPP_INFO(this->get_logger(), "Sending Drive Command");
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        double gamma = 2*abs(this->local_target_waypoint.position[1])/pow(this->local_lookahead,2);

        double steering_angle = atan2(gamma * this->local_lookahead, 1.0) * ((this->local_target_waypoint.position[1] >= 0) - (this->local_target_waypoint.position[1] < 0));

        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = this->speed_gain;
        drive_msg.drive.steering_angle_velocity = 0.001;

        this->drive_publisher_->publish(drive_msg);
        // RCLCPP_INFO(this->get_logger(), "Drive Command Sent");
    }
}

void RRT::update_target_waypoint(double x, double y, const std::vector<Waypoint> &waypoints, int &target_index, double lookahead) { //return the index of the closest waypoint to the car

    double dist = 0.0; //Calculate distance to point

    for (int i = target_index; i < waypoints.size(); i++) {
        dist = sqrt(pow((x - waypoints[i].position[0]), 2) + pow((y - waypoints[i].position[1]), 2));

        if (dist < lookahead) {
            if (i >= waypoints.size()-2) {
                target_index = 0; //Reset start index
                break;
            } else {
                target_index = i;
            }
        } else {
            return;
        }
    }
}

tf2::Vector3 RRT::interpolate_target_waypoint(const std::vector<Waypoint> &waypoints, const int &target_waypoint_index, double lookahead) {
    if (waypoints.size() > 0) {
        // RCLCPP_INFO(this->get_logger(), "Waypoints Size: %i", waypoints.size());
        double theta = atan2((waypoints[target_waypoint_index+1].position[1] - waypoints[target_waypoint_index].position[1]), (waypoints[target_waypoint_index+1].position[0] - waypoints[target_waypoint_index].position[0]));
        return tf2::Vector3(waypoints[target_waypoint_index].position[0] + lookahead * cos(theta), waypoints[target_waypoint_index].position[1] + lookahead * sin(theta), 0.0);
    } else {
        return tf2::Vector3(0.0, 0.0, 0.0);
    }
}

void RRT::update_target_waypoints(double x, double y) {
    //Update both global and local target waypoints and transform them into car frame.
    
    geometry_msgs::msg::TransformStamped tf_result;

    try {// Parent --> child frame transformation
        tf_result = this->tf_buffer_->lookupTransform(this->map_frame_id, this->car_frame_id, tf2::TimePointZero);

        tf2::Quaternion q(tf_result.transform.rotation.x, tf_result.transform.rotation.y, tf_result.transform.rotation.z, tf_result.transform.rotation.w);
        tf2::Vector3 p(tf_result.transform.translation.x, tf_result.transform.translation.y, tf_result.transform.translation.z);

        tf2::Transform transform(q, p);

        // RCLCPP_INFO(this->get_logger(), "Updating Global Target Waypoint");
        this->update_target_waypoint(x, y, this->global_waypoints, this->target_global_waypoint_idx, this->global_lookahead);

        tf2::Vector3 global_target_waypoint_in_parent_coordinates = this->interpolate_target_waypoint(this->global_waypoints, this->target_global_waypoint_idx, this->global_lookahead);;
        tf2::Vector3 global_target_waypoint_in_child_coordinates = transform.inverse() * global_target_waypoint_in_parent_coordinates;
        RCLCPP_INFO(this->get_logger(), "Global Target Waypoint X: %f\t Y: %f", global_target_waypoint_in_child_coordinates[0], global_target_waypoint_in_child_coordinates[1]);
        this->global_target_waypoint = Waypoint(global_target_waypoint_in_child_coordinates[0], global_target_waypoint_in_child_coordinates[1]);

        if (this->RRT_ACTIVE) {
            // RCLCPP_INFO(this->get_logger(), "Updating Local Target Waypoint");
            this->update_target_waypoint(x, y, this->local_waypoints, this->target_local_waypoint_idx, this->local_lookahead);

            tf2::Vector3 local_target_waypoint_in_parent_coordinates = this->interpolate_target_waypoint(this->local_waypoints, this->target_local_waypoint_idx, this->local_lookahead);;
            tf2::Vector3 local_target_waypoint_in_child_coordinates = transform.inverse() * local_target_waypoint_in_parent_coordinates;
            RCLCPP_INFO(this->get_logger(), "Local Target Waypoint X: %f\t Y: %f\t Index: %i", local_target_waypoint_in_child_coordinates[0], local_target_waypoint_in_child_coordinates[1], this->target_local_waypoint_idx);
            this->local_target_waypoint = Waypoint(local_target_waypoint_in_child_coordinates[0], local_target_waypoint_in_child_coordinates[1]);
        }
        
        // RCLCPP_INFO(this->get_logger(), "Waypoints Updated");
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", this->car_frame_id.c_str(), this->map_frame_id.c_str(), ex.what());
        return;
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
    double min_dist = 100.0;
    for (int i = 0; i < tree.size(); i++) {
        double dist = EUCLIDEAN_DISTANCE((tree[i].x - sampled_point[0]), (tree[i].y - sampled_point[1]));
        nearest_node = (dist >= min_dist) * nearest_node + (dist < min_dist) * i;
        min_dist = (dist >= min_dist) * min_dist + (dist < min_dist) * dist;
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
    new_node.x = ((x_delt / dist) * this->rrt_max_expansion_dist) * CHECK_MAX_DIST + nearest_node.x;
    new_node.y = ((y_delt / dist) * this->rrt_max_expansion_dist) * CHECK_MAX_DIST + nearest_node.y;

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
    // RCLCPP_INFO(this->get_logger(), "Checking Collision");

    bool collision = false;
    double start_x = ROUND(nearest_node.x, this->occupancy_grid_density);
    double start_y = ROUND(nearest_node.y, this->occupancy_grid_density);
    double goal_x = ROUND(new_node.x, this->occupancy_grid_density);
    double goal_y = ROUND(new_node.y, this->occupancy_grid_density);

    double delta_x = goal_x - start_x;
    double delta_y = goal_y - start_y;
    double slope = delta_y / delta_x;
    double slope2 = delta_x / delta_y;

    int CHECK_FUNCTION = ALONG_X *  + ALONG_Y * (delta_x == 0.0);

    if (delta_y == 0.0) {
        double check_y = ROUND(start_y, this->occupancy_grid_density);
        for (double check_x = start_x; check_x <= goal_x; check_x += this->occupancy_grid_density) {
            // RCLCPP_INFO(this->get_logger(), "Check: %i", this->occupancy_grid[check_x][check_y]);
            if (this->occupancy_grid[check_x][check_y]) return true;
            // else this->occupancy_grid[check_x][check_y] = false;
        }
    } else if (delta_x == 0.0) {
        double increment = this->occupancy_grid_density * (delta_y / abs(delta_y));
        double check_x = ROUND(start_x, this->occupancy_grid_density);
        for (double check_y = start_y; abs(check_y) <= abs(goal_y); check_y += increment) {
            // RCLCPP_INFO(this->get_logger(), "Check: %i", this->occupancy_grid[check_x][check_y]);
            if (this->occupancy_grid[check_x][check_y]) return true;
            // else this->occupancy_grid[check_x][check_y] = false;;
        }
    } else {
        for (double check_x = 0.0; check_x <= delta_x; check_x += this->occupancy_grid_density) {
            double check_y = check_x * slope + start_y;
            check_y = ROUND(check_y, this->occupancy_grid_density);
            // RCLCPP_INFO(this->get_logger(), "Check: %i", this->occupancy_grid[check_x + start_x][check_y]);
            if (this->occupancy_grid[check_x + start_x][check_y]) return true;
            // else this->occupancy_grid[check_x + start_x][check_y] = false;
        }

        double increment = this->occupancy_grid_density * (delta_y / abs(delta_y));
        for (double check_y = 0.0; abs(check_y) <= abs(delta_y); check_y += increment) {
            double check_x = check_y * slope2 + start_x;
            check_x = ROUND(check_x, this->occupancy_grid_density);
            // RCLCPP_INFO(this->get_logger(), "Check X: %f\t Check Y: %f", check_x, check_y + start_y);
            // RCLCPP_INFO(this->get_logger(), "Check: %i", this->occupancy_grid[check_x][check_y + start_y]);
            if (this->occupancy_grid[check_x][check_y + start_y]) return true;
            // else this->occupancy_grid[check_x][check_y + start_y] = false;;
        }
    }
    return false;
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

std::vector<Waypoint> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    std::vector<Waypoint> found_path;
    found_path.push_back(Waypoint(latest_added_node.x, latest_added_node.y));
    // RCLCPP_INFO(this->get_logger(), "%i", latest_added_node.parent);

    try {
        int check_idx = latest_added_node.parent;

        // RCLCPP_INFO(this->get_logger(), "Check_idx: %i\t Is_Root: %i", check_idx, tree[check_idx].is_root);

        while (!tree[check_idx].is_root) {
            found_path.push_back(Waypoint(tree[check_idx].x, tree[check_idx].y));
            check_idx = tree[check_idx].parent;
        }

        // RCLCPP_INFO(this->get_logger(), "Found Path Size: %i", found_path.size());

        return std::vector<Waypoint>(found_path.rbegin(), found_path.rend());
    } catch(...) {
        RCLCPP_INFO(this->get_logger(), "No valid points found");
        return found_path;
    }
    
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
    auto delete_msg = visualization_msgs::msg::Marker();
    delete_msg.action = 2;
    this->occupancy_grid_visualization_publisher_->publish(delete_msg);
    this->publish_occupancy_grid(this->occupancy_grid, "ego_racecar/base_link");
}

void RRT::rrt_tree_visualization_timer_callback_() {
    auto delete_msg = visualization_msgs::msg::Marker();
    delete_msg.action = 2;
    this->rrt_tree_nodes_visualization_publisher_->publish(delete_msg);
    this->rrt_tree_edges_visualization_publisher_->publish(delete_msg);
    this->publish_rrt_tree_nodes(this->tree, "ego_racecar/base_link");
    this->publish_rrt_tree_edges(this->tree, "ego_racecar/base_link");
}

void RRT::global_waypoints_visualization_timer_callback_() {
    auto delete_msg = visualization_msgs::msg::Marker();
    delete_msg.action = 2;
    this->waypoints_visualization_publisher_->publish(delete_msg);
    this->publish_global_waypoints(this->global_waypoints, "map");
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
            // if (y->second) {
                auto point = geometry_msgs::msg::Point();
                auto color = std_msgs::msg::ColorRGBA();

                point.x = x->first;
                point.y = y->first;
                point.z = 0.0;

                color.r = (float)y->second;
                color.g = (float)!y->second;
                color.b = 0.0;
                color.a = 1.0;

                marker.points.push_back(point);
                marker.colors.push_back(color);
            // }
        }
    }

    this->occupancy_grid_visualization_publisher_->publish(marker);
}

void RRT::publish_rrt_tree_nodes(std::vector<RRT_Node> &tree, std::string frame_id) {
    auto tree_nodes = visualization_msgs::msg::Marker();

    tree_nodes.type = 7;
    tree_nodes.action = 0;
    tree_nodes.header.frame_id = frame_id;

    tree_nodes.ns = "rrt_tree_nodes";

    tree_nodes.scale.x = this->rrt_tree_visualization_point_scale;
    tree_nodes.scale.y = this->rrt_tree_visualization_point_scale;
    tree_nodes.scale.z = this->rrt_tree_visualization_point_scale;

    tree_nodes.pose.position.x = 0.0;
    tree_nodes.pose.position.y = 0.0;
    tree_nodes.pose.position.z = 0.0;

    tree_nodes.pose.orientation.x = 0.0;
    tree_nodes.pose.orientation.y = 0.0;
    tree_nodes.pose.orientation.z = 0.0;
    tree_nodes.pose.orientation.w = 1.0;

    for (int i = 0; i < this->local_waypoints.size(); i++) {
        auto point = geometry_msgs::msg::Point();
        auto color = std_msgs::msg::ColorRGBA();

        point.x = this->local_waypoints[i].position[0];
        point.y = this->local_waypoints[i].position[1];
        point.z = 0.0;

        color.r = 0.0;
        color.g = (float)(this->target_local_waypoint_idx == i);
        color.b = (float)(this->target_local_waypoint_idx != i);
        color.a = 1.0;

        tree_nodes.points.push_back(point);
        tree_nodes.colors.push_back(color);
    }

    auto point = geometry_msgs::msg::Point();
    auto color = std_msgs::msg::ColorRGBA();

    point.x = this->global_target_waypoint.position[0];
    point.y = this->global_target_waypoint.position[1];
    point.z = 0.0;

    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 1.0;

    tree_nodes.points.push_back(point);
    tree_nodes.colors.push_back(color);

    this->rrt_tree_nodes_visualization_publisher_->publish(tree_nodes);
}

void RRT::publish_rrt_tree_edges(std::vector<RRT_Node> &tree, std::string frame_id) {
    auto tree_edges = visualization_msgs::msg::Marker();

    tree_edges.type = 5;
    tree_edges.action = 0;
    tree_edges.header.frame_id = frame_id;

    tree_edges.ns = "rrt_tree_edges";

    tree_edges.scale.x = this->rrt_tree_visualization_edge_scale;
    tree_edges.scale.y = this->rrt_tree_visualization_edge_scale;
    tree_edges.scale.z = this->rrt_tree_visualization_edge_scale;

    tree_edges.pose.position.x = 0.0;
    tree_edges.pose.position.y = 0.0;
    tree_edges.pose.position.z = 0.0;

    tree_edges.pose.orientation.x = 0.0;
    tree_edges.pose.orientation.y = 0.0;
    tree_edges.pose.orientation.z = 0.0;
    tree_edges.pose.orientation.w = 1.0;

    for (int i = 0; i < this->local_waypoints.size()-1; i++) {
        auto point = geometry_msgs::msg::Point();
        auto point_parent = geometry_msgs::msg::Point();
        auto color = std_msgs::msg::ColorRGBA();
        auto color_parent = std_msgs::msg::ColorRGBA();

        point.x = this->local_waypoints[i].position[0];
        point.y = this->local_waypoints[i].position[1];
        point.z = 0.0;

        point_parent.x = this->local_waypoints[i+1].position[0];
        point_parent.y = this->local_waypoints[i+1].position[1];
        point_parent.z = 0.0;

        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;

        color_parent.r = 0.0;
        color_parent.g = 1.0;
        color_parent.b = 0.0;
        color_parent.a = 1.0;

        tree_edges.points.push_back(point);
        tree_edges.points.push_back(point_parent);
        tree_edges.colors.push_back(color);
        tree_edges.colors.push_back(color_parent);
    }

    this->rrt_tree_edges_visualization_publisher_->publish(tree_edges);
}

void RRT::publish_global_waypoints(std::vector<Waypoint> &waypoints, std::string frame_id) {
    auto waypoints_msg = visualization_msgs::msg::Marker();
    waypoints_msg.type = 7;
    waypoints_msg.action = 0;
    waypoints_msg.header.frame_id = frame_id;

    waypoints_msg.ns = "global_waypoints";

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

    for (int i = 0; i < this->global_waypoints.size(); i++) {
        auto point = geometry_msgs::msg::Point();
        auto color = std_msgs::msg::ColorRGBA();

        point.x = this->global_waypoints[i].position[0];
        point.y = this->global_waypoints[i].position[1];
        point.z = 0.0;

        color.r = (float)(this->target_global_waypoint_idx != i);
        color.g = (float)(this->target_global_waypoint_idx == i);
        color.b = 0.0;
        color.a = 1.0;

        waypoints_msg.points.push_back(point);
        waypoints_msg.colors.push_back(color);
    }

    this->waypoints_visualization_publisher_->publish(waypoints_msg);
}