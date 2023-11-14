#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/msg/joy.hpp"

#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <memory>
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

namespace {
    std::string POSE_TOPIC = "pose_topic";
    std::string TARGET_WAYPOINT_TOPIC = "target_waypoint_topic";
    std::string DRIVE_TOPIC = "drive_topic";
    std::string WAYPOINTS_FILENAME = "waypoints_filename";
    std::string LOOKAHEAD = "lookahead";
    std::string SPEED_GAIN = "speed_gain";
}

class PurePursuit : public rclcpp::Node
{

public:
    std::string pose_topic;
    std::string drive_topic;
    std::string target_waypoint_topic;
    std::string map_frame_id = "map";
    std::string car_frame_id = "car";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<std::vector<double>> waypoints; //in map frame
    std::vector<double> target_waypoint;

    PurePursuit() : Node("pure_pursuit_node")
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
        param_desc.description = "Pure Pursuit Parameters";

        this->declare_parameter(POSE_TOPIC, "");
        this->declare_parameter(TARGET_WAYPOINT_TOPIC, "");
        this->declare_parameter(DRIVE_TOPIC, "");
        this->declare_parameter(WAYPOINTS_FILENAME, "");

        this->declare_parameter(LOOKAHEAD, 0.0);
        this->declare_parameter(SPEED_GAIN, 0.0);

        // this->declare_parameter(DEADMAN_SWITCH_AXIS, 0);
        // this->declare_parameter(DEADMAN_SWITCH_THRESHOLD, 0.0);

        pose_topic = this->get_parameter(POSE_TOPIC).as_string();
        target_waypoint_topic = this->get_parameter(TARGET_WAYPOINT_TOPIC).as_string();
        drive_topic = this->get_parameter(DRIVE_TOPIC).as_string();
        waypoints_filename = this->get_parameter(WAYPOINTS_FILENAME).as_string();
        L = this->get_parameter(LOOKAHEAD).as_double();
        speed_gain = this->get_parameter(SPEED_GAIN).as_double();
        // deadman_switch_axis = this->get_parameter(DEADMAN_SWITCH_AXIS).as_int();
        // deadman_switch_threshold = this->get_parameter(DEADMAN_SWITCH_THRESHOLD).as_double();

        RCLCPP_INFO(this->get_logger(), POSE_TOPIC + ": " + pose_topic);
        RCLCPP_INFO(this->get_logger(), TARGET_WAYPOINT_TOPIC + ": " + target_waypoint_topic);
        RCLCPP_INFO(this->get_logger(), DRIVE_TOPIC + ": " + drive_topic);
        RCLCPP_INFO(this->get_logger(), WAYPOINTS_FILENAME + ": " + waypoints_filename);
        RCLCPP_INFO(this->get_logger(), LOOKAHEAD + ": %f", L);
        RCLCPP_INFO(this->get_logger(), SPEED_GAIN + ": %f", speed_gain);

        load_waypoints(waypoints_filename);
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
        target_waypoint_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(target_waypoint_topic, 1);
        pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Call on_timer function every second
        timer_ = this->create_wall_timer(1ms, std::bind(&PurePursuit::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node Initialized");
    }

private:

    std::string waypoints_filename;

    double L; //lookahead distance
    double speed_gain;
    int target_waypoint_index = 0; //current waypoint index

    void load_waypoints(std::string filename) {
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

            this->waypoints.push_back(row);
        }

        this->target_waypoint = this->waypoints[0];
        RCLCPP_INFO(this->get_logger(), "Waypoints Successfully Loaded");
    }

    void publish_target_waypoint(double x_goal, double y_goal, std::vector<float> color, std::string frame_id) {
        auto marker = visualization_msgs::msg::Marker();

        marker.type = 2;
        marker.action = 0;
        marker.header.frame_id = frame_id;

        marker.ns = "visualized_waypoints";

        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];

        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.pose.position.x = x_goal;
        marker.pose.position.y = y_goal;

        marker.lifetime.sec = 0.1;
        this->target_waypoint_publisher_->publish(marker);
    }

    std::vector<double> update_target_waypoint(double x, double y) { //return the index of the closest waypoint to the car

        for (auto i = this->target_waypoint_index; i < waypoints.size(); i++) {

            double dist = sqrt(pow((x - this->waypoints[i][0]), 2) + pow((y - this->waypoints[i][1]), 2)); //Calculate distance to point

            if (dist < L) { //If point is too close --> continue to next point
                if (i >= waypoints.size()-5) {
                    RCLCPP_INFO(this->get_logger(), "Restart");
                    this->target_waypoint_index = 0;
                    this->target_waypoint = this->waypoints[0];
                    break;
                } else {
                    this->target_waypoint_index = i;
                    this->target_waypoint = this->waypoints[i];
                }
            } else if (dist >= L) { //If point is too far --> interpolate and return point
                double theta = atan2((this->waypoints[i][1] - this->target_waypoint[1]), (this->waypoints[i][0] - this->target_waypoint[0]));
                double x_goal = this->target_waypoint[0] + L * cos(theta);
                double y_goal = this->target_waypoint[1] + L * sin(theta);
                return {x_goal, y_goal, 0.0};
            }
        }
        return {0.0, 0.0, 0.0};
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {

        this->target_waypoint = this->update_target_waypoint(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);

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

    void timer_callback() {

        std::vector<float> color = {0.0, 1.0, 0.0, 1.0};
        this->publish_target_waypoint(this->target_waypoint[0], this->target_waypoint[1], color, map_frame_id);

        geometry_msgs::msg::TransformStamped tf_result;

        try {
            tf_result = tf_buffer_->lookupTransform(map_frame_id, car_frame_id, tf2::TimePointZero);
        }catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", car_frame_id.c_str(), map_frame_id.c_str(), ex.what());
            return;
        }

        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w
        );

        tf2::Vector3 p(
            tf_result.transform.translation.x,
            tf_result.transform.translation.y,
            tf_result.transform.translation.z
        );

        tf2::Transform transform(q, p);
        tf2::Vector3 point_in_parent_coordinates(target_waypoint[0], target_waypoint[1], 0.0);
        tf2::Vector3 point_in_child_coordinates = transform.inverse() * point_in_parent_coordinates;

        double x = point_in_child_coordinates[0];
        double y = point_in_child_coordinates[1];

        double gamma = 2*abs(y)/pow(L,2);

        double steering_angle = atan2(gamma * L, 1.0) * ((y >= 0) - (y < 0));

        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.steering_angle = steering_angle;
        message.drive.speed = std::min(speed_gain / sqrt(abs(steering_angle)), 3.0);
        this->drive_publisher_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}