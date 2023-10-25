#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

std::vector<std::vector<double>> load_waypoints(std::string filename) {

    std::vector<std::vector<double>> waypoints;
    
    std::fstream fin;
    fin.open(filename, ios::in);

    std::string line, value;

    while (!fin.eof()) {
        std::getline(fin, line);
        std::stringstream s(line);
        
        std::vector<double> row;
        while (std::getline(s, value, ',')) {
            row.push_back(std::stod(value));
        }

        waypoints.push_back(row);
    }
    return waypoints;
}

class PurePursuit : public rclcpp::Node
{

private:

    std::string waypoints_filename;

    std::vector<std::vector<double>> waypoints; //in map frame
    tf2::Matrix3x3 closest_waypoint;

    double L; //lookahead distance
    int current_index = 0; //current waypoint index

    std::string pose_topic = "/odom";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        waypoints = load_waypoints(waypoints_filename);
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 1, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1));
    }
    
    tf2::Matrix3x3 update_closest_waypoint(float x, float y) { //return the index of the closest waypoint to the car
        double prev_dist = std::numeric_limits<double>::infinity();

        for (auto waypoint = begin(this->waypoints)+this->current_index; waypoint != end(this->waypoints); ++waypoint) {
            double dist = sqrt(pow((x - waypoint->at(0)), 2) + pow((y - waypoint->at(1)), 2));
            if (dist < prev_dist) {
                prev_dist = dist;
                current_index += 1;
            } else {
                return tf2::Matrix3x3(1, 0, waypoint->at(0), 0, 1, waypoint->at(1), 0, 0, 1);
            }
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    {
        // TODO: find the current waypoint to track using methods mentioned in lecture
        this->closest_waypoint = this->update_closest_waypoint(pose_msg->pose.position.x, pose_msg->pose.position.y);

        tf2::Quaternion q(pose_msg->pose.orientation.x,
                            pose_msg->pose.orientation.y,
                            pose_msg->pose.orientation.z,
                            pose_msg->pose.orientation.w);
        tf2::Matrix3x3 R(q);

        this->closest_waypoint = R * this->closest_waypoint; //Transform into vehicle frame

        tf2::Vector3 waypoint = this->closest_waypoint[2];

        double x = this->closest_waypoint[2][0];
        double y = this->closest_waypoint[2][1];

        double L_2 = (pow(x, 2) + pow(y, 2));

        double gamma = 2*abs(y)/L_2;

        double steering_angle = gamma * sqrt(L_2);

        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.steering_angle = steering_angle;
        message.drive.speed = 1.0;
        this->drive_publisher_->publish(message);
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}