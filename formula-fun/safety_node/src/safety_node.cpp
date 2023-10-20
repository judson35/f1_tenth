#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>
#include <memory>

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        RCLCPP_INFO(this->get_logger(), "Initializing safety_node");
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, std::bind(&Safety::scan_callback, this, _1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Safety::drive_callback, this, _1));
    }

private:
    double speed = 0.0;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        int length = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment;
        float TTC = std::numeric_limits<double>::infinity();
        for (int i = 0; i < length; i++) {
            bool INVALID = isinff(scan_msg->ranges[i]) | isnanf(scan_msg->ranges[i]);
            if (!INVALID){
                float angle = scan_msg->angle_min + i*scan_msg->angle_increment;
                //Only consider points within width of car's front field of view (can't travel sideways in no slip model)
                float r_dot = this->speed * cos(angle) * (int)(abs(sin(angle)) * scan_msg->ranges[i] < 0.2032);
                r_dot = (r_dot > 0.0f) ? r_dot : 0.0f;

                float iTTC = scan_msg->ranges[i] / r_dot;
                TTC = (TTC > iTTC) ? iTTC : TTC;
            }
        }

        RCLCPP_INFO(this->get_logger(), "TTC: %f\t Threshold: %f", TTC, this->speed*2.5);

        if (TTC <= this->speed*2.5) { //Use speed as indication for braking threshold
            auto message = ackermann_msgs::msg::AckermannDriveStamped();
            message.drive.speed = 0.0f;
            this->publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Stopping car");
        }

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}