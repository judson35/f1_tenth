#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("car_tf2_frame_publisher")
  {
    // Declare and acquire `turtlename` parameter
    car_name_ = this->declare_parameter<std::string>("car_name", "ego_racecar");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
    // callback function on each message
    std::ostringstream stream;
    stream << "/" << car_name_.c_str() << "/pose";
    std::string pose_topic = stream.str();

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 1, std::bind(&FramePublisher::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = car_name_.c_str();

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.y;
    t.transform.rotation.z = msg->pose.orientation.z;
    t.transform.rotation.w = msg->pose.orientation.w;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string car_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}