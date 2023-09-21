#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <algorithm>
#include <iterator>
#include <memory>

#define IS_INVALID(a) (isinff(a) | isnanf(a))

namespace {
    std::string KP = "kp";
    std::string KI = "ki";
    std::string KD = "kd";
    std::string INTEGRAL_CAP = "integral_cap";

    std::string LOOKAHEAD = "lookahead";
    std::string VELOCITY_GAIN = "velocity_gain";

    std::string SCAN_BEAMS = "scan_beams";
    std::string SCAN_RANGE = "scan_range";
}

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
        param_desc.description = "Wall Follow Parameters";

        this->declare_parameter(KP, 0.0);
        this->declare_parameter(KI, 0.0);
        this->declare_parameter(KD, 0.0);
        this->declare_parameter(INTEGRAL_CAP, 0.0);

        this->declare_parameter(LOOKAHEAD, 0.0);
        this->declare_parameter(VELOCITY_GAIN, 0.0);

        this->declare_parameter(SCAN_BEAMS, 1080);
        this->declare_parameter(SCAN_RANGE, 4.7);

        kp = this->get_parameter(KP).as_double();
        ki = this->get_parameter(KI).as_double();
        kd = this->get_parameter(KD).as_double();
        integral_cap = this->get_parameter(INTEGRAL_CAP).as_double();

        lookahead = this->get_parameter(LOOKAHEAD).as_double();
        velocity_gain = this->get_parameter(VELOCITY_GAIN).as_double();

        scan_beams = this->get_parameter(SCAN_BEAMS).as_int();
        scan_range = this->get_parameter(SCAN_RANGE).as_double();
        scan_angle_min = -scan_range/2;

        RCLCPP_INFO(this->get_logger(), KP + ": %f", kp);
        RCLCPP_INFO(this->get_logger(), KI + ": %f", ki);
        RCLCPP_INFO(this->get_logger(), KD + ": %f", KD, kd);
        RCLCPP_INFO(this->get_logger(), INTEGRAL_CAP + ": %f", integral_cap);

        RCLCPP_INFO(this->get_logger(), LOOKAHEAD + ": %f", lookahead);
        RCLCPP_INFO(this->get_logger(), VELOCITY_GAIN + ": %f", velocity_gain);

        RCLCPP_INFO(this->get_logger(), SCAN_BEAMS + ": %i", scan_beams);
        RCLCPP_INFO(this->get_logger(), SCAN_RANGE + ": %f", scan_range);
        RCLCPP_INFO(this->get_logger(), "scan_angle_min: %f", scan_angle_min);

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        // timer_ = this->create_wall_timer(1ms, std::bind(&MinimalParam::timer_callback, this));
    }

private:
    // PID CONTROL PARAMS
    double kp;
    double kd;
    double ki;
    
    double velocity_gain;
    double lookahead;
    
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double integral_cap;

    int scan_beams;
    double scan_range;
    double scan_angle_min;
    float scan_angle_increment = 0.0;

    double velocity = 0.0;

    float dt = 0.0;
    double prev_time_sec = 0.0;
    double prev_time_nanosec = 0.0;

    double theta = M_PI/6; //Angle between scans to consider for wall distance calculation

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    // OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    // rclcpp::TimerBase::SharedPtr timer_;

    // rcl_interfaces::msg::SetParametersResult set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {

    //     rcl_interfaces::msg::SetParametersResult result;
    //     result.successful = false;

    //     for (const auto &param:parameters) {
    //         if (param.get_name() == KP) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 kp = param.as_double();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %f", KP, kp);
    //             }
    //         } else if (param.get_name() == KI) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 ki = param.as_double();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %f", KI, ki);
    //             }
    //         } else if (param.get_name() == KD) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 kd = param.as_double();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %f", KD, kd);
    //             }
    //         } else if (param.get_name() == INTEGRAL_CAP) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 integral_cap = param.as_double();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %f", INTEGRAL_CAP, integral_cap);
    //             }
    //         } else if (param.get_name() == LOOKAHEAD) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 lookahead = param.as_double();
    //             }
    //         } else if (param.get_name() == VELOCITY_GAIN) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    //                 result.successful = true;
    //                 velocity_gain = param.as_double();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %f", VELOCITY_GAIN, velocity_gain);
    //             }
    //         } else if (param.get_name() == SCAN_BEAMS) {
    //             if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    //                 result.successful = true;
    //                 scan_beams = param.as_int();
    //                 RCLCPP_INFO(this->get_logger(), "%s: %i", SCAN_BEAMS, scan_beams);
    //             }
    //         }
    //     }
    //     return result;
    // }

    double get_range(const std::vector<float> range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // RCLCPP_INFO(this->get_logger(), "Hello");
        // for (int i = 0; i < scan_beams; i++) {
        //     float check_angle = scan_angle_min + i*scan_angle_increment;
        //     // RCLCPP_INFO(this->get_logger(), "Angle: %f", check_angle);
        //     if (check_angle > angle) {
        //         RCLCPP_INFO(this->get_logger(), "Range: %i", i);
        //         RCLCPP_INFO(this->get_logger(), "Check Range: %i", );
        //         return range_data[i-1];
        //     }
        // }
        // RCLCPP_INFO(this->get_logger(), "Range: %f", range_data[(int)((angle - this->scan_angle_min) / this->scan_angle_increment)]);
        float range = range_data[(int)((angle - scan_angle_min)/scan_angle_increment)];
        int i = 0;
        while (IS_INVALID(range)) {
            angle = angle + i*scan_angle_increment;
            range = range_data[(int)((angle - scan_angle_min)/scan_angle_increment)];
            i++;
        }

        return range;
    }

    double get_error(const std::vector<float> range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */
        // TODO:implement
        static double start_angle = (M_PI / 2);
        double b = get_range(range_data, start_angle);

        double alpha = 0.0;

        int num_measurements = 100;
        // RCLCPP_INFO(this->get_logger(), "Max Angle: %f", (num_measurements*scan_angle_increment));
        for (int i = 0; i < num_measurements; i++) {
            double theta = ((i+50)*scan_angle_increment);
            double a = get_range(range_data, start_angle - theta);
            alpha += atan2((a * cos(theta) - b), (a * cos(theta)));
        }

        double alpha_average = alpha / num_measurements;

        return (dist - (b * cos(alpha_average) + this->lookahead * sin(alpha_average)));
        
        
        // RCLCPP_INFO(this->get_logger(), "\na: %f \tb: %f", a, b);
        

        // RCLCPP_INFO(this->get_logger(), "\nError: %f", (dist - (b * cos(alpha) + this->lookahead * sin(alpha))));
        // return (dist - (b * cos(alpha) + this->lookahead * sin(alpha)));
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error

        Returns:
            None
        */

        this->integral += this->ki * error * (abs(this->integral + this->ki * error) <= this->integral_cap);
        double derivative = this->kd * (error - this->prev_error) / (this->dt);

        float steering_angle = (this->kp * error) + this->integral - derivative;

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = -steering_angle;
        drive_msg.drive.speed = velocity_gain / abs(steering_angle);

        RCLCPP_INFO(this->get_logger(), "Error: %f\tIntergal: %f\tDerivative: %f", error, this->integral, derivative);

        this->drive_publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        this->scan_angle_increment = scan_msg->angle_increment;
        this->dt = (double)(scan_msg->header.stamp.sec - prev_time_sec) + ((double)(scan_msg->header.stamp.nanosec - prev_time_nanosec) * 1e-9);

        // const int length = (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment;

        double error = get_error(scan_msg->ranges, 1.0);

        this->pid_control(error);

        // RCLCPP_INFO(this->get_logger(), "%f", error);

        this->prev_error = error;
        this->prev_time_sec = scan_msg->header.stamp.sec;
        this->prev_time_nanosec = scan_msg->header.stamp.nanosec;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}