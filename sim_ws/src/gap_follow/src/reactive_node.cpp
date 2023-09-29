#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <string>
#include <vector>
#include <cmath>
#include <exception>
#include <algorithm>

#define IS_INVALID(scan_range, scan_angle, thresh) (scan_range * cos(scan_angle) < thresh)

struct Gap {
    int start_i = 0;
    int end_i = 0;
    int max_i = 0;
    double max_range = 0.0;
    int length = 0;

    Gap(int i){
        start_i = i;
    }
};

namespace {
    std::string SCAN_CUTOFF_RANGE = "scan_cutoff_range";

    std::string REJECT_THRESHOLD = "reject_threshold";
    std::string DISPARITY_THRESHOLD = "disparity_threshold";
    std::string GAP_THRESHOLD = "gap_threshold";
    std::string MEAN_WINDOW = "mean_window";
    std::string DISPARITY_BUBBLE = "disparity_bubble";

    std::string SCAN_BEAMS = "scan_beams";
    std::string SCAN_FOV = "scan_fov";

    std::string VELOCITY_GAIN = "velocity_gain";
}

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
        param_desc.description = "Gap Follow Parameters";

        this->declare_parameter(SCAN_CUTOFF_RANGE, 0.0);
        this->declare_parameter(REJECT_THRESHOLD, 0.0);
        this->declare_parameter(DISPARITY_THRESHOLD, 0.0);
        this->declare_parameter(GAP_THRESHOLD, 0.0);
        this->declare_parameter(MEAN_WINDOW, 0);
        this->declare_parameter(DISPARITY_BUBBLE, 0);
        this->declare_parameter(SCAN_BEAMS, 0);
        this->declare_parameter(SCAN_FOV, 0.0);
        this->declare_parameter(VELOCITY_GAIN, 0.0);

        scan_cutoff_range = this->get_parameter(SCAN_CUTOFF_RANGE).as_double();
        reject_threshold = this->get_parameter(REJECT_THRESHOLD).as_double();
        disparity_threshold = this->get_parameter(DISPARITY_THRESHOLD).as_double();
        gap_threshold = this->get_parameter(GAP_THRESHOLD).as_double();
        mean_window = this->get_parameter(MEAN_WINDOW).as_int();
        disparity_bubble = this->get_parameter(DISPARITY_BUBBLE).as_int();
        scan_beams = this->get_parameter(SCAN_BEAMS).as_int();
        scan_fov = this->get_parameter(SCAN_FOV).as_double();
        velocity_gain = this->get_parameter(VELOCITY_GAIN).as_double();
        scan_angle_min = -scan_fov / 2;
        scan_angle_min_adjusted = -scan_cutoff_range/2;
        scan_angle_increment = scan_fov / scan_beams;
        start_i = (int)((-scan_cutoff_range/2 - scan_angle_min)/scan_angle_increment);
        end_i = (int)((scan_cutoff_range/2 - scan_angle_min)/scan_angle_increment);

        RCLCPP_INFO(this->get_logger(), SCAN_CUTOFF_RANGE + ": %f", scan_cutoff_range);
        RCLCPP_INFO(this->get_logger(), REJECT_THRESHOLD + ": %f", reject_threshold);
        RCLCPP_INFO(this->get_logger(), DISPARITY_THRESHOLD + ": %f", disparity_threshold);
        RCLCPP_INFO(this->get_logger(), GAP_THRESHOLD + ": %f", gap_threshold);
        RCLCPP_INFO(this->get_logger(), MEAN_WINDOW + ": %i", mean_window);
        RCLCPP_INFO(this->get_logger(), DISPARITY_BUBBLE + ": %i", disparity_bubble);
        RCLCPP_INFO(this->get_logger(), SCAN_BEAMS + ": %i", scan_beams);
        RCLCPP_INFO(this->get_logger(), VELOCITY_GAIN + ": %i", velocity_gain);

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 1);

        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    double scan_cutoff_range;

    double reject_threshold = 0.0;
    double disparity_threshold = 0.0;
    double gap_threshold = 0.0;
    int mean_window = 0; //Number of data points to average over on each side of current data point
    int disparity_bubble = 0;

    int scan_beams = 0;
    double scan_fov = 0.0;
    double scan_angle_min = 0.0;
    double scan_angle_min_adjusted = 0.0;
    double scan_angle_increment = 0.0;
    int start_i = 0;
    int end_i = 0;
    int max_gap_start = 0;
    int max_gap_end = 0;

    double velocity_gain = 0.0;
    double car_width = 0.25;

    Gap *max_gap;

    std::vector<float> processed_ranges;

    void preprocess_lidar(const std::vector<float> ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        std::vector<float> temp((end_i - start_i), 0.0f);

        double mean_range = 0;
        for (int i = 0; i < mean_window; i++) {
            mean_range += ranges[i] / mean_window;
        }
        
        for(int i = start_i; i < end_i; i++) {
            if ((i < mean_window)) {
                mean_range = ((mean_range * (mean_window + (i-1))) + ranges[i+mean_window]) / (mean_window + i);
            } else if (i > (ranges.size() - mean_window)) {
                mean_range = (mean_range * (mean_window + (ranges.size()-i)) - ranges[i-mean_window]) / (mean_window + (ranges.size()-(i+1)));
            } else {
                mean_range += ranges[i+mean_window]/(mean_window*2); //Add new range data to mean
                mean_range -= ranges[i-mean_window]/(mean_window*2); //Delete out of scope range data
            }
            temp[i-start_i] = (std::min(mean_range, reject_threshold));
            RCLCPP_INFO(this->get_logger(), "Actual: %f\tAveraged: %f", ranges[i], temp[i-start_i]);
        }

        processed_ranges = {temp.begin(), temp.end()};

        int i = disparity_bubble;

        while (i < processed_ranges.size()) {
            // RCLCPP_INFO(this->get_logger(), "Range: %f\tDifference: %f\tDisparity Thresh: %f", processed_ranges[i], (processed_ranges[i] - processed_ranges[i-1]), disparity_threshold);
            bool DISPARITY = abs(processed_ranges[i] - processed_ranges[i-1]) > disparity_threshold;
            if (DISPARITY) {
                // RCLCPP_INFO(this->get_logger(), "Disparity found: %i", i);
                for (int j = i-disparity_bubble; j < i+disparity_bubble; j++) {
                    try {
                        processed_ranges.at(j) = 0.0;
                    } catch (std::out_of_range) {
                        return;
                    }
                }
                i = i + disparity_bubble;
            } else {
                i++;
            }
        }

        return;
    }

    void find_max_gap()
    {   
        // Return the start index & end index of the max gap in free_space_ranges

        bool IN_GAP = false;
        int gap_start_i = 0;
        max_gap_start = gap_start_i;
        max_gap_end = end_i - start_i;
        int num_gaps = 0;

        int max_gap_length = 0;

        for (size_t i = 0; i < processed_ranges.size(); i++) {
            if (!IN_GAP && (processed_ranges[i] > gap_threshold)) {
                IN_GAP = true;
                gap_start_i = i;
            } else if (IN_GAP && (processed_ranges[i] < gap_threshold) || i == processed_ranges.size()-1) {
                IN_GAP = false;
                num_gaps++;
                bool UPDATE = (i - start_i) > max_gap_length;
                max_gap_length = (UPDATE) * (i - gap_start_i) + (!UPDATE) * max_gap_length;
                max_gap_length = (UPDATE) * (gap_start_i) + (!UPDATE) * max_gap_start;
                max_gap_length = (UPDATE) * (i) + (!UPDATE) * max_gap_end;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Num Gaps: %i\tMax Gap Length: %i\tMax Gap Start: %i\tMax Gap End: %i", num_gaps, max_gap_length, max_gap_start, max_gap_end);
        
        return;
    }

    double find_best_point()
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        //Choose point that's furthest away but isn't going to result in a collision
        // int idx = 0;
        // RCLCPP_INFO(this->get_logger(), "Finding best point");
        // while (true) {
        //     if (!IS_INVALID(processed_ranges[max_gap->max_i - idx], scan_angle_min_adjusted + (max_gap->max_i - idx)*scan_angle_increment, car_width/2)) return scan_angle_min_adjusted + (max_gap->max_i - idx)*scan_angle_increment;
        //     if (!IS_INVALID(processed_ranges[max_gap->max_i + idx], scan_angle_min_adjusted + (max_gap->max_i + idx)*scan_angle_increment, car_width/2)) return scan_angle_min_adjusted + (max_gap->max_i + idx)*scan_angle_increment;
        //     idx++;
        // }
        // RCLCPP_INFO(this->get_logger(), "Best point found");

        return scan_angle_min_adjusted + ((int)((max_gap_end + max_gap_start)/2))*scan_angle_increment;

        // float max_range = 0.0;
        // float max_range_angle = 0.0;
        // int idx = 0;

        // for (float range : processed_ranges) {
        //     bool UPDATE = max_range < range;
        //     max_range = (UPDATE) * range + (!UPDATE) * max_range;
        //     max_range_angle = (UPDATE) * (scan_angle_min_adjusted + idx*scan_angle_increment) + (!UPDATE) * max_range_angle;
        //     idx++;
        // }
        // RCLCPP_INFO(this->get_logger(), "Steering Angle: %f", max_range_angle);
        // return max_range_angle;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
        this->preprocess_lidar(scan_msg->ranges);

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 
        this->find_max_gap();

        // Find the best point in the gap 
        double steering_angle = this->find_best_point();

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity_gain / sqrt(abs(-steering_angle));
        drive_publisher_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}