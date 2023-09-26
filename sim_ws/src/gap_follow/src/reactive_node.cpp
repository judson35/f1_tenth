#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include <cmath>

#define IS_INVALID(scan_range, scan_angle, thresh) (scan_range * cos(scan_angle) < thresh)

struct Gap {
    int start_i = 0;
    int end_i = 0;
    int max_i = 0;
    double max_range = 0.0;
    int length = 0;

    Gap(int i){
        start_i = i
    }
};

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers

    double reject_threshold = 0.0;
    int mean_window = 0; //Number of data points to average over on each side of current data point

    double car_width = 0.0;

    Gap max_gap;

    std::vector<float> processed_ranges;

    void preprocess_lidar(const std::vector<float> ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        double mean_range = 0;
        for (int i = 0; i < mean_window; i++) {
            mean_range += ranges[i] / mean_window;
        }

        for(int i = 0; ranges.size(); i++) {
            if ((i < mean_window)) {
                mean_range = ((mean_range * (mean_window + (i-1))) + ranges[i+mean_window]) / (mean_window + i);
            } else if (i > (ranges.size() - mean_window)) {
                mean_range = (mean_range * (mean_window + (ranges.size()-i)) - ranges[i-mean_window]) / (mean_window + (ranges.size()-(i+1)));
            } else {
                mean_range += ranges[i+mean_window]/(mean_window*2); //Add new range data to mean
                mean_range -= ranges[i-mean_window]/(mean_window*2); //Delete out of scope range data
            }
            processed_ranges[i] = min(mean_range, reject_threshold);
        }

        return;
    }

    void find_max_gap(int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges

        Gap max_gap;
        Gap current_gap;
        bool IN_GAP = false;

        for (int i = 1; i < processed_ranges.size(); i++) {
            if ((processed_ranges[i] - processed_ranges[i-1]) > disparity_threshold) {
                IN_GAP = true;
                current_gap.start_i = i;
            } else if ((processed_ranges[i] - processed_ranges[i-1]) < disparity_threshold) {
                IN_GAP = false;
                current_gap.end_i = i;
                bool UPDATE = max_gap.max_range < current_gap.max_range;
                max_gap.max_range = (UPDATE * current_gap.max_range) + (!UPDATE) * (max_gap.max_range);
                max_gap.start_i = (UPDATE * current_gap.start_i) + (!UPDATE) * (max_gap.start_i);
                max_gap.end_i = (UPDATE * current_gap.end_i) + (!UPDATE) * (max_gap.end_i);
                max_gap.max_i = (UPDATE * current_gap.max_i) + (!UPDATE) * (max_gap.max_i);
            } else if (IN_GAP) {
                bool UPDATE_MAX_RANGE = current_gap.max_range < processed_ranges[i];
                current_gap.max_i = (UPDATE_MAX_RANGE * i) + ((!UPDATE_MAX_RANGE) * current_gap.max_i);
                current_gap.max_range = (UPDATE_MAX_RANGE * processed_ranges[i]) + ((!UPDATE_MAX_RANGE) * current_gap.max_range);
            }
        }
        indice[0] = max_gap.start_i;
        indice[1] = max_gap.end_i;
        return;
    }

    void find_best_point(int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there

        //Choose point that's furthest away but isn't going to result in a collision
        int idx = 0;
        bool FLIP = false;
        while (true) {
            if (!IS_INVALID(processed_ranges[max_gap.max_i - idx], scan_angle_min + (max_gap.max_i - idx)*scan_angle_increment, car_width/2)) return;
            if (!IS_INVALID(processed_ranges[max_gap.max_i + idx], scan_angle_min + (max_gap.max_i + idx)*scan_angle_increment, car_width/2)) return;
            idx++;
        }
        
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
        this->preprocess_lidar(scan_msg->scan_ranges);

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 
        this->find_max_gap();

        // Find the best point in the gap 
        this->find_best_point();

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = -steering_angle;
        drive_msg.drive.speed = velocity_gain / sqrt(abs(-steering_angle));
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}