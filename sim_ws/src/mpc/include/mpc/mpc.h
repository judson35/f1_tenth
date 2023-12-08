#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

#include "mpc/mpc_utils.h"

class MPC : public rclcpp::Node
{
    // Implement MPC
    // This is just a template, you are free to implement your own node!

public:
    MPC();
    virtual ~MPC();

private:

    //String Parameters
    std::string drive_topic;
    std::string pose_topic;
    std::string mpc_input_traj_visualization_topic;
    std::string mpc_solve_traj_visualization_topic;

    //Int Parameters
    int waypoint_mod;
    int NXK; //Length of kinematic state vector
    int NU; //Length of input vector
    int TK; //Finite time horizon
    int N_IND_SEARCH = 20;  // Search index number

    //Double Parameters
    double DTK;  // time step [s] kinematic
    double dlk;  // dist step [m] kinematic
    double LENGTH;  // Length of the vehicle [m]
    double WIDTH;  // Width of the vehicle [m]
    double WB;  // Wheelbase [m]
    double MIN_STEER;  // maximum steering angle [rad]
    double MAX_STEER;  // maximum steering angle [rad]
    double MAX_DSTEER;  // maximum steering speed [rad/s]
    double MAX_SPEED; // maximum speed [m/s]
    double MIN_SPEED; // minimum backward speed [m/s]
    double MAX_ACCEL; // maximum acceleration [m/ss]

    std::vector<double> Qk = {100.0, 100.0, 1.0, 1.0};
    std::vector<double> Qf = {100.0, 100.0, 1.0, 1.0};
    std::vector<double> Rk = {1.0, 1.0};

    std::vector<State> mpc_input_traj;

    //Car state parameters
    geometry_msgs::msg::Point position;
    tf2::Quaternion orientation;
    geometry_msgs::msg::Vector3 velocity;

    Eigen::DiagonalMatrix<double, 1> Q_block;
    Eigen::DiagonalMatrix<double, 1> R_block;

    //Initialization/helper functions
    void initialize_parameters();
    void initialize_parameter(std::string parameter_name, std::string &parameter);
    void initialize_parameter(std::string parameter_name, int &parameter);
    void initialize_parameter(std::string parameter_name, double &parameter);
    void load_waypoints(std::string filename);

    //Subscriber callback functions
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr &pose_msg);

    //Timer callback functions
    void mpc_callback();

    void mpc_prob_init();
    void get_model_matrix(double v, double phi, double delta, casadi::Matrix<double>& A, casadi::Matrix<double>& B, casadi::Matrix<double>& C);
    void addDynamicsConstraints(casadi::Opti& mpc_prob, casadi::MX& xk, casadi::MX& uk);
    void addControlConstraints(casadi::Opti& mpc_prob, casadi::MX& uk);
    void addStateConstraints(casadi::Opti& mpc_prob, casadi::MX& xk, casadi::MX& uk);
    std::vector<State> calculate_reference_trajectory(const State &state, const std::vector<State> &trajectory);
};