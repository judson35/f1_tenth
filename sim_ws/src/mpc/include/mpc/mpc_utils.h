#include <string>
#include <cmath>

#define EUCLIDEAN_DISTANCE(x_delt, y_delt) (sqrt(pow(x_delt,2) + pow(y_delt,2)))

namespace {
    //String parameter names
    std::string DRIVE_TOPIC = "drive_topic";
    std::string POSE_TOPIC = "pose_topic";
    std::string MPC_INPUT_TRAJ_VISUALIZATION_TOPIC = "mpc_input_traj_visualization_topic";
    std::string MPC_SOLVE_TRAJ_VISUALIZATION_TOPIC = "mpc_solve_traj_visualization_topic";


    //Int parameter names
    std::string WAYPOINT_MOD = "waypoint_mod";

    //Double parameter names
    std::string DOUBLE_PARAMETER = "double_parameter";
}

struct State {
    double x, y, yaw, v;
    double roll, pitch;
    State(){}
    State(double x, double y, double yaw, double v): x(x), y(y), yaw(yaw), v(v) {}
    State(geometry_msgs::msg::Point &position, double yaw, geometry_msgs::msg::Vector3 &velocity): yaw(yaw) {
        x = position.x;
        y = position.y;

        v = velocity.x;
    }
};

/**
 * @brief Calculate the index of the trajectory point closest to the input state. This is based on a euclidean distance function.
 * 
 * @param state Current state of the car
 * @param trajectory Trajectory to search for the nearest point
 * @return int 
 */
int nearest_point(const State &state, const std::vector<State> &trajectory) {

    double min_dist = 0.0;
    int min_idx = 0;
    for (int i = 0; i < trajectory.size(); i++) {
        double dist = abs(EUCLIDEAN_DISTANCE((trajectory[i].x - state.x), (trajectory[i].y - state.y)));
        bool UPDATE = dist < min_dist;
        min_dist = UPDATE * dist + !UPDATE * min_dist;
        min_idx = UPDATE * i + !UPDATE * min_idx;
    }

    return min_idx;
}