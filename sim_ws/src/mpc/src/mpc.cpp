#include "mpc/mpc.h"

// Destructor of the RRT class
MPC::~MPC() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("MPC"), "%s\n", "MPC shutting down");
}

// Constructor of the RRT class
MPC::MPC(): rclcpp::Node("mpc_node") {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};           
    param_desc.description = "MPC Parameters";

    // this->initialize_parameters();

    // load_waypoints(this->waypoints_filename);

    // // ROS publishers
    // this->drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 1);

    // ROS subscribers


    //Transforms


    //Timers


    RCLCPP_INFO(rclcpp::get_logger("MPC"), "%s\n", "MPC Initialized.");
}

/**
 * @brief Initialize all parameters for MPC node. Parameters should be input into namespace in header file and appropriately inserted
 *          into the function below.
 * 
 */
void MPC::initialize_parameters() {
    //Initialize string parameters
    this->initialize_parameter(DRIVE_TOPIC, this->drive_topic);
    this->initialize_parameter(POSE_TOPIC, this->pose_topic);
    this->initialize_parameter(MPC_INPUT_TRAJ_VISUALIZATION_TOPIC, this->mpc_input_traj_visualization_topic);
    this->initialize_parameter(MPC_SOLVE_TRAJ_VISUALIZATION_TOPIC, this->mpc_solve_traj_visualization_topic);

    //Initialize int parameters
    this->initialize_parameter(WAYPOINT_MOD, this->waypoint_mod);

    //Initializze double parameters
    
}

/**
 * @brief Initialize ros2 string parameter
 * 
 * @param parameter_name 
 * @param parameter 
 */
void MPC::initialize_parameter(std::string parameter_name, std::string &parameter) {
    this->declare_parameter(parameter_name, "");
    parameter = this->get_parameter(parameter_name).as_string();
    RCLCPP_INFO(this->get_logger(), "%s: %s", parameter_name.c_str(), parameter.c_str());
    return;
}

/**
 * @brief Initialize ros2 int parameter
 * 
 * @param parameter_name 
 * @param parameter 
 */
void MPC::initialize_parameter(std::string parameter_name, int &parameter) {
    this->declare_parameter(parameter_name, 0);
    parameter = this->get_parameter(parameter_name).as_int();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", parameter);
    return;
}

/**
 * @brief Initialize ros2 double parameter
 * 
 * @param parameter_name 
 * @param parameter 
 */
void MPC::initialize_parameter(std::string parameter_name, double &parameter) {
    this->declare_parameter(parameter_name, 0.0);
    parameter = this->get_parameter(parameter_name).as_double();
    RCLCPP_INFO(this->get_logger(), parameter_name + ": %f", parameter);
    return;
}

void MPC::load_waypoints(std::string filename) {
    RCLCPP_INFO(this->get_logger(), "Loading Waypoints");
    RCLCPP_INFO(this->get_logger(), "Filename: %s", filename.c_str());
    
    std::fstream fin;
    fin.open(filename, std::ios::in);

    std::string line, value;

    int idx = 0;

    while (!fin.eof()) {
        std::getline(fin, line);
        std::stringstream s(line);
        
        if (idx % this->waypoint_mod == 0) {
            std::vector<double> row;
            while (std::getline(s, value, ',')) {
                row.push_back(std::stod(value));
            }
            this->mpc_input_traj.push_back(State(row[0], row[1], row[2], row[3]));
        }
        idx++;
    }
    RCLCPP_INFO(this->get_logger(), "Waypoints Successfully Loaded");
}

/**
 * @brief Update current state of the robot with teh pose message converting quaternion angles to yaw.
 * 
 * @param pose_msg 
 */
void MPC::pose_callback(const nav_msgs::msg::Odometry::SharedPtr &pose_msg) {

    this->position = pose_msg->pose.pose.position;
    tf2::convert(pose_msg->pose.pose.orientation, this->orientation);
    this->velocity = pose_msg->twist.twist.linear;

}

/**
 * @brief Solve mpc for next steering action and send out in the form of AckermannDriveStamped message
 * 
 */
void MPC::mpc_callback() {
    //Pull current vehicle state
    double roll, pitch, yaw;
    tf2::Matrix3x3 t(this->orientation);
    t.getRPY(roll, pitch, yaw);

    State vehicle_state(this->position, yaw, this->velocity);

    //calculate reference trajectory
    this->calculate_reference_trajectory(vehicle_state, this->mpc_input_traj);

    //solve mpc

    //convert output to drive_msg

    //publish drive msg
}

void MPC::mpc_prob_init() {
    auto mpc_prob = casadi::Opti();

    auto xk = mpc_prob.variable(this->NXK, this->TK + 1);
    auto uk = mpc_prob.variable(this->NU, this->TK);

    int Q_block_size = (this->NXK * (this->TK + 1));
    int R_block_size = (this->NXK * this->TK);
    int Rd_block_size = (this->NXK * (this->TK - 1));

    // Eigen::VectorXd Q_diagonal;
    // Q_diagonal.resize(Q_block_size);
    // for (int i = 0; i < this->NXK; i++) {
    //     for (int j = 0; j < this->TK+1; j++) {
    //         Q_diagonal(i + j*(this->TK+1)) = this->Qk[i];
    //     }
    // }
    // Q_block.diagonal() = Q_diagonal;

    // Eigen::VectorXd R_diagonal;
    // R_diagonal.resize(Q_block_size);
    // for (int i = 0; i < this->NXK; i++) {
    //     for (int j = 0; j < this->TK+1; j++) {
    //         R_diagonal(i + j*(this->TK+1)) = this->Qk[i];
    //     }
    // }
    // R_block.diagonal() = R_diagonal;

    std::vector<std::vector<double>> Q_block_vec;
    Q_block_vec.resize(Q_block_size, std::vector<double>(Q_block_size));
    for (int i = 0; i < this->NXK; i++) {
        for (int j = 0; j < this->TK; i++) {
            int index = i + j*(this->TK+1);
            Q_block_vec[index][index] = this->Qk[i];
        }
        Q_block_vec[i + (this->TK+1)*(this->TK+1)][i + (this->TK+1)*(this->TK+1)] = this->Qf[i];
    }
    casadi::Matrix<double> Q_block(Q_block_vec);

    std::vector<std::vector<double>> R_block_vec;
    R_block_vec.resize(R_block_size, std::vector<double>(R_block_size));
    for (int i = 0; i < this->NXK; i++) {
        for (int j = 0; j < this->TK; i++) {
            int index = i + j*(this->TK+1);
            R_block_vec[index][index] = this->Qk[i];
        }
        R_block_vec[i + (this->TK+1)*(this->TK+1)][i + (this->TK+1)*(this->TK+1)] = this->Qf[i];
    }
    casadi::Matrix<double> R_block(R_block_vec);

    std::vector<std::vector<double>> Rd_block_vec;
    Rd_block_vec.resize(Rd_block_size, std::vector<double>(Rd_block_size));
    for (int i = 0; i < this->NXK; i++) {
        for (int j = 0; j < this->TK; i++) {
            int index = i + j*(this->TK+1);
            Rd_block_vec[index][index] = this->Qk[i];
        }
        Rd_block_vec[i + (this->TK+1)*(this->TK+1)][i + (this->TK+1)*(this->TK+1)] = this->Qf[i];
    }
    casadi::Matrix<double> Rd_block(R_block_vec);

    
    auto xk_reshape = reshape(xk, Q_block_size, 1);
    auto uk_reshape = reshape(uk, R_block_size, 1);

    casadi::Slice first(0, this->TK-1, 1);
    casadi::Slice second(1, this->TK, 1);
    auto slew_rate = uk_reshape(second,0) - uk_reshape(first,0);
    auto slew_rate_reshape = reshape(slew_rate, Rd_block_size, 1);

    auto objective = xk_reshape * Q_block * xk + uk_reshape * R_block * uk + slew_rate_reshape * Rd_block * slew_rate;

    mpc_prob.minimize(objective);

    //Add constraints

    casadi::Matrix<double> A, B, C;
    this->get_model_matrix(0.0, 0.0, 0.0, A, B, C);

    std::vector<casadi::Matrix<double>> A_block_vec(this->TK);
    std::vector<casadi::Matrix<double>> B_block_vec(this->TK);
    std::vector<casadi::Matrix<double>> C_block_vec(this->TK);

    for (int i = 0; i < this->TK; i++) {
        A_block_vec.emplace_back(A);
        B_block_vec.emplace_back(B);
        C_block_vec.emplace_back(C);
    }
    
    casadi::Matrix<double> A_block(A_block_vec);
    casadi::Matrix<double> B_block(B_block_vec);
    casadi::Matrix<double> C_block(C_block_vec);

    //Dynamics Constraints
    

    //Control Constraints
    this->addControlConstraints(mpc_prob, uk);

    //State Constraints
    // constraints += [xk <= self.config.MAX_SPEED for xk in self.xk[2,:].flatten()]
    // constraints += [xk >= self.config.MIN_SPEED for xk in self.xk[2,:].flatten()]
    // constraints += [(xk1 - xk)/self.config.DTK <= self.config.MAX_ACCEL for xk, xk1 in zip(self.xk[2,:-1].flatten(), self.xk[2,1:].flatten())]
    // constraints += [uk >= self.config.MIN_STEER for uk in self.uk[0,:].flatten()]
    // constraints += [uk <= self.config.MAX_STEER for uk in self.uk[0,:].flatten()]
    // constraints += [xk0 == x0k for xk0, x0k in zip(self.xk[:,0], self.x0k)]

    mpc_prob.subject_to();


}

void MPC::addControlConstraints(casadi::Opti& mpc_prob, casadi::MX& uk) {
    casadi::Slice slice_0(0, this->TK, 1);
    casadi::Slice slice_1_0(0, this->TK-1, 1);
    casadi::Slice slice_1_1(1, this->TK, 1);

    mpc_prob.subject_to(uk(0, slice_0) <= this->MAX_ACCEL);
    mpc_prob.subject_to((uk(1, slice_1_1) - uk(1, slice_1_0)) <= (this->MAX_ACCEL * this->DTK));
}

void MPC::addStateConstraints(casadi::Opti& mpc_prob, casadi::MX& xk, casadi::MX& uk) {
    casadi::Slice slice_0(0, this->TK, 1);
    casadi::Slice slice_1_0(0, this->TK-1, 1);
    casadi::Slice slice_1_1(1, this->TK, 1);

    mpc_prob.subject_to(uk(0, slice_0) <= this->MAX_ACCEL);
    mpc_prob.subject_to((uk(1, slice_1_1) - uk(1, slice_1_0)) <= (this->MAX_ACCEL * this->DTK));
}

/**
 * @brief Return the A, B, and C matrices for the dynamics of the car.
 * 
 * @param v 
 * @param phi 
 * @param delta 
 * @param A 
 * @param B 
 * @param C 
 */
void MPC::get_model_matrix(double v, double phi, double delta, 
                                                    casadi::Matrix<double>& A, 
                                                    casadi::Matrix<double>& B, 
                                                    casadi::Matrix<double>& C) {    

    std::vector<casadi_int> A_colind = {0, 1, 2, 3, 2, 3, 2, 3, 2};
    std::vector<casadi_int> A_rowind = {0, 1, 2, 3, 0, 0, 1, 1, 3};
    casadi::Sparsity A_sparse(this->NXK, this->NXK, A_colind, A_rowind);
    casadi::Matrix<double> A_(A_sparse);

    A_(0, 0) = 1.0;
    A_(1, 1) = 1.0;
    A_(2, 2) = 1.0;
    A_(3, 3) = 1.0;
    A_(0, 2) = this->DTK * cos(phi);
    A_(0, 3) = -this->DTK * v * sin(phi);
    A_(1, 2) = this->DTK * sin(phi);
    A_(1, 3) = this->DTK * v * cos(phi);
    A_(3, 2) = this->DTK * tan(delta) / this->WB;


    std::vector<casadi_int> B_colind = {0, 1};
    std::vector<casadi_int> B_rowind = {2, 3};
    casadi::Sparsity B_sparse(this->NXK, this->NU, B_colind, B_rowind);
    casadi::Matrix<double> B_(B_sparse);

    B_(2, 0) = this->DTK;
    B_(3, 1) = this->DTK * v / (this->WB * pow(cos(delta), 2));

    casadi::Sparsity B_sparse(this->NXK, this->NU, B_colind, B_rowind);
    casadi::Matrix<double> C_(1, this->NXK);

    C_(0, 0) = this->DTK * v * sin(phi) * phi;
    C_(0, 1) = -this->DTK * v * cos(phi) * phi;
    C_(0, 3) = -this->DTK * v * delta / (this->WB * pow(cos(delta), 2));

    A = A_;
    B = B_;
    C = C_;
}

/**
 * @brief Calculate the new reference trajectory for mpc for the next time horizon given the state of the car
 * 
 * @param state Current State of the vehicle
 * @param reference_trajectory Reference trajectory to calculate nearest point to
 */
std::vector<State> MPC::calculate_reference_trajectory(const State &state, const std::vector<State> &trajectory) {

    int nearest_idx = nearest_point(state, trajectory);

    std::vector<State>::const_iterator start = trajectory.begin() + nearest_idx;
    std::vector<State>::const_iterator end = trajectory.begin() + nearest_idx + this->TK + 1;

    std::vector<State> reference_trajectory(start, end);

    return reference_trajectory;
}