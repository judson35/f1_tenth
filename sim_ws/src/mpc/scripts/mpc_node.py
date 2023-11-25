#!/usr/bin/env python3
import math
from dataclasses import dataclass, field

import cvxpy
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Transform, Twist, Vector3
from tf2_geometry_msgs import do_transform_point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PointStamped, Quaternion
from std_msgs.msg import ColorRGBA
from rclpy.node import Node
from scipy.linalg import block_diag
from scipy.sparse import block_diag, csc_matrix, diags
from sensor_msgs.msg import LaserScan
from mpc.utils import nearest_point, getTrajectoryMarkerMessage
from tf_transformations import euler_from_quaternion
from collections.abc import Sequence
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException

@dataclass
class mpc_config:
    NXK: int = 4  # length of kinematic state vector: z = [x, y, v, yaw]
    NU: int = 2  # length of input vector: u = = [steering, acceleration]
    TK: int = 8  # finite time horizon length kinematic

    # ---------------------------------------------------
    # TODO: you may need to tune the following matrices
    Rk: list = field(
        default_factory=lambda: np.diag([0.01, 100.0])
    )  # input cost matrix, penalty for inputs - [accel, steering]
    Rdk: list = field(
        default_factory=lambda: np.diag([0.01, 100.0])
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering]
    Qk: list = field(
        default_factory=lambda: np.diag([13.5, 13.5, 13.0, 5.5])
    )  # state error cost matrix, for the the next (T) prediction time steps [x, y, v, yaw]
    Qfk: list = field(
        default_factory=lambda: np.diag([13.5, 13.5, 13.0, 5.5])
    )  # final state error matrix, penalty  for the final state constraints: [x, y, v, yaw]
    # ---------------------------------------------------

    N_IND_SEARCH: int = 20  # Search index number
    DTK: float = 0.1  # time step [s] kinematic
    dlk: float = 0.03  # dist step [m] kinematic
    LENGTH: float = 0.58  # Length of the vehicle [m]
    WIDTH: float = 0.31  # Width of the vehicle [m]
    WB: float = 0.33  # Wheelbase [m]
    MIN_STEER: float = -np.deg2rad(45.0)  # maximum steering angle [rad]
    MAX_STEER: float = np.deg2rad(45.0)  # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad(270.0)  # maximum steering speed [rad/s]
    MAX_SPEED: float = 6.0  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 3.0  # maximum acceleration [m/ss]


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    v: float = 0.0
    yaw: float = 0.0

def reference_trajectory_init(filename : str) -> np.ndarray:
    waypoints = []
    with open(filename, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if (i % 12 == 0):
                data = line.split(',')
                data = [float(val) for val in data]
                data[2] = data[2]*3
                waypoints.append(data)
    return np.array(waypoints)

class MPC(Node):
    """ 
    Implement Kinematic MPC on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('mpc_node')

        self.drive_topic = "/drive"
        self.pose_topic = "/ego_racecar/odom"
        self.mpc_input_visualization_topic = "/mpc_input_traj"
        self.mpc_visualization_topic = "/mpc_traj"
        self.reference_trajectory_filename = "/home/judson35/f1_tenth/sim_ws/Mpc_Waypoints.txt"

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)
        self.mpc_input_traj_publisher = self.create_publisher(Marker, self.mpc_input_visualization_topic, 1)
        self.mpc_traj_publisher = self.create_publisher(Marker, self.mpc_visualization_topic, 1)
        self.pose_subscriber = self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 1)

        self.waypoints = reference_trajectory_init(self.reference_trajectory_filename)
        # self.waypoints = np.flip(self.waypoints, axis=0)

        self.mpc_traj_timer = self.create_timer(0.5, self.mpc_traj_timer_callback)
        self.dt_mpc = 0.005
        self.mpc_control_timer = self.create_timer(self.dt_mpc, self.mpc_callback)

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        self.config = mpc_config(Qk=np.diag([20, 20, 1, 5]), Qfk=np.diag([30, 30, 1, 10]))
        self.odelta = None
        self.oa = None
        self.init_flag = 0
        self.ref_path = []

        self.position = Point()
        self.orientation = Quaternion()
        self.v = 0
        self.yaw = 0

        self.transform = Transform()

        self.ox = []
        self.oy = []
        self.x_guess = []
        self.y_guess = []

        # initialize MPC problem
        self.mpc_prob_init()

    def pose_callback(self, pose_msg : Odometry):

        # calculate linear velocity
        # calculate yaw

        self.position = pose_msg.pose.pose.position
        self.orientation = pose_msg.pose.pose.orientation
        self.v = pose_msg.twist.twist.linear.x

        _, _, self.yaw = euler_from_quaternion([pose_msg.pose.pose.orientation.x,
                                                pose_msg.pose.pose.orientation.y,
                                                pose_msg.pose.pose.orientation.z,
                                                pose_msg.pose.pose.orientation.w])

        # self.transform = Transform(rotation=pose_msg.pose.pose.orientation, translation=pose_msg.pose.pose.position)

    def mpc_callback(self):

        vehicle_state = State(self.position.x, self.position.y, self.v, self.yaw)

        # TODO: Calculate the next reference trajectory for the next T steps
        #       with current vehicle pose.
        #       ref_x, ref_y, ref_yaw, ref_v are columns of self.waypoints
        self.ref_path = self.calc_ref_trajectory(vehicle_state, self.waypoints[:, 0],
                                                                    self.waypoints[:, 1],
                                                                    self.waypoints[:, 3],
                                                                    self.waypoints[:, 2])
        x0 = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        # print(self.ref_path)

        # TODO: solve the MPC control problem
        (
            self.oa,
            self.odelta,
            self.ox,
            self.oy,
            oyaw,
            ov,
            state_predict,
        ) = self.linear_mpc_control(self.ref_path, x0, self.oa, self.odelta)

        # TODO: publish drive message.
        steer_output = self.odelta[0]
        speed_output = vehicle_state.v + self.oa[0] * self.config.DTK

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steer_output
        drive_msg.drive.speed = speed_output
        self.drive_publisher.publish(drive_msg)

    def mpc_prob_init(self):
        """
        Create MPC quadratic optimization problem using cvxpy, solver: OSQP
        Will be solved every iteration for control.
        More MPC problem information here: https://osqp.org/docs/examples/mpc.html
        More QP example in CVXPY here: https://www.cvxpy.org/examples/basic/quadratic_program.html
        """
        # Initialize and create vectors for the optimization problem
        # Vehicle State Vector
        self.xk = cvxpy.Variable(
            (self.config.NXK, self.config.TK + 1)
        )
        # Control Input vector
        self.uk = cvxpy.Variable(
            (self.config.NU, self.config.TK)
        )
        objective = 0.0  # Objective value of the optimization problem
        constraints = []  # Create constraints array

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter((self.config.NXK,))
        self.x0k.value = np.zeros((self.config.NXK,))

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter((self.config.NXK, self.config.TK + 1))
        self.ref_traj_k.value = np.zeros((self.config.NXK, self.config.TK + 1))

        # Initializes block diagonal form of R = [R, R, ..., R] (NU*T, NU*T)
        R_block = block_diag(tuple([self.config.Rk] * self.config.TK))

        # Initializes block diagonal form of Rd = [Rd, ..., Rd] (NU*(T-1), NU*(T-1))
        Rd_block = block_diag(tuple([self.config.Rdk] * (self.config.TK - 1)))

        # Initializes block diagonal form of Q = [Q, Q, ..., Qf] (NX*T, NX*T)
        Q_block = [self.config.Qk] * (self.config.TK)
        Q_block.append(self.config.Qfk)
        Q_block = block_diag(tuple(Q_block))

        # Formulate and create the finite-horizon optimal control problem (objective function)
        # The FTOCP has the horizon of T timesteps

        # --------------------------------------------------------
        # TODO: fill in the objectives here, you should be using cvxpy.quad_form() somehwhere
        objective += cvxpy.quad_form((self.ref_traj_k.flatten() - self.xk.flatten()), Q_block)      #State objective
        objective += cvxpy.quad_form(self.uk.flatten(), R_block)                          #Control objective
        objective += cvxpy.quad_form((self.uk[:,1:].flatten() - self.uk[:,:-1].flatten()), Rd_block)    #Slew rate objective

        # --------------------------------------------------------

        # Constraints 1: Calculate the future vehicle behavior/states based on the vehicle dynamics model matrices
        # Evaluate vehicle Dynamics for next T timesteps
        A_block = []
        B_block = []
        C_block = []
        # init path to zeros
        path_predict = np.zeros((self.config.NXK, self.config.TK + 1))
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        # [AA] Sparse matrix to CVX parameter for proper stuffing
        # Reference: https://github.com/cvxpy/cvxpy/issues/1159#issuecomment-718925710
        m, n = A_block.shape
        self.Annz_k = cvxpy.Parameter(A_block.nnz)
        data = np.ones(self.Annz_k.size)
        rows = A_block.row * n + A_block.col
        cols = np.arange(self.Annz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Annz_k.size))

        # Setting sparse matrix data
        self.Annz_k.value = A_block.data

        # Now we use this sparse version instead of the old A_ block matrix
        self.Ak_ = cvxpy.reshape(Indexer @ self.Annz_k, (m, n), order="C")

        # Same as A
        m, n = B_block.shape
        self.Bnnz_k = cvxpy.Parameter(B_block.nnz)
        data = np.ones(self.Bnnz_k.size)
        rows = B_block.row * n + B_block.col
        cols = np.arange(self.Bnnz_k.size)
        Indexer = csc_matrix((data, (rows, cols)), shape=(m * n, self.Bnnz_k.size))
        self.Bk_ = cvxpy.reshape(Indexer @ self.Bnnz_k, (m, n), order="C")
        self.Bnnz_k.value = B_block.data

        # No need for sparse matrices for C as most values are parameters
        self.Ck_ = cvxpy.Parameter(C_block.shape)
        self.Ck_.value = C_block

        # -------------------------------------------------------------
        # TODO: Constraint part 1:
        #       Add dynamics constraints to the optimization problem
        #       This constraint should be based on a few variables:
        #       self.xk, self.Ak_, self.Bk_, self.uk, and self.Ck_
        dynamics = self.Ak_ @ self.xk[:,:-1].flatten() + self.Bk_ @ self.uk.flatten() + self.Ck_
        constraints += [xk1 == xk for xk1, xk in zip(self.xk[:,1:].flatten(), dynamics)]
        
        # TODO: Constraint part 2:
        #       Add constraints on steering, change in steering angle
        #       cannot exceed steering angle speed limit. Should be based on:
        #       self.uk, self.config.MAX_DSTEER, self.config.DTK
        constraints += [uk <= self.config.MAX_ACCEL for uk in self.uk[0,:].flatten()]
        constraints += [(uk1 - uk)/self.config.DTK <= self.config.MAX_DSTEER for uk, uk1 in zip(self.uk[1,:-1].flatten(), self.uk[1,1:].flatten())]

        # TODO: Constraint part 3:
        #       Add constraints on upper and lower bounds of states and inputs
        #       and initial state constraint, should be based on:
        #       self.xk, self.x0k, self.config.MAX_SPEED, self.config.MIN_SPEED,
        #       self.uk, self.config.MAX_ACCEL, self.config.MAX_STEER
        constraints += [xk <= self.config.MAX_SPEED for xk in self.xk[2,:].flatten()]
        constraints += [xk >= self.config.MIN_SPEED for xk in self.xk[2,:].flatten()]
        constraints += [(xk1 - xk)/self.config.DTK <= self.config.MAX_ACCEL for xk, xk1 in zip(self.xk[2,:-1].flatten(), self.xk[2,1:].flatten())]
        constraints += [uk >= self.config.MIN_STEER for uk in self.uk[0,:].flatten()]
        constraints += [uk <= self.config.MAX_STEER for uk in self.uk[0,:].flatten()]
        constraints += [xk0 == x0k for xk0, x0k in zip(self.xk[:,0], self.x0k)]
        
        # -------------------------------------------------------------

        # Create the optimization problem in CVXPY and setup the workspace
        # Optimization goal: minimize the objective function
        self.MPC_prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

    def calc_ref_trajectory(self, state: State, cx: list, cy: list, sp: list, cyaw: list):
        """
        calc referent trajectory ref_traj in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param cx: Course X-Position
        :param cy: Course y-Position
        :param sp: speed profile
        :param cyaw: Course Heading
        :dl: distance step
        :pind: Setpoint Index
        :return: reference trajectory ref_traj, reference steering angle
        """

        # Create placeholder Arrays for the reference trajectory for T steps
        ref_traj = np.zeros((self.config.NXK, self.config.TK + 1))
        ncourse = len(cx)

        # Find nearest index/setpoint from where the trajectories are calculated
        _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)

        # Load the initial parameters from the setpoint into the trajectory
        ref_traj[0, 0] = cx[ind]
        ref_traj[1, 0] = cy[ind]
        ref_traj[2, 0] = sp[ind]
        ref_traj[3, 0] = cyaw[ind]

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs(state.v) * self.config.DTK
        dind = travel / self.config.dlk
        ind_list = int(ind) + np.insert(
            np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        ).astype(int)
        ind_list[ind_list >= ncourse] -= ncourse
        ref_traj[0, :] = cx[ind_list]
        ref_traj[1, :] = cy[ind_list]
        ref_traj[2, :] = sp[ind_list]
        cyaw[cyaw - state.yaw > 4.5] = np.abs(
            cyaw[cyaw - state.yaw > 4.5] - (2 * np.pi)
        )
        cyaw[cyaw - state.yaw < -4.5] = np.abs(
            cyaw[cyaw - state.yaw < -4.5] + (2 * np.pi)
        )
        ref_traj[3, :] = cyaw[ind_list]

        ref_traj[0, 1:] = cx[ind:(ind+self.config.TK)]
        ref_traj[1, 1:] = cy[ind:(ind+self.config.TK)]
        ref_traj[2, 1:] = sp[ind:(ind+self.config.TK)]
        ref_traj[3, 1:] = cyaw[ind:(ind+self.config.TK)]

        self.x_guess = self.waypoints[ind:(ind+self.config.TK),0]
        self.y_guess = self.waypoints[ind:(ind+self.config.TK),1]

        return ref_traj

    def predict_motion(self, x0, oa, od, xref):
        path_predict = xref * 0.0
        for i, _ in enumerate(x0):
            path_predict[i, 0] = x0[i]

        state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.config.TK + 1)):
            state = self.update_state(state, ai, di)
            path_predict[0, i] = state.x
            path_predict[1, i] = state.y
            path_predict[2, i] = state.v
            path_predict[3, i] = state.yaw

        return path_predict

    def update_state(self, state: State, a: float, delta: float):

        # input check
        if delta >= self.config.MAX_STEER:
            delta = self.config.MAX_STEER
        elif delta <= -self.config.MAX_STEER:
            delta = -self.config.MAX_STEER

        state.x = state.x + state.v * math.cos(state.yaw) * self.config.DTK
        state.y = state.y + state.v * math.sin(state.yaw) * self.config.DTK
        state.yaw = (
            state.yaw + (state.v / self.config.WB) * math.tan(delta) * self.config.DTK
        )
        state.v = state.v + a * self.config.DTK

        if state.v > self.config.MAX_SPEED:
            state.v = self.config.MAX_SPEED
        elif state.v < self.config.MIN_SPEED:
            state.v = self.config.MIN_SPEED

        return state

    def get_model_matrix(self, v, phi, delta):
        """
        Calc linear and discrete time dynamic model-> Explicit discrete time-invariant
        Linear System: Xdot = Ax +Bu + C
        State vector: x=[x, y, v, yaw]
        :param v: speed
        :param phi: heading angle of the vehicle
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        # State (or system) matrix A, 4x4
        A = np.zeros((self.config.NXK, self.config.NXK))
        A[0, 0] = 1.0
        A[1, 1] = 1.0
        A[2, 2] = 1.0
        A[3, 3] = 1.0
        A[0, 2] = self.config.DTK * math.cos(phi)
        A[0, 3] = -self.config.DTK * v * math.sin(phi)
        A[1, 2] = self.config.DTK * math.sin(phi)
        A[1, 3] = self.config.DTK * v * math.cos(phi)
        A[3, 2] = self.config.DTK * math.tan(delta) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros((self.config.NXK, self.config.NU))
        B[2, 0] = self.config.DTK
        B[3, 1] = self.config.DTK * v / (self.config.WB * math.cos(delta) ** 2)

        C = np.zeros(self.config.NXK)
        C[0] = self.config.DTK * v * math.sin(phi) * phi
        C[1] = -self.config.DTK * v * math.cos(phi) * phi
        C[3] = -self.config.DTK * v * delta / (self.config.WB * math.cos(delta) ** 2)

        return A, B, C

    def mpc_prob_solve(self, ref_traj, path_predict, x0):
        self.x0k.value = x0

        A_block = []
        B_block = []
        C_block = []
        for t in range(self.config.TK):
            A, B, C = self.get_model_matrix(
                path_predict[2, t], path_predict[3, t], 0.0
            )
            A_block.append(A)
            B_block.append(B)
            C_block.extend(C)

        A_block = block_diag(tuple(A_block))
        B_block = block_diag(tuple(B_block))
        C_block = np.array(C_block)

        self.Annz_k.value = A_block.data
        self.Bnnz_k.value = B_block.data
        self.Ck_.value = C_block

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem in CVXPY
        # Solver selections: cvxpy.OSQP; cvxpy.GUROBI
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            ox = np.array(self.xk.value[0, :]).flatten()
            oy = np.array(self.xk.value[1, :]).flatten()
            ov = np.array(self.xk.value[2, :]).flatten()
            oyaw = np.array(self.xk.value[3, :]).flatten()
            oa = np.array(self.uk.value[0, :]).flatten()
            odelta = np.array(self.uk.value[1, :]).flatten()

        else:
            print("Error: Cannot solve mpc..")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

        return oa, odelta, ox, oy, oyaw, ov

    def linear_mpc_control(self, ref_path, x0, oa, od):
        """
        MPC contorl with updating operational point iteraitvely
        :param ref_path: reference trajectory in T steps
        :param x0: initial state vector
        :param oa: acceleration of T steps of last time
        :param od: delta of T steps of last time
        """

        if oa is None or od is None:
            oa = [0.0] * self.config.TK
            od = [0.0] * self.config.TK

        # Call the Motion Prediction function: Predict the vehicle motion for x-steps
        path_predict = self.predict_motion(x0, oa, od, ref_path)
        poa, pod = oa[:], od[:]

        # Run the MPC optimization: Create and solve the optimization problem
        mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v = self.mpc_prob_solve(
            ref_path, path_predict, x0
        )

        return mpc_a, mpc_delta, mpc_x, mpc_y, mpc_yaw, mpc_v, path_predict
    
    def mpc_traj_timer_callback(self):
        # x_ref = self.ref_path[0,:]
        # y_ref = self.ref_path[1,:]
        # x_ref = self.x_guess
        # y_ref = self.y_guess
        x_ref = []
        y_ref = []
        try:
            trans = self.tf_buffer.lookup_transform("ego_racecar/base_link", "map", rclpy.time.Time())
            x_ref = np.add(x_ref, trans.transform.translation.x)
            y_ref = np.add(x_ref, trans.transform.translation.y)
            for x,y in zip(x_ref, y_ref):
                point_in_parent_coords = PointStamped()
                point_in_parent_coords.point.x = x
                point_in_parent_coords.point.y = y
                point = do_transform_point(point_in_parent_coords, trans)
                x_ref.append(point.point.x)
                y_ref.append(point.point.y)
        except LookupException as e:
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))
        # print(x_ref)
        mpc_input_traj = getTrajectoryMarkerMessage("/ego_racecar/base_link", "mpc_input_waypoints", x_ref, y_ref, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        mpc_traj = getTrajectoryMarkerMessage("/map", "mpc_waypoints", self.ox, self.oy, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))

        self.mpc_input_traj_publisher.publish(mpc_input_traj)
        self.mpc_traj_publisher.publish(mpc_traj)

def main(args=None):
    rclpy.init(args=args)
    print("MPC Initialized")
    mpc_node = MPC()
    rclpy.spin(mpc_node)

    mpc_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()