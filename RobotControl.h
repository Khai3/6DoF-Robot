#pragma once

#include <Eigen/Dense>
#include <vector>

class Robot
{
public:
    /* Constructor
     * Input: link lengths of Skyentific Robot (mm)
     * Description: Define the link lengths of Robot. The home postion 'M' and its corresponding screw axes are defined and
     *              calculated here as well
     */   
    Robot(float r1_in=47, float r2_in=110, float r3_in=26, float d1_in = 133, float d4_in=117.5, float d6_in=28);

    /* Input: Vector of joint angles in radians
     * Description: Calculate end effector pose given joint angles using the product of exponentials formula 
     * Output: A 4x4 transformation matrix 
     */
    Eigen::Matrix4d ForwardKinematics(const Eigen::VectorXd &theta);

    /* Input: Transformation matrix of end effector
     * Description: Calculate desired joint angles analytically given an end effector pose
     * Output: An array of joint angle values
     */
    Eigen::VectorXd InverseKinematics(const Eigen::Matrix4d &end_pose);

    /* Input: Vector of current joint angles
     * Description: Calculate the space jacobian given the current joint angles
     * Output: The space jacobian matrix
     */
    Eigen::MatrixXd JacobianSpace(const Eigen::VectorXd &theta);

    /* Input: Vector of current joint angles
     * Description: Calculate the body jacobian given the current joint angles
     * Output: The body jacobian matrix
     */
    Eigen::MatrixXd JacobianBody(const Eigen::VectorXd &theta);

    /* Inputs:
     *   t : The current time t satisfying 0 < t < Tf
     *   Tf: Total time of the motion in seconds from start to end
     * Description: Compute s(t) for a cubic time scaling
     * Output: The path parameter corresponding to a third-order
     *         polynomial motion that begins and ends at zero velocity
     */
    float CubicTimeScaling(float t,float Tf);

    /* Inputs:
     *   t : The current time t satisfying 0 < t < Tf
     *   Tf: Total time of the motion in seconds from start to end
     * Description: Compute derivation of s(t) for a cubic time scaling
     * Output: The path velocity parameter corresponding to a third-order
     *         polynomial motion that begins and ends at zero velocity
     */
    float CubicTimeScalingDot(float t,float Tf);
    
    /* Inputs:
     *  t : The current time t satisfying 0 < t < Tf
     *  Tf: Total time of the motion in seconds from start to end
     * Description: Compute s(t) for a quintic time scaling
     * Output: The path parameter corresponding to a fifth-order
     *          polynomial motion that begins and ends at zero velocity
     *	        and zero acceleration
     */
    float QuinticTimeScaling(float t,float Tf);

        /* Inputs:
     *  t : The current time t satisfying 0 < t < Tf
     *  Tf: Total time of the motion in seconds from start to end
     * Description: Compute derivation of s(t) for a quintic time scaling
     * Output: The path velocity parameter corresponding to a fifth-order
     *          polynomial motion that begins and ends at zero velocity
     *	        and zero acceleration
     */
    float QuinticTimeScalingDot(float t,float Tf);

    /* Inputs: 
     *   thetastart: Initial joint variables
     *   thetaend  : Final joint variables
     *   Tf        : Total time of the motion in seconds from start to end
     *   N         : Number of discrete points in trajectory
     *   method    : The time scaling method (Cubic,Quintic)
     * Description: Compute a straight-line trajectory in joint space
     * Output: A trajectory as an N x 7 matrix. The 7th column is time elasped 
     *         since start. The first row is thetastart + 0 and the Nth row is 
     *         thetaend + T. The elapsed time between each row is Tf / (N - 1)
     */
    std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> JointTrajectory(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend, float Tf, int N, const std::string &method);

    /*
     * Inputs:
     *   Xstart: The initial end-effector configuration
     *   Xend  : The final end-effector configuration
     *   Tf    : Total time of the motion in seconds from start to end
     *	 N     : Number of discrete points in trajectory
     *   method: The time scaling method (Cubic,Quintic)
     * Description: Compute a trajectory as a list of N SE(3) matrices corresponding to
     *			    the screw motion about a space screw axis
     * Outputs: The discretized trajectory as a list of tuples of N matrices in SE(3)
     *          and elapsed time. The first in the list is Xstart
     *          and the Nth is Xend
     */
    std::vector<std::tuple<Eigen::Matrix4d, float>> ScrewTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend, float Tf, int N, const std::string &method);

    /*
     * Inputs:
     *   Xstart: The initial end-effector configuration
     *   Xend  : The final end-effector configuration
     *   Tf    : Total time of the motion in seconds from start to end
     *	 N     : Number of discrete points in trajectory
     *   method: The time scaling method (Cubic,Quintic)
     * Description: Compute a trajectory as a list of N SE(3) matrices corresponding to
     *			    the origin of the end-effector frame following a straight line
     * Outputs: The discretized trajectory as a list of tuples of N matrices in SE(3)
     *          and elapsed time. The first in the list is Xstart
     *          and the Nth is Xend
     */
    std::vector<std::tuple<Eigen::Matrix4d, float>> CartesianTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend, float Tf, int N, const std::string &method);

    
    /* Inputs: 
     *   points: Vector of transformation matrices
     *   Tf    : Total time of the motion in seconds from start to end
     *   N     : Number of discrete points in trajectory between each via point
     * Description: Compute a trajectory using cubic via-point interpolation in task space
     * Output: The discretized trajectory as a list of tuples of N matrices in SE(3)
     *         and elapsed time.
     */
    std::vector<std::tuple<Eigen::Matrix4d, float>> ViaTrajectory(const std::vector<Eigen::Matrix4d> &points, float Tf, int N);

    /* Inputs: 
     *   Kp           : Proportional Gain Value
     *   Ki           : Integral Gain Value
     *   currentAngles: Vector of current joint angles of robot
     *   desiredAngles: Vector of desired joint angles of robot
     *   dt           : Time step
     *   feedforward  : Vector of joint velocities as feedforward control
     * Description: Compute the required joint velocity of the robot using feedback 
     *              and feedforward control
     * Output: The vector of joint velocities for actuation
     */
    Eigen::VectorXd VelocityControl(const float Kp, const float Ki, const Eigen::VectorXd &currentAngles, const Eigen::VectorXd &desiredAngles, const float dt, const Eigen::VectorXd &feedforward = Eigen::VectorXd::Zero(6));

    /* Inputs: 
     *   Kp         : Proportional Gain Value
     *   Ki         : Integral Gain Value
     *   currentX   : Current transformation matrix of robot
     *   desiredX   : Desired transformation matrix of robot
     *   dt         : Time step
     *   feedforward: End effector twist as feedforward control
     * Description: Compute the required end effector twist of the robot using feedback 
     *              and feedforward control
     * Output: The computed end effector twist
     */
    Eigen::VectorXd VelocityEndControl(const float Kp, const float Ki, const Eigen::MatrixXd &currentX, const Eigen::MatrixXd &desiredX, const float dt, const Eigen::VectorXd &feedforward = Eigen::VectorXd::Zero(6));

    /* Inputs: 
     *   angles      : Vector of curent angles
     *   angularVel  : Vector of angular velocities
     *   angularAccel: Vector of angular acceleration
     * Description: Compute the torque required to produce a desired motion 
     * Output: Vector of torques
     */
    Eigen::VectorXd InverseDynamics(const Eigen::VectorXd &angles, const Eigen::VectorXd &angularVel, const Eigen::VectorXd &angularAccel);

    /* Inputs: 
     *   Kp               : Proportional Gain Value
     *   Ki               : Integral Gain Value
     *   Kd               : Derivative Gain Value
     *   currentAngles    : Vector of current joint angles of robot
     *   desiredAngles    : Vector of desired joint angles of robot
     *   currentAngularVel: Vector of current joint anglular velocities of robot
     *   desiredAngularVel: Vector of desired joint anglular velocities of robot
     *   dt               : Time step
     *   feedforward      : Vector of joint accelerations as feedforward control
     * Description: Compute the required joint torques of the robot using feedback 
     *              and feedforward control
     * Output: The vector of joint torques for actuation
     */
    Eigen::VectorXd MotionControl(const float Kp, const float Ki, const float Kd, const Eigen::VectorXd &currentAngles, const Eigen::VectorXd &desiredAngles, const Eigen::VectorXd &currentAngularVel, const Eigen::VectorXd &desiredAngularVel, const float dt, const Eigen::VectorXd &feedforward = Eigen::VectorXd::Zero(6));

private:
    float r1;
    float r2;
    float r3;
    float d1;
    float d4;
    float d6; 
    Eigen::Matrix4d M1;
    Eigen::Matrix4d M2;
    Eigen::Matrix4d M3;
    Eigen::Matrix4d M4;
    Eigen::Matrix4d M5;
    Eigen::Matrix4d M6;

    Eigen::MatrixXd G1{6,6};
    Eigen::MatrixXd G2{6,6};
    Eigen::MatrixXd G3{6,6};
    Eigen::MatrixXd G4{6,6};
    Eigen::MatrixXd G5{6,6};
    Eigen::MatrixXd G6{6,6};

    Eigen::VectorXd S1{6};
    Eigen::VectorXd S2{6};
    Eigen::VectorXd S3{6};
    Eigen::VectorXd S4{6};
    Eigen::VectorXd S5{6};
    Eigen::VectorXd S6{6};

};