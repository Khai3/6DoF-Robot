#include "RobotControl.h"
#include "modern_robotics.h"
#include <iostream>
#include <cmath>

#define PI 3.14159265

Robot::Robot(float r1_in, float r2_in, float r3_in, float d1_in, float d4_in, float d6_in)
    :
    r1(r1_in),
    r2(r2_in),
    r3(r3_in),
    d1(d1_in),
    d4(d4_in),
    d6(d6_in)
{   
    // Initialise home configuration of robot
    M << 0,0,1,r1+d4+d6,
         1,0,0,0,
         0,1,0,d1+r2+r3,
         0,0,0,1;
    
    // Calculate screw axes of the robot joints at its home configuration
    S1 << 0,0,1,0,0,0;  
    S2 << 0,-1,0,d1,0,-r1;
    S3 << 0,-1,0,r2+d1,0,-r1;
    S4 << 1,0,0,0,r3+r2+d1,0;     
    S5 << 0,-1,0,r3+r2+d1,0,-(r1+d4);
    S6 << 1,0,0,0,r3+r2+d1,0;      
}

Eigen::Matrix4d Robot::ForwardKinematics(const Eigen::VectorXd &theta)
{ 
    // Calculate the exponential coordinates given the screw axes and desired joint angles of robot 
    Eigen::Matrix4d exp1 = mr::MatrixExp6(mr::VecTose3(S1*theta(0)));
    Eigen::Matrix4d exp2 = mr::MatrixExp6(mr::VecTose3(S2*theta(1)));
    Eigen::Matrix4d exp3 = mr::MatrixExp6(mr::VecTose3(S3*theta(2)));
    Eigen::Matrix4d exp4 = mr::MatrixExp6(mr::VecTose3(S4*theta(3)));
    Eigen::Matrix4d exp5 = mr::MatrixExp6(mr::VecTose3(S5*theta(4)));
    Eigen::Matrix4d exp6 = mr::MatrixExp6(mr::VecTose3(S6*theta(5)));

    // Calculate end effector pose using exponential coordinates and the robot's home configuration
    Eigen::Matrix4d end_pose = exp1 * exp2 * exp3 * exp4 * exp5 * exp6 * M;
    return (end_pose);
}

Eigen::VectorXd Robot::InverseKinematics(Eigen::Matrix4d end_pose)
{
    // Calculate wrist center
    Eigen::Vector3d end_position = end_pose.block<3,1>(0,3);
    Eigen::Vector3d z_axis = end_pose.block<3,1>(0,2);
    Eigen::Vector3d wrist_center = end_position - (z_axis * d6);

    // Calculate desired joint angles of robot analytically
    Eigen::VectorXd joint_angles(6);
    float px,py,pz;
    px =  wrist_center[0]; 
    py = wrist_center[1];
    pz = wrist_center[2];

    // Joint 1
    joint_angles[0] = std::atan2(py, px);

    // Joint 2
    float l,h,p,br3,cos_b;
    l = std::sqrt(px*px + py*py) - r1;
    h = pz-d1;
    p = std::sqrt(h*h + l*l);
    br3 = std::sqrt(r3*r3+d4*d4);
    cos_b = (p*p + r2*r2 - br3*br3) / (2*p*r2); 
    joint_angles[1] = std::atan2(h, l) + std::atan2(std::sqrt(1-cos_b*cos_b),cos_b) - PI/2;

    // Joint 3
    float cos_v,d;
    cos_v = (r2*r2 + br3*br3 - p*p) / (2*r2*br3);
    d = std::atan2(d4,r3);
    joint_angles[2] = std::atan2(std::sqrt(1-cos_v*cos_v),cos_v) + d - PI; 

    // Joint 4,5 & 6
    Eigen::Matrix4d expN1 = mr::MatrixExp6(mr::VecTose3((-S1)*joint_angles[0]));
    Eigen::Matrix4d expN2 = mr::MatrixExp6(mr::VecTose3((-S2)*joint_angles[1]));
    Eigen::Matrix4d expN3 = mr::MatrixExp6(mr::VecTose3((-S3)*joint_angles[2]));
    Eigen::Matrix4d R = expN3 * expN2 * expN1 * end_pose * M.inverse(); 

    joint_angles[3] = std::atan2(-R(1,0),R(2,0));
    joint_angles[4] = std::atan2((std::sqrt(R(0,1)*R(0,1) + R(0,2)*R(0,2))), R(0,0));
    joint_angles[5] = std::atan2(-R(0,1),-R(0,2));

    return joint_angles;
}

float Robot::CubicTimeScaling(float t,float Tf)
{
    float time_ratio = t/Tf;
    float time_scale = 3*std::pow(time_ratio,2) - 2*std::pow(time_ratio,3);
    return time_scale;
}

float Robot::QuinticTimeScaling(float t,float Tf)
{
    float time_ratio = t/Tf;
    float time_scale = 10*std::pow(time_ratio, 3) - 15*std::pow(time_ratio, 4) + 6*std::pow(time_ratio, 5);
    return time_scale;
}

Eigen::MatrixXd Robot::JointTrajectory(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend,const float Tf,const int N, const std::string &method)
{
    Eigen::MatrixXd traj(N,7);
    for (int i = 0; i < N; i++)
    {
        float elapsed_time = (float)i/float(N-1) * Tf;
        float time_scale;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            time_scale = CubicTimeScaling(elapsed_time,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            time_scale = QuinticTimeScaling(elapsed_time,Tf);
        }
        
        // Calculate joint angles for specific elapsed time 
        Eigen::VectorXd joint_angles = thetastart + time_scale * (thetaend - thetastart);
        traj.block<1,7>(i,0) << joint_angles.transpose(), elapsed_time;
    }
    return traj;
}

std::vector<std::tuple<Eigen::Matrix4d, float>> Robot::ScrewTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend,const float Tf,const int N, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj;
    for (int i = 0; i < N; i++)
    {
        float elapsed_time = (float)i/float(N-1) * Tf;
        float time_scale;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            time_scale = CubicTimeScaling(elapsed_time,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            time_scale = QuinticTimeScaling(elapsed_time,Tf);
        }
   
        // Calculate SE(3) matrix for specific elapsed time
        Eigen::Matrix4d transform = Xstart * mr::MatrixExp6(mr::MatrixLog6(Xstart.inverse()*Xend) * time_scale);
        std::tuple<Eigen::Matrix4d,float> transform_stamped = std::make_tuple(transform,elapsed_time);
        traj.push_back(transform_stamped);
    }
    return traj;
}

std::vector<std::tuple<Eigen::Matrix4d, float>> Robot::CartesianTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend,const float Tf,const int N, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj ;

    // Initialise start & ending positions and rotations
    Eigen::Vector3d start_pos = Xstart.col(3).head(3);
    Eigen::Vector3d end_pos = Xend.col(3).head(3);
    Eigen::Matrix3d start_rot = Xstart.block<3,3>(0,0);
    Eigen::Matrix3d end_rot = Xend.block<3,3>(0,0);

    for (int i = 0; i < N; i++)
    {
        float elapsed_time = (float)i/float(N-1) * Tf;
        float time_scale;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            time_scale = CubicTimeScaling(elapsed_time,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            time_scale = QuinticTimeScaling(elapsed_time,Tf);
        }
   
        // Calculate SE(3) matrix for specific elapsed time
        Eigen::Vector3d position = start_pos + time_scale * (end_pos - start_pos);
        Eigen::Matrix3d rotation = start_rot * mr::MatrixExp3(mr::MatrixLog3(start_rot.inverse()*end_rot) * time_scale);
        Eigen::Matrix4d transform; 
        transform << rotation, position, 0, 0, 0, 1;
        std::tuple<Eigen::Matrix4d,float> transform_stamped = std::make_tuple(transform,elapsed_time);
        traj.push_back(transform_stamped);
    }
    return traj;
}