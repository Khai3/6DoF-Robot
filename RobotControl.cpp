#include "RobotControl.h"
#include "modern_robotics.h"

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

Eigen::Matrix4d Robot::ForwardKinematics(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{ 
    // Calculate the exponential coordinates given the screw axes and desired joint angles of robot 
    Eigen::Matrix4d exp1 = mr::MatrixExp6(mr::VecTose3(S1*theta1));
    Eigen::Matrix4d exp2 = mr::MatrixExp6(mr::VecTose3(S2*theta2));
    Eigen::Matrix4d exp3 = mr::MatrixExp6(mr::VecTose3(S3*theta3));
    Eigen::Matrix4d exp4 = mr::MatrixExp6(mr::VecTose3(S4*theta4));
    Eigen::Matrix4d exp5 = mr::MatrixExp6(mr::VecTose3(S5*theta5));
    Eigen::Matrix4d exp6 = mr::MatrixExp6(mr::VecTose3(S6*theta6));

    // Calculate end effector pose using exponential coordinates and the robot's home configuration
    Eigen::Matrix4d end_pose = exp1 * exp2 * exp3 * exp4 * exp5 * exp6 * M;
    return (end_pose);
}

float* Robot::InverseKinematics(Eigen::Matrix4d end_pose)
{
    // Calculate wrist center
    Eigen::Vector3d end_position = end_pose.block<3,1>(3,0);
    Eigen::Vector3d x_axis = end.pose.block<3,1>(0,0);
    Eigen::Vector3d wrist_center = end_position - (x_axis * d6);

    // Calculate desired joint angles of robot analytically
    float joint_angles[5];
    px =  wrist_center[0]; 
    py = wrist_center[1];
    pz = wrist_center[2];

    joint_angles[0] = std::atan2(py, px)
    joint_angles[1] = std::atan2(pz-d1, std:sqrt(px*px + py*py) + r1)

}