#pragma once

#include <Eigen/Dense>

class Robot
{
public:
    // Constructor
    // Input: link lengths of Skyentific Robot and home position M
    Robot(float r1_in=47, float r2_in=110, float r3_in=26, float d1_in = 133, float d4_in=117.5, float d6_in=28);

    // Calculate end effector pose given joint angles
    Eigen::Matrix4d ForwardKinematics(float theta1=0, float theta2=0, float theta3=0, float theta4=0, float theta5=0, float theta6=0);

    // void Inverse Kinematics();

private:
    float r1;
    float r2;
    float r3;
    float d1;
    float d4;
    float d6; 
    Eigen::Matrix4d M;
    Eigen::VectorXd S1{6};
    Eigen::VectorXd S2{6};
    Eigen::VectorXd S3{6};
    Eigen::VectorXd S4{6};
    Eigen::VectorXd S5{6};
    Eigen::VectorXd S6{6};

};