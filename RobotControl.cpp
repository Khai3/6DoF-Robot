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
    Eigen::Matrix4d endPose = exp1 * exp2 * exp3 * exp4 * exp5 * exp6 * M;
    return (endPose);
}

Eigen::VectorXd Robot::InverseKinematics(Eigen::Matrix4d endPose)
{
    // Calculate wrist center
    Eigen::Vector3d end_position = endPose.block<3,1>(0,3);
    Eigen::Vector3d z_axis = endPose.block<3,1>(0,2);
    Eigen::Vector3d wrist_center = end_position - (z_axis * d6);

    // Calculate desired joint angles of robot analytically
    Eigen::VectorXd jointAngles(6);
    float px,py,pz;
    px =  wrist_center[0]; 
    py = wrist_center[1];
    pz = wrist_center[2];

    // Joint 1
    jointAngles[0] = std::atan2(py, px);

    // Joint 2
    float l,h,p,br3,cos_b;
    l = std::sqrt(px*px + py*py) - r1;
    h = pz-d1;
    p = std::sqrt(h*h + l*l);
    br3 = std::sqrt(r3*r3+d4*d4);
    cos_b = (p*p + r2*r2 - br3*br3) / (2*p*r2); 
    jointAngles[1] = std::atan2(h, l) + std::atan2(std::sqrt(1-cos_b*cos_b),cos_b) - PI/2;

    // Joint 3
    float cos_v,d;
    cos_v = (r2*r2 + br3*br3 - p*p) / (2*r2*br3);
    d = std::atan2(d4,r3);
    jointAngles[2] = std::atan2(std::sqrt(1-cos_v*cos_v),cos_v) + d - PI; 

    // Joint 4,5 & 6
    Eigen::Matrix4d expN1 = mr::MatrixExp6(mr::VecTose3((-S1)*jointAngles[0]));
    Eigen::Matrix4d expN2 = mr::MatrixExp6(mr::VecTose3((-S2)*jointAngles[1]));
    Eigen::Matrix4d expN3 = mr::MatrixExp6(mr::VecTose3((-S3)*jointAngles[2]));
    Eigen::Matrix4d R = expN3 * expN2 * expN1 * endPose * M.inverse(); 

    jointAngles[3] = std::atan2(-R(1,0),R(2,0));
    jointAngles[4] = std::atan2((std::sqrt(R(0,1)*R(0,1) + R(0,2)*R(0,2))), R(0,0));
    jointAngles[5] = std::atan2(-R(0,1),-R(0,2));

    return jointAngles;
}

float Robot::CubicTimeScaling(float t,float Tf)
{
    float timeRatio = t/Tf;
    float timeScale = 3*std::pow(timeRatio,2) - 2*std::pow(timeRatio,3);
    return timeScale;
}

float Robot::CubicTimeScalingDot(float t,float Tf)
{  
    float timeScaleDot = 6*t/std::pow(Tf,2) - 6*std::pow(t,2)/std::pow(Tf,3);
    return timeScaleDot;
}

float Robot::QuinticTimeScaling(float t,float Tf)
{
    float timeRatio = t/Tf;
    float timeScale = 10*std::pow(timeRatio,3) - 15*std::pow(timeRatio,4) + 6*std::pow(timeRatio,5);
    return timeScale;
}

float Robot::QuinticTimeScalingDot(float t,float Tf)
{
    float timeScaleDot = 30*std::pow(t,2)/std::pow(Tf, 3) - 60*std::pow(t,3)/std::pow(Tf,4) + 30*std::pow(t,4)/std::pow(Tf,5);
    return timeScaleDot;
}

std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> Robot::JointTrajectory(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend,const float Tf,const int N, const std::string &method)
{
    std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> traj;
    for (int i = 0; i < N; i++)
    {
        float elapsedTime = (float)i/float(N-1) * Tf;
        float timeScale,timeScaleDot;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            timeScale = CubicTimeScaling(elapsedTime,Tf);
            timeScaleDot = CubicTimeScalingDot(elapsedTime,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            timeScale = QuinticTimeScaling(elapsedTime,Tf);
            timeScaleDot = QuinticTimeScalingDot(elapsedTime,Tf);
        }
        
        // Calculate joint angles for specific elapsed time 
        Eigen::VectorXd jointAngles = thetastart + timeScale * (thetaend - thetastart);
        Eigen::VectorXd jointVelocity = timeScaleDot * (thetaend - thetastart);
        std::tuple<Eigen::VectorXd, Eigen::VectorXd, float> jointStamped = std::make_tuple(jointAngles,jointVelocity,elapsedTime);
        traj.push_back(jointStamped);
    }
    return traj;
}

std::vector<std::tuple<Eigen::Matrix4d, float>> Robot::ScrewTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend,const float Tf,const int N, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj;
    for (int i = 0; i < N; i++)
    {
        float elapsedTime = (float)i/float(N-1) * Tf;
        float timeScale;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            timeScale = CubicTimeScaling(elapsedTime,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            timeScale = QuinticTimeScaling(elapsedTime,Tf);
        }
   
        // Calculate SE(3) matrix for specific elapsed time
        Eigen::Matrix4d transform = Xstart * mr::MatrixExp6(mr::MatrixLog6(Xstart.inverse()*Xend) * timeScale);
        std::tuple<Eigen::Matrix4d,float> transformStamped = std::make_tuple(transform,elapsedTime);
        traj.push_back(transformStamped);
    }
    return traj;
}

std::vector<std::tuple<Eigen::Matrix4d, float>> Robot::CartesianTrajectory(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend,const float Tf,const int N, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj ;

    // Initialise start & ending positions and rotations
    Eigen::Vector3d startPos = Xstart.col(3).head(3);
    Eigen::Vector3d endPos = Xend.col(3).head(3);
    Eigen::Matrix3d startRot = Xstart.block<3,3>(0,0);
    Eigen::Matrix3d endRot = Xend.block<3,3>(0,0);

    for (int i = 0; i < N; i++)
    {
        float elapsedTime = (float)i/float(N-1) * Tf;
        float timeScale;

        // Calculate polynomial time scaling
        if (method == std::string("Cubic"))
        {
            timeScale = CubicTimeScaling(elapsedTime,Tf);
        }
        else if (method == std::string("Quintic"))
        {
            timeScale = QuinticTimeScaling(elapsedTime,Tf);
        }
   
        // Calculate SE(3) matrix for specific elapsed time
        Eigen::Vector3d position = startPos + timeScale * (endPos - startPos);
        Eigen::Matrix3d rotation = startRot * mr::MatrixExp3(mr::MatrixLog3(startRot.inverse()*endRot) * timeScale);
        Eigen::Matrix4d transform; 
        transform << rotation, position, 0, 0, 0, 1;
        std::tuple<Eigen::Matrix4d,float> transformStamped = std::make_tuple(transform,elapsedTime);
        traj.push_back(transformStamped);
    }
    return traj;
}

std::vector<std::tuple<Eigen::Matrix4d, float>> Robot::ViaTrajectory(const std::vector<Eigen::Matrix4d> points, float Tf, int N)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj ;
    int length = points.size();
    float timeGap = Tf/(length-1);

    // Calculate heuristic for velocity
    float dist = 0.0;
    for (int i = 0; i < length - 1; i++) 
    {
        dist += (points[i] - points[i+1]).norm();  // Compute total euclidean distance
    }
    double speed = (dist/(Tf));

    for (int i = 0; i < length-1; i++)
    {
        // Calculate cubic polynomial between via points i and i+1 for position. 
        Eigen::Matrix4d mat1 = points[i],             mat2 = points[i+1];
        Eigen::Vector3d pos1 = mat1.col(3).head(3),   pos2 = mat2.col(3).head(3),   pos3 = points[i+2].col(3).head(3);     
        Eigen::Matrix3d rot1 = mat1.block<3,3>(0,0),  rot2 = mat2.block<3,3>(0,0);
        Eigen::Vector3d vel1,vel2;
        vel1 = (i == 0) ? Eigen::Vector3d::Zero() : vel2;
        vel2 = (i+1 == length-1) ? Eigen::Vector3d::Zero() : (pos3-pos2).normalized();
        // vel2 *= speed;

        Eigen::Vector3d a0 = pos1;
        Eigen::Vector3d a1 = vel1;
        Eigen::Vector3d a2 = (3*pos2 - 3*pos1 - 2*vel1*timeGap - vel2*timeGap)/(timeGap*timeGap);
        Eigen::Vector3d a3 = (2*pos1 + (vel1+vel2)*timeGap - 2*pos2) / std::pow(timeGap,3);

        for (int j = 0; j < N; j++)
        {
            float delta_t = (float)j/float(N-1) * timeGap; 
            Eigen::Vector3d position = a0 + a1*delta_t + a2*std::pow(delta_t,2) + a3*std::pow(delta_t,3);

            // Use cubic time scaling for rotation
            float timeScale = CubicTimeScaling(delta_t,timeGap);
            Eigen::Matrix3d rotation = rot1 * mr::MatrixExp3(mr::MatrixLog3(rot1.inverse()*rot2) * timeScale);
            
            Eigen::Matrix4d transform; 
            transform << rotation, position, 0, 0, 0, 1;
            std::tuple<Eigen::Matrix4d,float> transformStamped = std::make_tuple(transform, (timeGap*i + delta_t));
            traj.push_back(transformStamped);
        }
    }
    return traj;
}

Eigen::VectorXd Robot::VelocityControl(const float Kp, const float Ki, const Eigen::VectorXd currentAngles, const Eigen::VectorXd desiredAngles, const float dt, const Eigen::VectorXd feedforward)
{
    Eigen::MatrixXd matKp = Eigen::MatrixXd::Identity(6, 6) * Kp;
    Eigen::MatrixXd matKi = Eigen::MatrixXd::Identity(6, 6) * Ki;
    Eigen::VectorXd error = desiredAngles - currentAngles;
    static Eigen::VectorXd errorInt = Eigen::VectorXd::Zero(6);
    errorInt += error*dt;

    Eigen::VectorXd jointVelocity = feedforward + matKp*error + matKi*errorInt;
    return jointVelocity;
}