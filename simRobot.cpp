#include "RemoteAPIClient.h"
#include "RobotControl.h"
#include <iostream>

#define PI 3.14159265

// Initialise Armbot and get joint Objects from Coppelia Simulation
RemoteAPIClient client;
auto sim = client.getObject().sim();
Robot ArmBot;

auto joint1 = sim.getObject("/Joint_1");
auto joint2 = sim.getObject("/Joint_2");
auto joint3 = sim.getObject("/Joint_3");
auto joint4 = sim.getObject("/Joint_4");
auto joint5 = sim.getObject("/Joint_5");
auto joint6 = sim.getObject("/Joint_6");

void setJointPos(const Eigen::VectorXd &theta)
{
    sim.setJointTargetPosition(joint1,theta(0));
    sim.setJointTargetPosition(joint2,theta(1));
    sim.setJointTargetPosition(joint3,theta(2));
    sim.setJointTargetPosition(joint4,theta(3));
    sim.setJointTargetPosition(joint5,theta(4));
    sim.setJointTargetPosition(joint6,theta(5));
}

// Define simulation tests

void FKSim(const Eigen::VectorXd &theta)
{
    sim.startSimulation();
    setJointPos(theta);

    auto end = ArmBot.ForwardKinematics(theta);
    std::cout << end;
}

void IKSim(Eigen::Matrix4d end_pose)
{
    sim.startSimulation();
    Eigen::VectorXd theta = ArmBot.InverseKinematics(end_pose);
    setJointPos(theta);    
}

void TrajSim(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend, int Tf, int N, const std::string &method)
{
    Eigen::MatrixXd traj = ArmBot.JointTrajectory(thetastart,thetaend,Tf,N,method);
    Eigen::VectorXd elapsed_time = traj.col(6);
    for (int i = 0; i < N; i++) {
        while (true) {   
            if (sim.getSimulationTime() > elapsed_time(i)){
                std::cout << sim.getSimulationTime();
                Eigen::VectorXd theta = traj.row(i);
                setJointPos(theta);
                break;
            }
        }
    }
}

void TrajSim(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend, int Tf, int N, int path, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj;
    if (path = 0){
        traj = ArmBot.ScrewTrajectory(Xstart,Xend,Tf,N,method);
    }
    else if(path = 1){
        traj = ArmBot.CartesianTrajectory(Xstart,Xend,Tf,N,method);
    }
    for (const auto& transform_stamped : traj) {
        Eigen::MatrixXd transform = std::get<0>(transform_stamped);
        float elapsed_time = std::get<1>(transform_stamped);
        while (true) {
            if (sim.getSimulationTime() > elapsed_time) {
                Eigen::VectorXd theta = ArmBot.InverseKinematics(transform);
                setJointPos(theta);
                break;
            }
        }
    }
}

int main(int argc, char *argv[])
{   
    // Input basic robot configurations
    Eigen::VectorXd thetaend(6),thetastart(6);
    thetastart << 0,0,0,0,0,0;
    thetaend << 90*PI/180,
                0*PI/180,
                0*PI/180,
                90*PI/180,
                90*PI/180,
                90*PI/180;

    Eigen::Matrix4d Xstart,Xend;
    Xstart << 0,0,1,192.5,
           1,0,0,0,
           0,1,0,269,
           0,0,0,1;
    Xend << 0,0,1,60,
           1,0,0,100,
           0,1,0,306,
           0,0,0,1;

    // Run simulation test based on input argument
    if (argv[1] == std::string("FKSim"))
    {
        FKSim(thetaend);
    }

    else if (argv[1] == std::string("IKSim"))
    {

        // end << 1,0,0,90,
        //        0,1,0,50,
        //        0,0,1,200,
        //        0,0,0,1;

        // end << 0,0,1,28,
        //        1,0,0,129,
        //        0,1,0,306,
        //        0,0,0,1;
        std::cout << Xend;
        IKSim(Xend);
    }

    else if (argv[1] == std::string("JointTraj"))
    {
        int Tf = 5;
        int N = 10;
        TrajSim(thetastart,thetaend,Tf,N,"cubic");
    }

    else if (argv[1] == std::string("ScrewTraj"))
    {
        int Tf = 5;
        int N = 10;
        TrajSim(Xstart,Xend,Tf,N,0,"cubic");
    }

    else if (argv[1] == std::string("CartesianTraj"))
    {
        int Tf = 5;
        int N = 10;
        TrajSim(Xstart,Xend,Tf,N,1,"cubic");
    }

    return 0;
}