#include "RemoteAPIClient.h"
#include "RobotControl.h"
#include <iostream>
#include <unistd.h>

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

void TrajSim(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend, const float Tf, const int N, const std::string &method)
{
    Eigen::MatrixXd traj = ArmBot.JointTrajectory(thetastart,thetaend,Tf,N,method);
    Eigen::VectorXd elapsed_time = traj.col(6);
    client.setStepping(true);
    sim.startSimulation();
    elapsed_time.array() += sim.getSimulationTime();

    for (int i = 0; i < N; i++) {
        while (true) {
            client.step();  
            if (sim.getSimulationTime() > elapsed_time(i)){
                Eigen::VectorXd theta = traj.row(i);
                setJointPos(theta);
                break;
            }
        }
    }
    // Stop simulation only after joints are at rest
    while (true){
        client.step();
        auto vel = sim.getObjectVelocity(joint6);
        std::vector<double> linearVel = std::get<0>(vel);
        std::vector<double> angularVel = std::get<1>(vel);
        Eigen::VectorXd linearVelX = Eigen::Map<Eigen::VectorXd>(linearVel.data(), linearVel.size());
        Eigen::VectorXd angularVelX = Eigen::Map<Eigen::VectorXd>(angularVel.data(), angularVel.size());
        if (linearVelX.norm() == 0 && angularVelX.norm() == 0) { break; }
    }
}

void TrajSim(const Eigen::Matrix4d &Xstart, const Eigen::Matrix4d &Xend, const float Tf, const int N, const int path, const std::string &method)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj;
    if (path == 0){
        traj = ArmBot.ScrewTrajectory(Xstart,Xend,Tf,N,method);
    }
    else if(path == 1){
        traj = ArmBot.CartesianTrajectory(Xstart,Xend,Tf,N,method);
    }
    client.setStepping(true);
    sim.startSimulation();
    const float start_time = sim.getSimulationTime(); 

    for (const auto& transform_stamped : traj) {
        Eigen::MatrixXd transform = std::get<0>(transform_stamped);
        float elapsed_time = std::get<1>(transform_stamped) + start_time;
        while (true) {
            client.step(); 
            if (sim.getSimulationTime() > elapsed_time) {
                Eigen::VectorXd theta = ArmBot.InverseKinematics(transform);
                setJointPos(theta);
                break;
            }
        }
    }
    // Stop simulation only after joints are at rest
    while (true){
        client.step();
        auto vel = sim.getObjectVelocity(joint6);
        std::vector<double> linearVel = std::get<0>(vel);
        std::vector<double> angularVel = std::get<1>(vel);
        Eigen::VectorXd linearVelX = Eigen::Map<Eigen::VectorXd>(linearVel.data(), linearVel.size());
        Eigen::VectorXd angularVelX = Eigen::Map<Eigen::VectorXd>(angularVel.data(), angularVel.size());
        if (linearVelX.norm() == 0 && angularVelX.norm() == 0) { break; }
    }
}

int main(int argc, char *argv[])
{   
    Eigen::VectorXd thetaend(6),thetastart(6);
    thetastart << 0,0,0,0,0,0;
    thetaend << -110*PI/180,
                -55*PI/180,
                -10*PI/180,
                0*PI/180,
                90*PI/180,
                90*PI/180;

    Eigen::Matrix4d Xstart,Xend;
    Xstart << 0,0,1,192.5,
           1,0,0,0,
           0,1,0,269,
           0,0,0,1;
    Xend << 0.14,-0.94,-0.31,-80.6,
            0.4,0.34,-0.85,-221.5,
            0.91,0,0.42,112.424,
            0,0,0,1;

    // Run simulation test based on input argument
    if (argv[1] == std::string("FKSim"))
    {
        FKSim(thetaend);
    }

    else if (argv[1] == std::string("IKSim"))
    {
        std::cout << Xend;
        IKSim(Xend);
    }

    else if (argv[1] == std::string("JointTraj"))
    {
        float Tf = 3;
        int N = 20;
        TrajSim(thetastart,thetaend,Tf,N,"Quintic");
        sleep(1);
        TrajSim(thetaend,thetastart,Tf,N,"Quintic");
    }

    else if (argv[1] == std::string("ScrewTraj"))
    {
        float Tf = 3;
        int N = 20;
        TrajSim(Xstart,Xend,Tf,N,0,"Quintic");
        sleep(1);
        TrajSim(Xend,Xstart,Tf,N,0,"Quintic");
    }

    else if (argv[1] == std::string("CartesianTraj"))
    {
        float Tf = 3;
        int N = 30;
        TrajSim(Xstart,Xend,Tf,N,1,"Quintic");
        sleep(1);
        TrajSim(Xend,Xstart,Tf,N,1,"Quintic");
    }

    return 0;
}