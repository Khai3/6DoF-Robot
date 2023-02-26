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

// Define different functions for simulation test

void FKSim(float theta1, float theta2, float theta3, float theta4, float theta5, float theta6)
{
    sim.startSimulation();
    sim.setJointTargetPosition(joint1,theta1);
    sim.setJointTargetPosition(joint2,theta2);
    sim.setJointTargetPosition(joint3,theta3);
    sim.setJointTargetPosition(joint4,theta4);
    sim.setJointTargetPosition(joint5,theta5);
    sim.setJointTargetPosition(joint6,theta6);

    auto end = ArmBot.ForwardKinematics(theta1,theta2,theta3,theta4,theta5,theta6);
    std::cout << end;
}

void IKSim(Eigen::Matrix4d end_pose)
{
    sim.startSimulation();
    Eigen::VectorXd theta = ArmBot.InverseKinematics(end_pose);
    sim.setJointTargetPosition(joint1,theta(0));
    sim.setJointTargetPosition(joint2,theta(1));
    sim.setJointTargetPosition(joint3,theta(2));
    sim.setJointTargetPosition(joint4,theta(3));
    sim.setJointTargetPosition(joint5,theta(4));
    sim.setJointTargetPosition(joint6,theta(5));
}

int main(int argc, char *argv[])
{   
    if (argv[1] == std::string("FKSim"))
    {
        float theta1 = 90*PI/180;
        float theta2 = 0*PI/180;
        float theta3 = 0*PI/180;
        float theta4 = 90*PI/180;
        float theta5 = 90*PI/180;
        float theta6 = 90*PI/180;
        FKSim(theta1,theta2,theta3,theta4,theta5,theta6);
    }

    else if (argv[1] == std::string("IKSim"))
    {
        Eigen::Matrix4d end;
        end << 0,0,1,60,
               1,0,0,100,
               0,1,0,306,
               0,0,0,1;

        // end << 1,0,0,90,
        //        0,1,0,50,
        //        0,0,1,200,
        //        0,0,0,1;

        // end << 0,0,1,28,
        //        1,0,0,129,
        //        0,1,0,306,
        //        0,0,0,1;
        std::cout << end;
        IKSim(end);
    }
    return 0;
}