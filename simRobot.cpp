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

// Define functions for simulation test

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

int main(int argc, char *argv[])
{   
    if (argv[1] == std::string("FKTest"))
    {
        float theta1 = 90*PI/180;
        float theta2 = 10*PI/180;
        float theta3 = 10*PI/180;
        float theta4 = 90*PI/180;
        float theta5 = 90*PI/180;
        float theta6 = 90*PI/180;
        FKSim(theta1,theta2,theta3,theta4,theta5,theta6);
    }
    return 0;
}