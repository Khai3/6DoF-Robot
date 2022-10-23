#include "RemoteAPIClient.h"
#include "RobotControl.h"
#include <iostream>

#define PI 3.14159265

int main()
{
    RemoteAPIClient client;
    auto sim = client.getObject().sim();
    Robot ArmBot;

    auto joint1 = sim.getObject("/Joint_1");
    auto joint2 = sim.getObject("/Joint_2");
    auto joint3 = sim.getObject("/Joint_3");
    auto joint4 = sim.getObject("/Joint_4");
    auto joint5 = sim.getObject("/Joint_5");
    auto joint6 = sim.getObject("/Joint_6");
    
    float theta1 = 90*PI/180;
    float theta2 = 0;
    float theta3 = 0;
    float theta4 = 0;
    float theta5 = 0;
    float theta6 = 0;
    // client.setStepping(true);
    sim.startSimulation();

    // while (true)
    // {
    sim.setJointTargetPosition(joint1,90*PI/180);
    auto end = ArmBot.ForwardKinematics(theta1,theta2,theta3,theta4,theta5,theta6);
    std::cout << end;
    //     client.step();
    // }
    // sim.stopSimulation();

    return 0;
}