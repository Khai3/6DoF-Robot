#include "RemoteAPIClient.h"
#include "RobotControl.h"
#include <iostream>
#include <unistd.h>

#define PI 3.14159265

// Initialise Armbot and get joint Objects from Coppelia Simulation 
RemoteAPIClient client;
auto sim = client.getObject().sim();
Robot ArmBot;
std::vector<int> markers;
Eigen::Vector3d prevPos = Eigen::Vector3d::Zero();

int joint1 = sim.getObject("/Joint_1");
int joint2 = sim.getObject("/Joint_2");
int joint3 = sim.getObject("/Joint_3");
int joint4 = sim.getObject("/Joint_4");
int joint5 = sim.getObject("/Joint_5");
int joint6 = sim.getObject("/Joint_6");

//=================
// Helper Functions
//=================

void setJointPos(const Eigen::VectorXd &theta) 
{
    sim.setJointTargetPosition(joint1,theta(0));
    sim.setJointTargetPosition(joint2,theta(1));
    sim.setJointTargetPosition(joint3,theta(2));
    sim.setJointTargetPosition(joint4,theta(3));
    sim.setJointTargetPosition(joint5,theta(4));
    sim.setJointTargetPosition(joint6,theta(5));
}

void setJointVel(const Eigen::VectorXd vel) 
{
    sim.setJointTargetVelocity(joint1,vel(0));
    sim.setJointTargetVelocity(joint2,vel(1));
    sim.setJointTargetVelocity(joint3,vel(2));
    sim.setJointTargetVelocity(joint4,vel(3));
    sim.setJointTargetVelocity(joint5,vel(4));
    sim.setJointTargetVelocity(joint6,vel(5));
}

Eigen::VectorXd getJointVel()
{
    float vel1 = sim.getJointVelocity(joint1);
    float vel2 = sim.getJointVelocity(joint2);
    float vel3 = sim.getJointVelocity(joint3);
    float vel4 = sim.getJointVelocity(joint4);
    float vel5 = sim.getJointVelocity(joint5);
    float vel6 = sim.getJointVelocity(joint6);
    Eigen::VectorXd currentTheta(6); 
    currentTheta << vel1,vel2,vel3,vel4,vel5,vel6;
    return currentTheta;
}

void stepToRest()
{   
    // Step client simulation until end effector is at rest
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

void markPoints(const std::vector<Eigen::Matrix4d> transformList, const double rad = 0.02)
{
    // Remove existing markers
    for (const int marker : markers){
        std::cout << marker << std::endl;
        sim.removeObject(marker);
    }
    markers.clear();

    // Add markers to input points
    for (const auto& transform : transformList) {
        Eigen::Vector3d point = transform.col(3).head(3);

        // get a handle to the sphere object
        std::vector<double> radius(3, rad);
        int sphereHandle = sim.createPrimitiveShape(sim.primitiveshape_spheroid, radius);
        markers.push_back(sphereHandle);
        // set the sphere's position
        double scale = 1000;
        std::vector<double> spherePos = {point[0]/scale, point[1]/scale, point[2]/scale};
        sim.setObjectPosition(sphereHandle, -1, spherePos);
        // set the sphere's color
        std::vector<double> sphereColor = {1.0, 0.3, 0.1};
        sim.setShapeColor(sphereHandle,"",sim.colorcomponent_ambient_diffuse, sphereColor);
    }
}

void markTrail(int object, const double rad = 0.005, const float gap = 0.005)
{   
    std::vector<double> p = sim.getObjectPosition(object,sim.handle_world);
    Eigen::Vector3d pos = Eigen::Map<Eigen::Vector3d>(p.data());
    // std::cout << pos << std::endl;
    if ((pos - prevPos).norm() > gap)
    {
        // get a handle to the sphere object
        std::vector<double> radius(3, rad);
        int sphereHandle = sim.createPrimitiveShape(sim.primitiveshape_spheroid, radius);
        // set the sphere's position
        std::vector<double> spherePos = {pos[0], pos[1], pos[2]};
        sim.setObjectPosition(sphereHandle, -1, spherePos);
        // set the sphere's color
        std::vector<double> sphereColor = {1.0, 0.3, 0.1};
        sim.setShapeColor(sphereHandle,"",sim.colorcomponent_ambient_diffuse, sphereColor);

        prevPos = pos;
    } 
}

void trackJointStamped(std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> traj)
{
    // Set the joint angles in the simulation as specified in the trajectory input 
    const float startTime = sim.getSimulationTime(); 
    for (const auto& jointStamped : traj) {
        Eigen::VectorXd theta = std::get<0>(jointStamped);
        float elapsedTime = std::get<2>(jointStamped) + startTime;
        while (true) {
            client.step(); 
            if (sim.getSimulationTime() > elapsedTime) {
                setJointPos(theta);
                break;
            }
        }
    }
}

void trackTransformStamped(const std::vector<std::tuple<Eigen::Matrix4d, float>> traj, const bool mark = false)
{
    // Set the end effector transformation in the simulation as specified in the trajectory input 
    const float startTime = sim.getSimulationTime(); 
    for (const auto& transform_stamped : traj) {
        Eigen::MatrixXd transform = std::get<0>(transform_stamped);
        float elapsedTime = std::get<1>(transform_stamped) + startTime;
        while (true) {
            client.step(); 
            if (sim.getSimulationTime() > elapsedTime) {
                Eigen::VectorXd theta = ArmBot.InverseKinematics(transform);
                setJointPos(theta);
                break;
            }
            if (mark) { markTrail(joint6); }
        }
    }
}

void JointVelControl(std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> traj,float Kp, float Ki)
{
    const float startTime = sim.getSimulationTime(); 
        for (const auto& jointStamped : traj) {
            Eigen::VectorXd thetaTarget = std::get<0>(jointStamped);
            Eigen::VectorXd thetaDot = std::get<1>(jointStamped);
            float elapsedTime = std::get<2>(jointStamped) + startTime;
            while (true) {
                client.step(); 
                Eigen::VectorXd currentTheta = getJointVel();
                Eigen::VectorXd jointVel = ArmBot.VelocityControl(Kp, Ki, currentTheta, thetaTarget, sim.getSimulationTimeStep, thetaDot);
                setJointVel(jointVel);
                if (sim.getSimulationTime() > elapsedTime) {
                    break;
                }
            }
        }
}

//=================
// Simulation Tests
//=================

void FKSim(const Eigen::VectorXd &theta)
{
    sim.startSimulation();
    setJointPos(theta);
    Eigen::Matrix4d end = ArmBot.ForwardKinematics(theta);
    std::cout << end;
}

void IKSim(Eigen::Matrix4d endpose)
{
    sim.startSimulation();
    Eigen::VectorXd theta = ArmBot.InverseKinematics(endpose);
    setJointPos(theta);    
}

void TrajSim(const Eigen::VectorXd &thetastart, const Eigen::VectorXd &thetaend, const float Tf, const int N, const std::string &method, const std::string &control)
{
    std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, float>> traj = ArmBot.JointTrajectory(thetastart,thetaend,Tf,N,method);
    client.setStepping(true);
    sim.startSimulation();
    
    if (control == std::string("Velocity")){
        float Kp = 1;
        float Ki = 0.5;
        JointVelControl(traj,Kp,Ki);
    }
    else{ trackJointStamped(traj);}
    stepToRest();
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

    // markPoints(std::vector<Eigen::Matrix4d>{Xstart,Xend});
    trackTransformStamped(traj);
    stepToRest();
}

void ViaTraj(const std::vector<Eigen::Matrix4d> points, const float Tf, const int N, const bool mark = true)
{
    std::vector<std::tuple<Eigen::Matrix4d, float>> traj;
    traj = ArmBot.ViaTrajectory(points,Tf,N);
    client.setStepping(true);
    sim.startSimulation();
    
    if (mark) { markPoints(points); }
    trackTransformStamped(traj,false);
    stepToRest();
}

int main(int argc, char *argv[])
{  
    // Basic joints and end effector configuration inputs 
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

    else if (argv[1] == std::string("ViaTraj"))
    {
        float Tf = 8;
        int N = 20;
        Eigen::Matrix4d point1,point2,point3,point4;
        point1 << 0,0,1,120,
                  1,0,0,-221.5,
                  0,1,0,130,
                  0,0,0,1;
        point2 << 0,0,1,70,
                  1,0,0,-221.5,
                  0,1,0,270,
                  0,0,0,1;
        point3 << 0,0,1,20,
                  1,0,0,-221.5,
                  0,1,0,130,
                  0,0,0,1;
        point4 << 0,0,1,-30,
                  1,0,0,-221.5,
                  0,1,0,270,
                  0,0,0,1;
        Xend(2,3) = 130;
        std::vector<Eigen::Matrix4d> points = {Xstart, point1, point2, point3, point4, Xend};
        ViaTraj(points,Tf,N,true);
    }
    return 0;
}