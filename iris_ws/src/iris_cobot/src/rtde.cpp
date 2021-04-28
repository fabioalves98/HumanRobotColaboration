#include "ros/ros.h"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_io_interface.h"

#include <thread>
#include <chrono>

#define ROBOT_IP "192.168.56.101"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rtde");
    ros::NodeHandle n;

    // Control Interface
    ur_rtde::RTDEControlInterface rtde_control(ROBOT_IP);
    // Receive Interface
    ur_rtde::RTDEReceiveInterface rtde_receive(ROBOT_IP);
    
    while (true)
    {
        auto t_start = std::chrono::high_resolution_clock::now();

        // Print Robot Information
        std::vector<double> actual_q = rtde_receive.getActualQ();
        // std::cout << std::endl << "Joint Positions" << std::endl;
        // for (auto i : actual_q)
            // std::cout << i << ', ';
        std::vector<double> actual_qd = rtde_receive.getActualQd();
        // std::cout << std::endl << "Joint Velocities" << std::endl;
        // for (auto i : actual_qd)
            // std::cout << i << ', ';
        std::vector<double> tcp_force = rtde_receive.getActualTCPForce();
        // std::cout << std::endl << "TCP Force" << std::endl;
        // for (auto i : tcp_force)
            // std::cout << i << ', ';
        std::vector<double> tcp_pose = rtde_receive.getActualTCPPose();
        // std::cout << std::endl << "TCP Pose" << std::endl;
        // for (auto i : tcp_pose)
            // std::cout << i << ', ';
        std::vector<double> tcp_speed = rtde_receive.getActualTCPSpeed();
        // std::cout << std::endl << "TCP Speed" << std::endl;
        // for (auto i : tcp_speed)
            // std::cout << i << ', ';
        auto t_stop = std::chrono::high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        std::cout << "Information reading stats" << std::endl;
        std::cout << t_duration.count() << std::endl;
        std::cout << 1 / t_duration.count() << std::endl;
    }
    
    
    // Parameters
    double acceleration = 0.5;
    double dt = 1.0/500; // 2ms
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
    std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Move to initial joint position with a regular moveJ
    rtde_control.moveJ(joint_q);

    // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
    for (unsigned int i=0; i<1000; i++)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        rtde_control.speedJ(joint_speed, acceleration, dt);
        joint_speed[0] += 0.0005;
        joint_speed[1] += 0.0005;
        auto t_stop = std::chrono::high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);

        if (t_duration.count() < dt)
        {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
        }
    }

    rtde_control.speedStop();
    rtde_control.stopScript();

    std::cout << "Finished Movement" << std::endl;

    // tcp_pose = rtde_receive.getActualTCPPose();
    // std::cout << "TCP Pose" << std::endl;
    // for (auto i : tcp_pose)
    //     std::cout << i << std::endl;
    
    ros::spin();
    return 0;
}