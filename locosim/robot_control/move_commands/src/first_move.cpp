#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "kinematic.h"
#include "joint_state_publisher.h"
#include "block.h"
#include <unistd.h>

int main(int argc, char **argv)
{
    JointStatePublisher joint_state_publisher(argc, argv);

    ros::Rate loop_rate(1000);

    Vector6 initial_jstate;
    initial_jstate << -0.32, -0.2, -2.8, -1.63, 0, -1.0;
    initial_jstate << -0.32, -0.2, -2.56, -1.63, -1.57, -1.0;
    Vector3 final_p;
    final_p << 0.5, 0.6, -0.7;
    Vector3 temp;
    temp << -M_PI, 0, 0;
    Matrix3 final_rotm = Algebra::eul2RotM(temp);
    usleep(1000000);
    std::cout<<"sono fermo"<<std::endl;
    usleep(1000000);
    Trajectory trajectory = Kinematic::differentialKinematic(initial_jstate, final_p, final_rotm);
    std::cout << "parto" << std::endl;
    joint_state_publisher.sendDesTrajectory(trajectory);
    std::cout << "pubblicato" << std::endl;
    usleep(5000000);

    while (ros::ok())
    {

        // joint_state_publisher.sendDesJState({M_PI/2,0,0,0,0,0});

        ros::spin();
    }
    ros::shutdown();
    return 0;
}