#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "kinematic.h"
#include "joint_state_publisher.h"
#include "block.h"

int main(int argc, char **argv)
{
    JointStatePublisher joint_state_publisher(argc, argv);

    ros::Rate loop_rate(1000);

    Vector6 initial_jstate;
    initial_jstate << -0.32, -0.2, -2.8, -1.63, 0, -1.0;
    Vector3 final_p;
    final_p << 0.2, 0.5, -0.8;
    Matrix3 final_rotm;
    final_rotm << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    // Trajectory trajectory = Kinematic::differentialKinematic(initial_jstate, final_p, final_rotm);

    while (ros::ok())
    {

        // joint_state_publisher.sendDesJState({M_PI/2,0,0,0,0,0});
        // joint_state_publisher.sendDesTrajectory(trajectory);
        ros::spin();
        
    }
    ros::shutdown();
    return 0;
}