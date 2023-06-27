#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "kinematic.h"
#include "joint_state_publisher.h"
#include "block.h"
#include <unistd.h>

int main(int argc, char **argv)
{
    JointStatePublisher joint_state_publisher(argc, argv);

    // example: ./launchWorld.sh 3 X1-Y1-Z2 X2-Y2-Z2 X1-Y2-Z1 X1-Y3-Z2-FILLET X1-Y4-Z2 X1-Y2-Z1

    ros::Rate loop_rate(1000);
    ros::spinOnce();
    usleep(500000);

    ros::spinOnce();
    joint_state_publisher.homingProcedure();

    std::cout << "Vision Client" << std::endl;
    vision::vision visionResult;
    if (joint_state_publisher.visionService.call(visionResult))
    {
        std::cout << "Vision Client | SUCCESS" << std::endl;
        std::cout << "#n res: " << (int)visionResult.response.n_res << std::endl;
    }
    else
    {
        std::cout << "Vision Client | ERROR" << std::endl;
        return 1;
    }
    usleep(500000);
    std::cout << "Vision Client | COMPLETED" << std::endl;
    ros::spinOnce();
    joint_state_publisher.updateJstate();

    joint_state_publisher.homingProcedure(q_front);
    
    std::cout << "Register Blocks" << std::endl;
    ros::spinOnce();
    joint_state_publisher.registerBlocks(visionResult);

    usleep(500000);

    std::cout << "Move block" << std::endl;
    ros::spinOnce();
    joint_state_publisher.multipleBlocks();

    ros::shutdown();
    return 0;
}