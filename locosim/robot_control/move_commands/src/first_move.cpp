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
    ros::spinOnce();
    usleep(500000);
    
    ros::spinOnce();
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

    joint_state_publisher.moveTo((Vector3() << 0.5, 0.7, -0.6).finished(), Algebra::eul2RotM((Vector3() << M_PI, 0.0, 0.0).finished()), 40.0);
    usleep(500000);

    std::cout << "Tegister Blocks" << std::endl;
    ros::spinOnce();
    joint_state_publisher.registerBlocks(visionResult);

    usleep(500000);

    std::cout << "Make Castle" << std::endl;
    ros::spinOnce();
    Json::Value json = joint_state_publisher.readJson();
    joint_state_publisher.castle(json);

    ros::spin();
    loop_rate.sleep();

    ros::shutdown();
    return 0;
}