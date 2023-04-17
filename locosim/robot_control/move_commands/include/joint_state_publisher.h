#ifndef __JOINT_STATE_PUBLISHER_h__
#define __JOINT_STATE_PUBLISHER_h__

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/SetStatic.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <vision/vision.h>

#include "constants.h"
#include "kinematic.h"
#include "block.h"

class JointStatePublisher
{
private:
    Vector6 q;
    Vector6 jstate;
    Block block;
    std::vector<Block> presentBlock;

    // * Robot configuration
    bool realRobot;
    bool vision;
    bool gripper;
    bool gripperSim;
    bool softGripper;
    bool useGraspPlugin;
#ifdef SOFT_GRIPPER
    Vector2 qGripper;
#endif
#ifndef SOFT_GRIPPER
    Vector3 qGripper;
#endif

    // * Ros topic
    ros::Publisher pubDesJstate;
    ros::Publisher pubtrajJstate;
    ros::Subscriber subJstate;

    ros::ServiceClient attachSrv;
    ros::ServiceClient detachSrv;
    ros::ServiceClient setStaticSrv;

    ros::ServiceClient getWorldProperties;
    ros::ServiceClient getModelState;

    ros::ServiceClient visionService;


    void receiveJstate(const sensor_msgs::JointState &state);

public:
    JointStatePublisher(int argc, char **argv);

    void sendDesJState(std::vector<double> q_des);
    void sendDesTrajectory(const Trajectory &trajectory) const;
};

#endif
