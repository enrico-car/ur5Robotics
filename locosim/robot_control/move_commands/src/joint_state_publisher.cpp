#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <cmath>

#include "constants.h"
#include "kinematic.h"
#include "joint_state_publisher.h"

JointStatePublisher::JointStatePublisher(int argc, char **argv)
{
    ros::init(argc, argv, Constants::init_node, ros::init_options::AnonymousName);
    ros::NodeHandle n;
    pub_des_joint_state = n.advertise<std_msgs::Float64MultiArray>(Constants::pub_des_jstate, 10);
    pub_traj_jstate = n.advertise<trajectory_msgs::JointTrajectory>(Constants::pub_traj_jstate, 10);
    // sub_jstate =  n.subscribe ("chatter", 1000, JointStatePublisher::subCallback);
}

void JointStatePublisher::sendDesJState(std::vector<double> q_des)
{
    if (q_des.size() < 6)
    {
        std::cout << "Wrong Jstate!!!" << std::endl;
    }
    else if (q_des.size() == 6)
    {
        // q_des.push_back(0.0);
        // q_des.push_back(0.0);
    }
    std_msgs::Float64MultiArray msg;
    msg.data = q_des;
    pub_des_joint_state.publish(msg);
}

void JointStatePublisher::sendDesTrajectory(const Trajectory &trajectory) const
{
    trajectory_msgs::JointTrajectory jt;
    jt.joint_names = Constants::joint_names;
    jt.header.stamp = ros::Time().now();

    trajectory_msgs::JointTrajectoryPoint jtp;

    for (int i = 0; i < trajectory.positions.size(); i++)
    {
        // std::cout << "position" << i << ":" << std::endl;
        // for (int j = 0; j < trajectory.positions[i].size(); j++)
        // {
        //     std::cout << trajectory.positions[i][j] << " " << std::endl;
        // }
        // std::cout << "velocities" << i << ":" << std::endl;
        // for (int j = 0; j < trajectory.velocities[i].size(); j++)
        // {
        //     std::cout << trajectory.velocities[i][j] << " " << std::endl;
        // }
        jtp.positions = trajectory.positions[i];
        jtp.velocities = trajectory.velocities[i];
        jtp.time_from_start = ros::Duration(trajectory.times[i]);
        jt.points.push_back(jtp);
    }

    pub_traj_jstate.publish(jt);
}
