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

#include "constants.h"
#include "kinematic.h"



class JointStatePublisher
{
private:
    ros::Publisher pub_des_joint_state;
    ros::Publisher pub_traj_jstate;
    ros::Subscriber sub_jstate;


    std::vector<double> linspace(double start, double end, int num);
    void subCallback(const sensor_msgs::JointState &msg);

public:
    JointStatePublisher(int argc, char **argv);

    void sendDesJState(std::vector<double> q_des);
    void sendDesTrajectory(const Trajectory& trajectory)const;
};

#endif
