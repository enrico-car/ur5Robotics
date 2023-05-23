#ifndef __JOINT_STATE_PUBLISHER_h__
#define __JOINT_STATE_PUBLISHER_h__

#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include "gazebo_ros_link_attacher/Attach.h"
#include "gazebo_ros_link_attacher/SetStatic.h"
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <vision/vision.h>
#include <visualization_msgs/MarkerArray.h>

#include "constants.h"
#include "kinematic.h"
#include "block.h"


class JointStatePublisher
{
private:
    Vector6 q;
    Vector6 jstate;
    Block block;
    std::vector<Block> presentBlocks;
    visualization_msgs::MarkerArray markerArray;

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
    ros::Publisher pubTrajJstate;
    ros::Publisher markerPub;

    ros::Subscriber subJstate;

    ros::ServiceClient attachSrv;
    ros::ServiceClient detachSrv;
    ros::ServiceClient setStaticSrv;

    ros::ServiceClient getWorldProperties;
    ros::ServiceClient getModelState;

    

    void receiveJstate(const sensor_msgs::JointState &state);

#ifdef SOFT_GRIPPER
    Vector2 mapGripperJointState(const double &diameter)
    {
        double d0 = 40.0;
        double l = 60.0;
        double delta = 0.5 * (diameter - d0);
        Vector2 ones;
        ones << 1.0, 1.0;
        return atan2(delta, l) * ones;
    }
#endif
#ifndef SOFT_GRIPPER
    Vector3 mapGripperJointState(const double &diameter)
    {
        Vector3 ones;
        ones << 1.0, 1.0, 1.0;
        return ((diameter - 22.0) / (130.0 - 22.0) * (-M_PI) + M_PI) * ones;
    }
#endif
    // MoveRealGripper

    std::pair<int, int> getGripPositions();
    void ungripping(const double &gripperPos, const bool &attachToTable);
    void gripping(const double &gripperPos);

public:

    ros::ServiceClient visionService;

    JointStatePublisher(int argc, char **argv);

    void sendDesJState(std::vector<double> qDes, std::vector<double> qDesGripper);
    void sendDesTrajectory(const Trajectory &trajectory) const;

    void moveTo(const Vector3 &finalP, const Matrix3 &finalRotm, double gripperPos, const bool &waitForEnd = false, const CurveType &curveType = CurveType::BEZIER, const double &vel = 2, const bool &useIK = false);
    void pickAndPlaceBlock(const Vector3 &finalP, RPY finalRpy, const double& zOffset=0.0,const bool &attachToTable=true);
    void rotateBlock(const RPY &newBlockRpy);
    void setupBlockForRotation();
    void rotateBlockStandardPosition(double xLandPose, double yLandPose, RPY finalRpy);

    void homingProcedure();
    void multipleBlocks(Detected d);
    void castle(Json::Value json);

    Json::Value readJson();
    double getYaw(double rot);
    void registerBlocks(const vision::vision &visionResult);

    void updateJstate();
};

#endif
