#ifndef __JOINT_STATE_PUBLISHER_h__
#define __JOINT_STATE_PUBLISHER_h__

#include <iostream>
#include <fstream>
#include <random>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>

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

    // * Ros topics
    ros::Publisher pubDesJstate;
    ros::Publisher pubTrajJstate;
    ros::Publisher markerPub;

    ros::Subscriber subJstate;

    ros::ServiceClient attachSrv;
    ros::ServiceClient detachSrv;
    ros::ServiceClient setStaticSrv;

    ros::ServiceClient getWorldProperties;
    ros::ServiceClient getModelState;

    
    /// @brief Callback called from the joint subscriber
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
    /// @brief Procudedure to attach block, only in simulation environment
    void ungripping(const double &gripperPos, const bool &attachToTable=false);
    /// @brief Procudedure to detach block, only in simulation environment
    void gripping(const double &gripperPos);

public:
    /// @brief vision service client
    ros::ServiceClient visionService;

    JointStatePublisher(int argc, char **argv);

    /// @brief Send joints angles to the ros topic
    /// @param qDes joint angles
    /// @param qDesGripper gripper angles
    void sendDesJState(std::vector<double> qDes, std::vector<double> qDesGripper);
    /// @brief sends joints trajectory to the ros topic
    /// @param trajectory desired trajectory
    void sendDesTrajectory(const Trajectory &trajectory) const;

    /// @brief Move the robot to a desired position using differential kinematic and ros trajectory topic
    /// @param finalP final desired position
    /// @param finalRotm final desired orientation
    /// @param gripperPos gripper desired position
    /// @param waitForEnd [true] wait the robot to reach the desired position
    /// @param curveType differential kinematic curve type
    void moveTo(const Vector3 &finalP, const Matrix3 &finalRotm, double gripperPos, const bool &waitForEnd = false, const CurveType &curveType = CurveType::BEZIER, const double &vel = 2);
    /// @brief Procedure to pick and place block to a desired position
    /// @param finalP final desired position
    /// @param finalRpy final desired orientation
    void pickAndPlaceBlock(const Vector3 &finalP, RPY finalRpy, const double& zOffset=0.0,const bool &attachToTable=true);
    /// @brief Procedure to rotate block
    void rotateBlock(const RPY &newBlockRpy, Cartesian newBlockPos);
    void setupBlockForRotation();
    /// @brief Procedure to rotate block to the stand position
    void rotateBlockStandardPosition(double xLandPose=-1, double yLandPose=-1, RPY finalRpy=RPY(-1, -1, -1));
    double checkCollision(double x, double y);
    Cartesian findFreeSpot(double castleXmin=0.6, double castleYmin=0.5);

    /// @brief move robot in the joint space
    /// Only use to reach homing position
    void homingProcedure(const Vector6 &final_q=q0);
    /// @brief Procedure to pick and place every blocks in the final position defined in [finalPosFromClass]
    void multipleBlocks();
    /// @brief Prodedure to create the castle specified in the [output.json] file
    void castle();
    /// @brief parse json file and return the json object
    Json::Value readJson();
    /// @brief convert the json rotation to the yaw rotation
    double getYaw(int rot);
    /// @brief Create a [Block] instance for each block detect from vision service
    /// @param visionResult 
    void registerBlocks(const vision::vision &visionResult);
    /// @brief active and disable gripper contact with object
    void setGripperContact(bool set)
    {
        gazebo_ros_link_attacher::SetStatic req;
        req.request.model_name = "ur5";
        req.request.link_name = "hand_1_link";
        req.request.set_static = set;
        setStaticSrv.call(req);
        req.request.model_name = "ur5";
        req.request.link_name = "hand_2_link";
        req.request.set_static = set;
        setStaticSrv.call(req);
    }
    /// @brief set the current block to handle
    /// @param b 
    void setBlock(Block b);
    /// @brief upadate current joints angles get from subscriber
    void updateJstate();
};

#endif
