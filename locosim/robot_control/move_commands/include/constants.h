#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>

// * params
const double q0[6] = {-0.32, -0.2, -2.8, -1.63, 0, -1.0};
const std::string frameName = "tool0";
const std::string jointNames[6] = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const std::string softGripperJointNames[2] = {"hand_1_joint", "hand_2_joint"};
const std::string gripperJointNames[3] = {"hand_1_joint", "hand_2_joint", "hand_3_joint"};
const int vel_limits = 3.14;

// #define REAL_ROBOT
#define GRIPPER_SIM
#define SOFT_GRIPPER
#define USE_GRASP_PLUGIN
#define VISION

#define D1 0.163
#define A2 -0.4250
#define A3 -0.39225
#define D4 0.134
#define D5 0.100

#ifdef SOFT_GRIPPER
#define D6 0.2475 // soft_gripper
#endif
#ifndef SOFT_GRIPPER
#define D6 0.27
#endif

#define DESK_HEIGHT -0.935

// * end params

typedef Eigen::Matrix<double, 2, 1> Vector2;
typedef Eigen::Matrix<double, 3, 1> Vector3;
typedef Eigen::Matrix<double, 4, 1> Vector4;
typedef Eigen::Matrix<double, 6, 1> Vector6;
typedef Eigen::Matrix<double, 2, 2> Matrix2;
typedef Eigen::Matrix<double, 3, 3> Matrix3;
typedef Eigen::Matrix<double, 4, 4> Matrix4;
typedef Eigen::Matrix<double, 6, 6> Matrix6;
typedef Eigen::Matrix<double, 6, 8> IKMatrix;

class Constants
{
public:
    // * joint definitions

    static const Vector6 d;
    static const Vector6 a;
    static const Vector6 alph;

    // * ros node definitions
    static const std::string init_node;

    static const std::string pubDesJstate;
    static const std::string pubtrajJstateRealRobot;
    static const std::string pubtrajJstate;

    static const std::string subJstate;

    static const std::string attachSrv;
    static const std::string detachSrv;
    static const std::string setStaticSrv;

    static const std::string getWorldProperties;
    static const std::string getModelState;

    static const std::string visionService;
};

const std::string Constants::init_node = "custom_joint_pub_node_cpp";
const std::string Constants::pubDesJstate = "/ur5/joint_group_pos_controller/command";
const std::string Constants::pubtrajJstateRealRobot = "/ur5/scaled_pos_joint_traj_controller/command";
const std::string Constants::pubtrajJstate = "/ur5/pos_joint_traj_controller/command";
const std::string Constants::subJstate = "/ur5/joint_states";

const std::string Constants::attachSrv = "/link_attacher_node/attach";
const std::string Constants::detachSrv = "/link_attacher_node/detach";
const std::string Constants::setStaticSrv = "/link_attacher_node/setstatic";

const std::string Constants::getWorldProperties = "/gazebo/get_world_properties";
const std::string Constants::getModelState = "/gazebo/get_model_state";

const std::string Constants::visionService = "vision_service";



const Vector6 Constants::d = (Vector6() << D1, 0, 0, D4, D5, D6).finished();
const Vector6 Constants::a = (Vector6() << 0, 0, A2, A3, 0, 0).finished();
const Vector6 Constants::alph = (Vector6() << M_PI, M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2).finished();

#endif