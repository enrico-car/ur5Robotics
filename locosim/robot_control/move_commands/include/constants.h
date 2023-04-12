#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#define SOFT_GRIPPER

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

    static const int vel_limits = 3.14;
    static const std::vector<std::string> joint_names;

    // * ros node definitions
    static const std::string init_node;
    static const std::string pub_des_jstate;
    static const std::string pub_traj_jstate;
    static const std::string sub_jstate;
};

const std::vector<std::string> Constants::joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

const std::string Constants::init_node = "custom_joint_pub_node_cpp";
const std::string Constants::pub_des_jstate = "/ur5/joint_group_pos_controller/command";
const std::string Constants::pub_traj_jstate = "/ur5/pos_joint_traj_controller/command";
const std::string Constants::sub_jstate = "/ur5/joint_states";

const Vector6 Constants::d = (Vector6() << D1, 0, 0, D4, D5, D6).finished();
const Vector6 Constants::a = (Vector6() << 0, 0, A2, A3, 0, 0).finished();
const Vector6 Constants::alph = (Vector6() << M_PI, M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2).finished();

#endif