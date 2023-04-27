#include "joint_state_publisher.h"
#include "block.h"

JointStatePublisher::JointStatePublisher(int argc, char **argv)
{
#ifdef REAL_ROBOT
    realRobot = true;
#endif
#ifndef REAL_ROBOT
    realRobot = false;
#endif

#ifdef GRIPPER_SIM
    gripperSim = true;
#endif
#ifndef GRIPPER_SIM
    gripperSim = false;
#endif

#ifdef SOFT_GRIPPER
    softGripper = true;
#endif
#ifndef SOFT_GRIPPER
    softGripper = false;
#endif

#ifdef USE_GRASP_PLUGIN
    useGraspPlugin = true;
#endif
#ifndef USE_GRASP_PLUGIN
    useGraspPlugin = false;
#endif

#ifdef VISION
    vision = true;
#endif
#ifndef VISION
    vision = false;
#endif

    if (realRobot)
    {
        gripper = false;
    }
    else
    {
        if (gripperSim)
        {
            gripper = true;
        }
        else
        {
            gripper = false;
            useGraspPlugin = false;
        }
    }

    ros::init(argc, argv, Constants::init_node, ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // * set pub topic
    pubDesJstate = n.advertise<std_msgs::Float64MultiArray>(Constants::pubDesJstate, 10);
    if (realRobot)
    {
        pubTrajJstate = n.advertise<trajectory_msgs::JointTrajectory>(Constants::pubtrajJstateRealRobot, 10);
    }
    else
    {
        pubTrajJstate = n.advertise<trajectory_msgs::JointTrajectory>(Constants::pubTrajJstate, 10);
    }

    markerPub = n.advertise<visualization_msgs::MarkerArray>(Constants::markerPub, 1);
    markerArray = visualization_msgs::MarkerArray();

    //* set sub topic
    subJstate = n.subscribe(Constants::subJstate, 1, &JointStatePublisher::receiveJstate, this);

    // * set service
    attachSrv = n.serviceClient<gazebo_ros_link_attacher::Attach>(Constants::attachSrv);
    attachSrv.waitForExistence();
    detachSrv = n.serviceClient<gazebo_ros_link_attacher::Attach>(Constants::detachSrv);
    detachSrv.waitForExistence();
    setStaticSrv = n.serviceClient<gazebo_ros_link_attacher::SetStatic>(Constants::setStaticSrv);
    setStaticSrv.waitForExistence();
    std::cout << "----- link attacher service started -----" << std::endl;

    ros::service::waitForService(Constants::getWorldProperties);
    getWorldProperties = n.serviceClient<gazebo_msgs::GetWorldProperties>(Constants::getWorldProperties);
    ros::service::waitForService(Constants::getModelState);
    getModelState = n.serviceClient<gazebo_msgs::GetModelState>(Constants::getModelState);

    if (vision)
    {
        visionService = n.serviceClient<vision::vision>(Constants::visionService);
        visionService.waitForExistence();
        std::cout << "----- vision server started -----" << std::endl;
    }

    usleep(500000);
    jstate = q;
}

void JointStatePublisher::sendDesJState(std::vector<double> qDes, std::vector<double> qDesGripper)
{
    if (qDes.size() == 6)
    {
        if (gripper && !realRobot)
        {
            for (double x : qDesGripper)
            {
                qDes.push_back(x);
            }
        }
    }
    std_msgs::Float64MultiArray msg;
    msg.data = qDes;
    pubDesJstate.publish(msg);
}

void JointStatePublisher::sendDesTrajectory(const Trajectory &trajectory) const
{
    trajectory_msgs::JointTrajectory jt;

    for (int i = 0; i < 6; i++)
    {

        jt.joint_names.push_back(jointNames[i]);
    }
    if (gripper)
    {
        if (softGripper)
        {
            jt.joint_names.push_back(softGripperJointNames[0]);
            jt.joint_names.push_back(softGripperJointNames[1]);
        }
        else
        {
            jt.joint_names.push_back(gripperJointNames[0]);
            jt.joint_names.push_back(gripperJointNames[1]);
            jt.joint_names.push_back(gripperJointNames[2]);
        }
    }

    jt.header.stamp = ros::Time().now();

    if (trajectory.velocities.size() == 0)
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint jtp;
            jtp.positions = trajectory.positions[i];
            jtp.time_from_start = ros::Duration(trajectory.times[i] + 1);
            jt.points.push_back(jtp);
        }
    }
    else
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory_msgs::JointTrajectoryPoint jtp;
            jtp.positions = trajectory.positions[i];
            jtp.velocities = trajectory.velocities[i];
            jtp.time_from_start = ros::Duration(trajectory.times[i] + 1);
            jt.points.push_back(jtp);
        }
    }

    pubTrajJstate.publish(jt);
}

void JointStatePublisher::moveTo(const Vector3 &finalP, const Matrix3 &finalRotm, double gripperPos, const bool &waitForEnd, const CurveType &curveType, const double &vel, const bool &useIK)
{
    Trajectory trajectory;
    if (useIK)
    {
        // use IK trajectory
    }
    else
    {
        trajectory = Kinematic::differentialKinematic(jstate, finalP + (Vector3() << -0.5, -0.35, 0).finished(), finalRotm, curveType, vel);
    }
    trajectory.velocities.clear();

    std::vector<double> finalJstate = trajectory.positions.back();

    if (finalJstate[5] > 2 * M_PI)
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i][5] -= 2 * M_PI;
        }
    }
    else if (finalJstate[5] < -2 * M_PI)
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i][5] += 2 * M_PI;
        }
    }

    if (!realRobot)
    {
#ifdef SOFT_GRIPPER
        Vector2 finalQGripper = mapGripperJointState(gripperPos);
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i].push_back(finalQGripper(0));
            trajectory.positions[i].push_back(finalQGripper(1));
        }
#endif
#ifndef SOFT_GRIPPER
        Vector3 finalQGripper = mapGripperJointState(gripperPos);
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i].push_back(finalQGripper(0));
            trajectory.positions[i].push_back(finalQGripper(1));
            trajectory.positions[i].push_back(finalQGripper(2));
        }
#endif
    }

    for (int i = 0; i < trajectory.times.size(); i++)
    {
        trajectory.times[i] += 0.1;
    }
    usleep(1000000);
    sendDesTrajectory(trajectory);
    jstate = (Vector6() << finalJstate[0], finalJstate[1], finalJstate[2], finalJstate[3], finalJstate[4], finalJstate[5]).finished();

    if (waitForEnd)
    {
        while ((jstate - q).norm() > 0.005)
            ;
    }
}

void JointStatePublisher::pickAndPlaceBlock(const Vector3 &finalP, RPY finalRpy, const double &zOffset, const bool &attachToTable)
{
    std::pair<double,double> gripperPos = getGripPositions();
    block.computeApproachAndLandPose(finalP(0),finalP(1),finalRpy,zOffset);

    std::cout<<"------- moving to block --------"<<std::endl;
    moveTo(block.getApproachPos()+(Vector3()<<0,0,0.15).finished(),block.getApproachRotm(),gripperPos.first);
    moveTo(block.getApproachPos(),block.getApproachRotm(),gripperPos.first,true,CurveType::LINE,0.5);

    std::cout<<"------- gripping --------"<<std::endl;
    if(gripper)
    {
        gripping(gripperPos.second);
    }

    std::cout<<"------- moving upwards --------"<<std::endl;
    moveTo(block.getApproachPos()+(Vector3()<<0,0,0.15).finished(),block.getApproachRotm(),gripperPos.second,false,CurveType::LINE);

    std::cout<<"------- moving above final position --------"<<std::endl;
    moveTo(block.getLandPos()+(Vector3()<<0,0,0.10).finished(),block.getLandRotm(),gripperPos.second);

    std::cout<<"------- moving to final position --------"<<std::endl;
    moveTo(block.getLandPos(),block.getLandRotm(),gripperPos.second,true,CurveType::LINE,0.3);

    std::cout<<"------- un-gripping --------"<<std::endl;
    if(gripper)
    {
        ungripping(gripperPos.first, attachToTable);
    }

    block.update();

    std::cout<<"------- moving upwards --------"<<std::endl;
    moveTo(block.getLandPos()+(Vector3()<<0,0,0.15).finished(),block.getLandRotm(),gripperPos.first,false, CurveType::LINE);

    if(attachToTable)
    {
        std::cout<<"------- setting block static --------"<<std::endl;
        gazebo_ros_link_attacher::SetStatic set;
        set.request.model_name = block.getName();
        set.request.link_name = "link";
        set.request.set_static = true;
        setStaticSrv.call(set);
    }
}

void JointStatePublisher::receiveJstate(const sensor_msgs::JointState &state)
{
    q.setZero();
    qGripper.setZero();

    for (int i = 0; i < state.name.size(); i++)
    {
        for (int j = 0; j < 6; j++)
        {
            if (jointNames[j] == state.name.at(i))
            {
                q(j) = state.position.at(i);
            }
        }
        if (gripper)
        {
            if (softGripper)
            {
                for (int j = 0; j < 2; j++)
                {
                    if (softGripperJointNames[j] == state.name.at(i))
                    {
                        qGripper(j) = state.position.at(i);
                    }
                }
            }
            else
            {
                for (int j = 0; j < 3; j++)
                {
                    if (gripperJointNames[j] == state.name.at(i))
                    {
                        qGripper(j) = state.position.at(i);
                    }
                }
            }
        }
    }
}

std::pair<int, int> JointStatePublisher::getGripPositions()
{
    std::pair<int, int> class2gripsize;
    if (block.getConfiguration() == BlockConfiguration::SIDE)
    {
        switch (block.getClass())
        {
        case X1_Y1_Z2:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y2_Z1:
            class2gripsize = std::pair<int, int>(70, 72);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(95, 72);
            break;
        case X1_Y2_Z2_CHAMFER:
            class2gripsize = std::pair<int, int>(95, 72);
            break;
        case X1_Y2_Z2_TWINFILLET:
            class2gripsize = std::pair<int, int>(95, 73);
            break;
        case X1_Y3_Z2:
            class2gripsize = std::pair<int, int>(95, 70);
            break;
        case X1_Y3_Z2_FILLET:
            class2gripsize = std::pair<int, int>(95, 67);
            break;
        case X1_Y4_Z1:
            class2gripsize = std::pair<int, int>(70, 45);
            break;
        case X1_Y4_Z2:
            class2gripsize = std::pair<int, int>(95, 70);
            break;
        case X2_Y2_Z2:
            class2gripsize = std::pair<int, int>(95, 67);
            break;
        case X2_Y2_Z2_FILLET:
            class2gripsize = std::pair<int, int>(95, 67);
            break;
        }
    }
    else if (block.getConfiguration() == BlockConfiguration::DOWN)
    {
        switch (block.getClass())
        {
        case X1_Y1_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z1:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z2_CHAMFER:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z2_TWINFILLET:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y3_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y3_Z2_FILLET:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y4_Z1:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y4_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X2_Y2_Z2:
            class2gripsize = std::pair<int, int>(100, 79);
            break;
        case X2_Y2_Z2_FILLET:
            class2gripsize = std::pair<int, int>(100, 79);
            break;
        }
    }
    else if (block.getConfiguration() == BlockConfiguration::UP)
    {
        switch (block.getClass())
        {
        case X1_Y1_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z1:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(70, 36);
            break;
        case X1_Y2_Z2_CHAMFER:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y2_Z2_TWINFILLET:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y3_Z2:
            class2gripsize = std::pair<int, int>(70, 36);
            break;
        case X1_Y3_Z2_FILLET:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y4_Z1:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X1_Y4_Z2:
            class2gripsize = std::pair<int, int>(70, 35);
            break;
        case X2_Y2_Z2:
            class2gripsize = std::pair<int, int>(100, 90);
            break;
        case X2_Y2_Z2_FILLET:
            class2gripsize = std::pair<int, int>(100, 78);
            break;
        }
    }
    else
    {
        switch (block.getClass())
        {
        case X1_Y1_Z2:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y2_Z1:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y2_Z2_CHAMFER:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y2_Z2_TWINFILLET:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y3_Z2:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y3_Z2_FILLET:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y4_Z1:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X1_Y4_Z2:
            class2gripsize = std::pair<int, int>(70, 34);
            break;
        case X2_Y2_Z2:
            class2gripsize = std::pair<int, int>(100, 73);
            break;
        case X2_Y2_Z2_FILLET:
            class2gripsize = std::pair<int, int>(100, 72);
            break;
        }
    }

    return class2gripsize;
}

void JointStatePublisher::ungripping(const double &gripperPos, const bool &attachToTable)
{
    std::vector<double> finalQ;

#ifdef SOFT_GRIPPER
    Vector2 desiredQGripper = mapGripperJointState(gripperPos);
    finalQ = {jstate(0), jstate(1), jstate(2), jstate(3), jstate(4), jstate(5), desiredQGripper(0), desiredQGripper(1)};
#endif
#ifndef SOFT_GRIPPER
    Vector3 desiredQGripper = mapGripperJointState(gripperPos);
    finalQ = {jstate(0), jstate(1), jstate(2), jstate(3), jstate(4), jstate(5), desiredQGripper(0), desiredQGripper(1), desiredQGripper(2)};
#endif

    Trajectory trajectory;
    trajectory.positions.push_back(finalQ);
    trajectory.times.push_back(0.1);
    usleep(500000);

    if (attachToTable)
    {
        std::cout << "attaching to ground plane" << std::endl;
        gazebo_ros_link_attacher::Attach attach;
        attach.request.model_name_1 = "my_ground_plane";
        attach.request.link_name_1 = "link";
        attach.request.model_name_2 = block.getName();
        attach.request.model_name_2 = "link";

        attachSrv.call(attach);
    }
    std::cout << "detaching block" << std::endl;
    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1 = "ur5";
    attach.request.link_name_1 = "wrist_3_link";
    attach.request.model_name_2 = block.getName();
    attach.request.model_name_2 = "link";
    attachSrv.call(attach);

    std::cout << "waiting to reach gripper position" << std::endl;

    while ((qGripper - desiredQGripper).norm() > 0.1)
        ;
}

void JointStatePublisher::gripping(const double &gripperPos)
{
    std::vector<double> finalQ;
    std::cout << "attaching block" << std::endl;
#ifdef SOFT_GRIPPER
    Vector2 desiredQGripper = mapGripperJointState(gripperPos + 2);
    finalQ = {jstate(0), jstate(1), jstate(2), jstate(3), jstate(4), jstate(5), desiredQGripper(0), desiredQGripper(1)};
#endif
#ifndef SOFT_GRIPPER
    Vector3 desiredQGripper = mapGripperJointState(gripperPos + 2);
    finalQ = {jstate(0), jstate(1), jstate(2), jstate(3), jstate(4), jstate(5), desiredQGripper(0), desiredQGripper(1), desiredQGripper(2)};
#endif

    Trajectory trajectory;
    trajectory.positions.push_back(finalQ);
    trajectory.times.push_back(0.1);
    usleep(500000);

    while ((qGripper - desiredQGripper).norm() > 0.005)
        ;

    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1 = "ur5";
    attach.request.link_name_1 = "wrist_3_link";
    attach.request.model_name_2 = block.getName();
    attach.request.model_name_2 = "link";
    attachSrv.call(attach);
    usleep(300000);
}
