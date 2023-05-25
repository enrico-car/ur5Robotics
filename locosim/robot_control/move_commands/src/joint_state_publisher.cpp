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
        while ((jstate - q).norm() > 0.005){
            ros::spinOnce();
        }
    }
}

void JointStatePublisher::pickAndPlaceBlock(const Vector3 &finalP, RPY finalRpy, const double &zOffset, const bool &attachToTable)
{
    std::pair<double, double> gripperPos = getGripPositions();
    block.computeApproachAndLandPose(finalP(0), finalP(1), finalRpy, zOffset);

    std::cout << "------- moving to block --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.first);
    moveTo(block.getApproachPos(), block.getApproachRotm(), gripperPos.first, true, CurveType::LINE, 0.5);

    std::cout << "------- gripping --------" << std::endl;
    if (gripper)
    {
        gripping(gripperPos.second);
    }

    std::cout << "------- moving upwards --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE);

    std::cout << "------- moving above final position --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.10).finished(), block.getLandRotm(), gripperPos.second);

    std::cout << "------- moving to final position --------" << std::endl;
    moveTo(block.getLandPos(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.3);

    std::cout << "------- un-gripping --------" << std::endl;
    if (gripper)
    {
        ungripping(gripperPos.first, attachToTable);
    }

    block.update();

    std::cout << "------- moving upwards --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.15).finished(), block.getLandRotm(), gripperPos.first, false, CurveType::LINE);

    if (attachToTable)
    {
        std::cout << "------- setting block static --------" << std::endl;
        gazebo_ros_link_attacher::SetStatic set;
        set.request.model_name = block.getName();
        set.request.link_name = "link";
        set.request.set_static = true;
        setStaticSrv.call(set);
    }
}

void JointStatePublisher::rotateBlock(const RPY &newBlockRpy)
{
    std::pair<double, double> gripperPos = getGripPositions();
    block.computeApproachAndLandPose(block.getPosition().x, block.getPosition().y, newBlockRpy);

    std::cout << "------- moving to block --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.first, true);
    moveTo(block.getApproachPos(), block.getApproachRotm(), gripperPos.first, true, CurveType::LINE, 0.5);

    std::cout << "------- gripping --------" << std::endl;
    if (gripper)
    {
        gripping(gripperPos.second);
    }

    std::cout << "------- rotating block --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.05).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE, 0.5);
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.05).finished(), block.getLandRotm(), gripperPos.second, false, CurveType::LINE);
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.002).finished(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.5);

    std::cout << "------- ungripping --------" << std::endl;
    if (gripper)
    {
        ungripping(gripperPos.first, false);
    }

    block.setRpyY(newBlockRpy.y);
    block.update();

    std::cout << "------- moving up --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.1).finished(), block.getLandRotm(), gripperPos.first, false, CurveType::LINE);
}

void JointStatePublisher::setupBlockForRotation()
{
    if (block.getQuadrant() == 1)
    {
        if (block.getRpyY() < 3 / 4 * M_PI || block.getRpyY() > 3 / 2 * M_PI)
        {
            std::cout << "------- setup block --------" << std::endl;
            if (block.getConfiguration() == BlockConfiguration::UP)
            {
                rotateBlock(RPY(M_PI / 2, 0, M_PI));
            }
            else if (block.getConfiguration() == BlockConfiguration::SIDE)
            {
                rotateBlock(RPY(0, M_PI / 2, M_PI));
            }
            else
            {
                if (block.getClass() == BlockClass::X1_Y1_Z2 || block.getClass() == BlockClass::X2_Y2_Z2)
                {
                    block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));
                }
                else if (block.getClass() == BlockClass::X1_Y2_Z1 || block.getClass() == BlockClass::X1_Y2_Z2 | block.getClass() == BlockClass::X1_Y3_Z2 | block.getClass() == BlockClass::X1_Y4_Z1 | block.getClass() == BlockClass::X1_Y4_Z2)
                {
                    block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));
                }
                else
                {
                    rotateBlock(RPY(0, M_PI, M_PI));
                }
            }
        }
    }
    else if (block.getQuadrant() == 4)
    {
        if (block.getRpyY() < M_PI / 2 - 0.01 || block.getRpyY() > 5 / 4 * M_PI + 0.01)
        {
            std::cout << "------- setup block --------" << std::endl;
            if (block.getClass() == BlockClass::X1_Y1_Z2)
            {
                rotateBlock(RPY(M_PI / 2, 0, M_PI));
            }
            else if (block.getClass() == BlockClass::X1_Y2_Z1)
            {
                rotateBlock(RPY(0, M_PI / 2, M_PI));
            }
            else
            {
                if (block.getClass() == BlockClass::X1_Y1_Z2 || block.getClass() == BlockClass::X2_Y2_Z2)
                {
                    block.setRpy(RPY(0, M_PI, M_PI / 2 + block.getRpyY()));
                }
                else if (block.getClass() == BlockClass::X1_Y2_Z1 || block.getClass() == BlockClass::X1_Y2_Z2 | block.getClass() == BlockClass::X1_Y3_Z2 | block.getClass() == BlockClass::X1_Y4_Z1 | block.getClass() == BlockClass::X1_Y4_Z2)
                {
                    block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));
                }
                else
                {
                    rotateBlock(RPY(0, M_PI, 0));
                }
            }
        }
    }
}

void JointStatePublisher::rotateBlockStandardPosition(double xLandPose, double yLandPose, RPY finalRpy)
{
    std::pair<double, double> gripperPos = getGripPositions();
    setupBlockForRotation();
    std::cout << "------- moving to block at 45° --------" << std::endl;
    block.computeApproachAndLandPose(xLandPose, yLandPose, finalRpy, 0.0, 45);

    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.first);
    moveTo(block.getApproachPos(), block.getApproachRotm(), gripperPos.first, true, CurveType::LINE, 0.3);
    std::cout << "------- gripping --------" << std::endl;
    gripping(gripperPos.second);

    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE, 0.5);

    std::cout << "------- moving block to landing pos --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.1).finished(), block.getLandRotm(), gripperPos.second, false, CurveType::BEZIER, 0.5);
    moveTo(block.getLandPos(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.15);

    usleep(500000);

    std::cout << "------- ungripping --------" << std::endl;
    ungripping(gripperPos.first, false);

    block.autoUpdate();

    std::cout << "------- moving up --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.22).finished(), block.getLandRotm(), gripperPos.first, false, CurveType::LINE, 0.5);
}

void JointStatePublisher::homingProcedure()
{
    std::cout << "------- homing procedure --------" << std::endl;

    Trajectory trajectory = Kinematic::jcubic(q, q0);

    std::cout << "Jqubic fin" << std::endl;

    if (!realRobot)
    {
        if (trajectory.velocities.size() == 0)
        {
            for (int i = 0; i < trajectory.positions.size(); i++)
            {
                trajectory.positions[i].push_back(qGripper(0));
                trajectory.positions[i].push_back(qGripper(1));
            }
        }
        else
        {
            for (int i = 0; i < trajectory.positions.size(); i++)
            {
                trajectory.positions[i].push_back(qGripper(0));
                trajectory.positions[i].push_back(qGripper(1));
                trajectory.velocities[i].push_back(qGripper(0));
                trajectory.velocities[i].push_back(qGripper(1));
            }
        }
    }

    for (int i = 0; i < trajectory.times.size(); i++)
    {
        trajectory.times[i] += 0.1;
    }

    std::cout << "send" << std::endl;
    usleep(1000000);
    sendDesTrajectory(trajectory);

    std::cout << "wait" << std::endl;
    while ((jstate - q).norm() > 0.005)
        ;
}

void JointStatePublisher::multipleBlocks(Detected d)
{
    std::vector<Block> blocks;
    for (int i = 0; i < d.n; i++)
    {
        Cartesian c(d.xCenter[i], d.yCenter[i], d.zCenter[i]);
        RPY r(d.roll[i], d.pitch[i], d.yaw[i]);
        std::string name = "";
        Block b(name, d.blockClass[i], c, r);
        blocks.push_back(b);
    }

    double height = 0.0;
    Cartesian finalTowerPos(0.8, 0.7, 0.0);
    RPY finalTowerRpy(0.0, 0.0, 0.0);

    for (int i = 0; i < d.n; i++)
    {
        Block current = blocks[i];

        // check if block is not in the standard position
        if (!(-0.01 < current.getRpy().r && current.getRpy().p < 0.01))
        {
            std::cout << "move block to stand position" << std::endl;
            RPY landRpy(M_PI / 2, 0.0, 0.0);
            rotateBlockStandardPosition(0.5, 0.75, landRpy);
        }

        if (!(-0.01 < current.getRpy().r && current.getRpy().r < 0.01))
        {
            std::cout << "move block to standard position" << std::endl;
            RPY landRpy(0.0, 0.0, M_PI / 2);
            rotateBlockStandardPosition(0.5, 0.75, landRpy);
        }

        std::cout << "move block to final position" << std::endl;
        Cartesian finalBlockPos = finalTowerPos + Cartesian(0.0, 0.0, current.getPosition().z + height);

        pickAndPlaceBlock(finalBlockPos.toVector(), finalTowerRpy, height, true);
        height += BlockDimension.at(current.getClass()).z - 0.0195;
    }
    std::cout << "multiple blocks task completed" << std::endl;
}

void JointStatePublisher::castle(Json::Value json)
{
    double xMax = 0.98;
    double yMax = 0.78;

    // int n = json["size"].asInt();
    int n = 2;
    BlockClass bb[] = {BlockClass::X1_Y2_Z2_CHAMFER,BlockClass::X1_Y3_Z2_FILLET};
    int rr[] = {3,0};
    double xx[] = {0.06300,0.01575};
    double yy[] = {0.01575,0.04725};
    double zz[] = {0,0};



    for (int i = 0; i < n; i++)
    {
        // const char idKey[] = std::to_string(i);
        // const Json::Value obj = *json.find(idKey, idKey+std::strlen(idKey));
        // std::cout<<obj<<std::endl;s
        
        //std::cout<<json[index]["class"].asString()<<std::endl;
        //std::string classString = json[index]["class"].asString();
        // BlockClass blockClass = stringToBlockClass();
        BlockClass blockClass = bb[i];

        int index = -1;
        for (int j = 0; j < presentBlocks.size(); j++)
        {
            if (presentBlocks[j].getClass() == blockClass && !presentBlocks[j].getProcessed())
            {
                index = j;
                break;
            }
        }
        if (index == -1)
        {
            continue;
        }

        block = presentBlocks[index];
        block.update();

        block.print();

        // double yaw = getYaw(json[std::to_string(i)]["r"].asDouble());
        // double xDes = json[std::to_string(i)]["x"].asDouble();
        // double yDes = json[std::to_string(i)]["y"].asDouble();
        // double zDes = json[std::to_string(i)]["z"].asDouble();
        double yaw = getYaw(rr[i]);
        double xDes = xx[i];
        double yDes = yy[i];
        double zDes = zz[i];

        while (block.getConfiguration() != BlockConfiguration::REGULAR)
        {
            rotateBlockStandardPosition(block.getPosition().x, block.getPosition().y, block.getFinalRpy());
        }

        Vector3 finalPos;
        finalPos << xMax - xDes, yMax - yDes, 0;
        RPY finalRpy(0, 0, yaw);

        pickAndPlaceBlock(finalPos, finalRpy, zDes);
        presentBlocks[index].setProcessed(true);
    }
}

Json::Value JointStatePublisher::readJson()
{
    std::ifstream ifs(Constants::jsonOutput);
    Json::Reader reader;
    Json::Value obj;
    reader.parse(ifs, obj);

    ifs.close();

    return obj;
}

// TODO: quando mettiamo a posto il JAVAFX, scambiare i segni di 1 e 3
double JointStatePublisher::getYaw(int rot)
{
    if (rot == 0)
    {
        return 0.0;
    }
    else if (rot == 1)
    {
        return -M_PI / 2;
    }
    else if (rot == 2)
    {
        return M_PI;
    }
    else
    {
        return M_PI / 2;
    }
}

void JointStatePublisher::registerBlocks(const vision::vision &visionResult)
{
    for (int i = 0; i < visionResult.response.n_res; i++)
    {
        Cartesian c(visionResult.response.x_center[i], visionResult.response.y_center[i], 0);
        RPY r(visionResult.response.roll[i], visionResult.response.pitch[i], visionResult.response.yaw[i]);
        std::string name = "";
        Block b(name, (BlockClass)visionResult.response.classe[i], c, r);
        b.print();
        presentBlocks.push_back(b);
    }

    gazebo_msgs::ModelStates modelState;
    modelState = *(ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states").get());

    for (int i = 0; i < modelState.name.size(); i++)
    {
        if (modelState.name[i].find("brick") != std::string::npos)
        {
            for (int j = 0; j < presentBlocks.size(); j++)
            {
                if (presentBlocks[j].getPosition().x - 0.1 < modelState.pose[i].position.x &&
                    modelState.pose[i].position.x < presentBlocks[j].getPosition().x + 0.1 && presentBlocks[j].getPosition().y - 0.1 < modelState.pose[i].position.y &&
                    modelState.pose[i].position.y < presentBlocks[j].getPosition().y + 0.1)
                {
                    presentBlocks[j].setName(modelState.name[i]);
                }
            }
        }
    }
}

void JointStatePublisher::updateJstate()
{
    jstate = q;
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
    sendDesTrajectory(trajectory);

    if (attachToTable)
    {
        std::cout << "attaching to ground plane" << std::endl;
        gazebo_ros_link_attacher::Attach attach;
        attach.request.model_name_1 = "my_ground_plane";
        attach.request.link_name_1 = "link";
        attach.request.model_name_2 = block.getName();
        attach.request.link_name_2 = "link";

        attachSrv.call(attach);
    }
    std::cout << "detaching block" << std::endl;
    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1 = "ur5";
    attach.request.link_name_1 = "wrist_3_link";
    attach.request.model_name_2 = block.getName();
    attach.request.link_name_2 = "link";
    detachSrv.call(attach);

    std::cout << "waiting to reach gripper position" << std::endl;

    while ((qGripper - desiredQGripper).norm() > 0.1)
    {
        ros::spinOnce();
    }
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
    sendDesTrajectory(trajectory);

    while ((qGripper - desiredQGripper).norm() > 0.005)
    {
        ros::spinOnce();
    }

    gazebo_ros_link_attacher::Attach attach;
    attach.request.model_name_1 = "ur5";
    attach.request.link_name_1 = "wrist_3_link";
    attach.request.model_name_2 = block.getName();
    attach.request.link_name_2 = "link";
    attachSrv.call(attach);
    usleep(300000);
}
