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
    std::cout << "!! moveTo: " << finalP[0] << "," << finalP[1] << "," << finalP[2] << " !! " << std::endl;
    std::cout << "starting jstate: " << jstate(0) << "," << jstate(1) << "," << jstate(2) << "," << jstate(3) << "," << jstate(4) << "," << jstate(5) << "\n";
    Matrix4 actual_pose = Kinematic::directKinematic(jstate);
    CurveType curve;
    double dist = sqrt(pow(actual_pose(0, 3) - finalP(0) + 0.5, 2) + pow(actual_pose(1, 3) - finalP(1) + 0.35, 2));
    if (dist > 0.03)
        curve = CurveType::BEZIER;
    else
        curve = CurveType::LINE;

    Trajectory trajectory;
    if (useIK)
    {
        // use IK trajectory
    }
    else
    {
        trajectory = Kinematic::differentialKinematic(jstate, finalP + (Vector3() << -0.5, -0.35, 0).finished(), finalRotm, curve, vel);
    }
    trajectory.velocities.clear();

    std::vector<double> finalJstate = trajectory.positions.back();

    if (finalJstate[5] > 2 * M_PI)
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i][5] += -2 * M_PI;
        }
    }
    else if (finalJstate[5] < -2 * M_PI)
    {
        for (int i = 0; i < trajectory.positions.size(); i++)
        {
            trajectory.positions[i][5] += 2 * M_PI;
        }
    }
    finalJstate = trajectory.positions.back();

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
    usleep(1750000);
    sendDesTrajectory(trajectory);
    jstate = (Vector6() << finalJstate[0], finalJstate[1], finalJstate[2], finalJstate[3], finalJstate[4], finalJstate[5]).finished();
    if (waitForEnd)
    {
        while ((jstate - q).norm() > 0.005)
            ros::spinOnce();
    }
    std::cout << std::endl;
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
        gripping(gripperPos.second);

    std::cout << "------- moving upwards --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE);

    std::cout << "------- moving above final position --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.10).finished(), block.getLandRotm(), gripperPos.second);

    std::cout << "------- moving to final position --------" << std::endl;
    moveTo(block.getLandPos(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.3);

    std::cout << "------- un-gripping --------" << std::endl;
    if (gripper)
        ungripping(gripperPos.first, attachToTable);

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

void JointStatePublisher::rotateBlock(const RPY &newBlockRpy, Cartesian newBlockPos)
{
    std::cout << "!! rotateBlock !!" << std::endl;

    std::pair<double, double> gripperPos = getGripPositions();
    block.computeApproachAndLandPose(newBlockPos.x, newBlockPos.y, newBlockRpy);

    std::cout << "------- moving to block --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.first, true);
    moveTo(block.getApproachPos(), block.getApproachRotm(), gripperPos.first, true, CurveType::LINE, 0.5);

    std::cout << "------- gripping --------" << std::endl;
    if (gripper)
        gripping(gripperPos.second);

    std::cout << "------- rotating block --------" << std::endl;
    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.05).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE, 0.5);
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.05).finished(), block.getLandRotm(), gripperPos.second, false, CurveType::LINE);
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.002).finished(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.5);

    std::cout << "------- ungripping --------" << std::endl;
    if (gripper)
        ungripping(gripperPos.first, false);

    block.autoUpdate();
    block.print();

    std::cout << "------- moving up --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.1).finished(), block.getLandRotm(), gripperPos.first, false, CurveType::LINE);

    std::cout << std::endl;
}

void JointStatePublisher::setupBlockForRotation()
{
    std::cout << "!! setupBlockForRotation !!" << std::endl;

    RPY newBlockRpy;
    if (block.getDistanceFromShoulder() < 0.3)
    {
        if (block.getConfiguration() == BlockConfiguration::UP)
            newBlockRpy = RPY(M_PI / 2, 0.0, M_PI);

        else if (block.getConfiguration() == BlockConfiguration::SIDE)
            newBlockRpy = RPY(0.0, M_PI / 2, M_PI);

        else if (block.getConfiguration() == BlockConfiguration::DOWN)
            newBlockRpy = RPY(0.0, M_PI, M_PI / 2);

        Cartesian newBlockPos = findFreeSpot();
        rotateBlock(newBlockRpy, newBlockPos);
    }
    else
    {
        if (block.getQuadrant() == 1)
        {
            if (block.getRpyY() < M_PI / 2 - 0.1 || block.getRpyY() > 3 / 2 * M_PI + 0.1)
            {
                std::cout << "------- setup block --------" << std::endl;
                if (block.getConfiguration() == BlockConfiguration::UP)
                    rotateBlock(RPY(M_PI / 2, 0, M_PI), block.getPosition());

                else if (block.getConfiguration() == BlockConfiguration::SIDE)
                {
                    if ((block.getRpyY() < -0.1 || block.getRpyY() > M_PI + 0.1) && block.getClass() == BlockClass::X1_Y2_Z2_CHAMFER)
                        rotateBlock(RPY(0, M_PI / 2, M_PI / 2), block.getPosition());

                    else if (block.getClass() == BlockClass::X1_Y2_Z2_TWINFILLET || block.getClass() == BlockClass::X1_Y3_Z2_FILLET || block.getClass() == BlockClass::X2_Y2_Z2_FILLET)
                        rotateBlock(RPY(0, M_PI / 2, M_PI), block.getPosition());
                }
                else if (block.getConfiguration() == BlockConfiguration::DOWN)
                {
                    if (block.getClass() == BlockClass::X1_Y1_Z2 || block.getClass() == BlockClass::X2_Y2_Z2)
                        block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));

                    else if (block.getClass() == BlockClass::X1_Y2_Z1 || block.getClass() == BlockClass::X1_Y2_Z2 | block.getClass() == BlockClass::X1_Y3_Z2 | block.getClass() == BlockClass::X1_Y4_Z1 | block.getClass() == BlockClass::X1_Y4_Z2)
                        block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));

                    else
                        rotateBlock(RPY(0, M_PI, M_PI), block.getPosition());
                }
            }
        }
        else if (block.getQuadrant() == 4)
        {
            if (block.getRpyY() < M_PI / 2 - 0.01 || block.getRpyY() > 3 * M_PI / 2 + 0.1)
            {
                std::cout << "------- setup block --------" << std::endl;
                if (block.getConfiguration() == BlockConfiguration::UP)
                    rotateBlock(RPY(M_PI / 2, 0, M_PI), block.getPosition());

                else if (block.getConfiguration() == BlockConfiguration::SIDE)
                {
                    if ((block.getRpyY() < -0.1 || block.getRpyY() > M_PI / 2 + 0.1) && block.getClass() == BlockClass::X1_Y2_Z2_CHAMFER)
                        rotateBlock(RPY(0, M_PI / 2, M_PI / 2), block.getPosition());

                    else if (block.getClass() == BlockClass::X1_Y2_Z2_TWINFILLET || block.getClass() == BlockClass::X1_Y3_Z2_FILLET || block.getClass() == BlockClass::X2_Y2_Z2_FILLET)
                        rotateBlock(RPY(0, M_PI / 2, M_PI), block.getPosition());
                }
                else if (block.getConfiguration() == BlockConfiguration::DOWN)
                {
                    if (block.getClass() == BlockClass::X1_Y1_Z2 || block.getClass() == BlockClass::X2_Y2_Z2)
                        block.setRpy(RPY(0, M_PI, M_PI / 2 + block.getRpyY()));

                    else if (block.getClass() == BlockClass::X1_Y2_Z1 || block.getClass() == BlockClass::X1_Y2_Z2 | block.getClass() == BlockClass::X1_Y3_Z2 | block.getClass() == BlockClass::X1_Y4_Z1 | block.getClass() == BlockClass::X1_Y4_Z2)
                        block.setRpy(RPY(0, M_PI, M_PI + block.getRpyY()));

                    else
                        rotateBlock(RPY(0, M_PI, 0), block.getPosition());
                }
            }
        }
    }
    std::cout << std::endl;
}

void JointStatePublisher::rotateBlockStandardPosition(double xLandPose, double yLandPose, RPY finalRpy)
{
    std::cout << "!! rotateBlockStandardPosition !!" << std::endl;

    std::pair<double, double> gripperPos = getGripPositions();
    setupBlockForRotation();

    std::cout << "------- moving to block at 45° --------" << std::endl;
    if (xLandPose == -1 && yLandPose == -1 && block.getDistanceFromShoulder() < 0.35)
    {
        Cartesian free_spot = findFreeSpot();
        xLandPose = free_spot.x;
        yLandPose = free_spot.y;
    }
    block.computeApproachAndLandPose(xLandPose, yLandPose, finalRpy, 0.0, 45);

    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.first);
    moveTo(block.getApproachPos(), block.getApproachRotm(), gripperPos.first, true, CurveType::LINE, 0.3);

    std::cout << "------- gripping --------" << std::endl;
    gripping(gripperPos.second);

    moveTo(block.getApproachPos() + (Vector3() << 0, 0, 0.15).finished(), block.getApproachRotm(), gripperPos.second, false, CurveType::LINE, 0.5);

    std::cout << "------- moving block to landing pos --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.15).finished(), block.getLandRotm(), gripperPos.second, false);
    moveTo(block.getLandPos(), block.getLandRotm(), gripperPos.second, true, CurveType::LINE, 0.15);

    // usleep(500000);

    std::cout << "------- ungripping --------" << std::endl;
    ungripping(gripperPos.first, false);

    block.autoUpdate();
    block.print();

    std::cout << "------- moving up --------" << std::endl;
    moveTo(block.getLandPos() + (Vector3() << 0, 0, 0.22).finished(), block.getLandRotm(), gripperPos.first, false, CurveType::LINE, 0.5);
    std::cout << std::endl;
}

double JointStatePublisher::checkCollision(double x, double y)
{
    for (auto b : presentBlocks)
    {
        if (b.getName() != block.getName())
        {
            double dist = sqrt((x - b.getPosition().x) * (x - b.getPosition().x) + (y - b.getPosition().y) * (y - b.getPosition().y));
            if (dist < (block.getRadius() + b.getRadius() + 0.07))
            {
                return true;
            }
        }
    }
    return false;
}

Cartesian JointStatePublisher::findFreeSpot(double castleXmin, double castleYmin)
{
    std::cout << "!! findFreeSpot !!" << std::endl;

    double dist, x, y;
    int xmin, xmax, ymin, ymax;

    if (block.getQuadrant() <= 2)
    {
        xmin = 50;
        xmax = 93;
        ymin = 30;
        ymax = castleYmin - 0.1;
    }
    else
    {
        xmin = 7;
        xmax = 50;
        ymin = 30;
        ymax = 73;
    }

    do
    {
        x = ((double)(rand() % (xmax - xmin) + xmin) / 100); // x in [7, 50] se q=(1,2)       OPPURE [50, 93] se q=(3,4)
        y = ((double)(rand() % (ymax - ymin) + ymin) / 100); // y in [30, area castello - 10] OPPURE [30, 73]
        dist = sqrt((x - 0.5) * (x - 0.5) + (y - 0.35) * (y - 0.35));
    } while (dist < 0.4 || (castleXmin < x && x < 1 && castleYmin < y && y < 0.8) || checkCollision(x, y));

    std::cout << std::endl;
    return Cartesian(x, y, 0.0);
}

void JointStatePublisher::homingProcedure(const Vector6 &final_q)
{
    std::cout << "------- homing procedure --------" << std::endl;

    updateJstate();
    if ((jstate - final_q).norm() < 0.005) // if the robot is already in the final configuration, do not create the trajectory
        return;

    Trajectory trajectory = Kinematic::jcubic(jstate, final_q);
    // std::cout << trajectory << std::endl;

    std::cout << "Jcubic fin" << std::endl;

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

    usleep(1500000);
    sendDesTrajectory(trajectory);

    jstate = (Vector6() << final_q[0], final_q[1], final_q[2], final_q[3], final_q[4], final_q[5]).finished();
    while ((jstate - q).norm() > 0.005)
        ros::spinOnce();

    std::cout << "position reached" << std::endl;
}

void JointStatePublisher::multipleBlocks()
{
    RPY finalTowerRpy(0.0, 0.0, M_PI / 2);
    std::vector<double> height(11,0.0);

    for (int i = 0; i < presentBlocks.size(); i++)
    {
        block = presentBlocks[i];
        Cartesian finalTowerPos = finalPosFromClass(block.getClass());
        block.update();

        block.print();

        while (block.getConfiguration() != BlockConfiguration::REGULAR)
        {
            rotateBlockStandardPosition();
        }

        std::cout << "move block to final position" << std::endl;
        Cartesian finalBlockPos = finalTowerPos + Cartesian(0.0, 0.0, block.getPosition().z + height[block.getClass()]);
        std::cout << "final blockPos " << finalBlockPos.x << " " << finalBlockPos.y << std::endl;
        pickAndPlaceBlock(finalBlockPos.toVector(), finalTowerRpy, height[block.getClass()], true);
        height[block.getClass()] += BlockDimension.at(block.getClass()).z - 0.0195;
    }

    std::cout << "multiple blocks task completed" << std::endl;
}

void JointStatePublisher::castle()
{
    /* Read Json region*/
    std::ifstream ifs(Constants::jsonOutput);
    Json::Reader reader;
    Json::Value json;
    reader.parse(ifs, json);
    std::cout << "json loaded" << std::endl;
    ifs.close();
    /*End Read Json region*/

    usleep(500000);

    int n = json["size"].asInt();

    double xMax = 0.99;
    double yMax = 0.79;
    double xMin = 1.0;
    double yMin = 1.0;

    for (int i = 1; i <= n; i++)
    {
        BlockClass blockClass = stringToBlockClass(json[std::to_string(i)]["class"].asString());
        Cartesian dim = BlockDimension.at(blockClass);
        double yaw = getYaw(json[std::to_string(i)]["r"].asDouble());
        double xDes = json[std::to_string(i)]["x"].asDouble() / 100000.0;
        double yDes = json[std::to_string(i)]["y"].asDouble() / 100000.0;
        double x = 0.98 - xDes - dim.x * cos(yaw) / 2 - dim.y * sin(yaw) / 2;
        double y = 0.78 - yDes - dim.y * cos(yaw) / 2 - dim.x * sin(yaw) / 2;

        xMin = std::min(x, xMin);
        yMin = std::min(y, yMin);
    }
    for (auto b : presentBlocks)
    {
        if (xMin < b.getPosition().x && b.getPosition().x < 1.0 && yMin < b.getPosition().y && b.getPosition().y < 0.8)
        {
            block = b;
            block.update();
            Cartesian c = findFreeSpot(xMin, yMin);
            rotateBlockStandardPosition(c.x, c.y, block.getFinalRpy());
        }
    }
    for (int i = 1; i <= n; i++)
    {
        BlockClass blockClass = stringToBlockClass(json[std::to_string(i)]["class"].asString());
        int index = -1;
        for (int j = 0; j < presentBlocks.size(); j++)
        {
            std::cout << presentBlocks[j].getClass() << " - " << blockClass << " - " << presentBlocks[j].getProcessed() << std::endl;
            if (presentBlocks[j].getClass() == blockClass && !presentBlocks[j].getProcessed())
            {
                index = j;
                break;
            }
        }
        if (index == -1)
            continue;

        block = presentBlocks[index];
        block.update();

        block.print();

        double yaw = getYaw(json[std::to_string(i)]["r"].asDouble());
        double xDes = json[std::to_string(i)]["x"].asDouble() / 100000.0;
        double yDes = json[std::to_string(i)]["y"].asDouble() / 100000.0;
        double zDes = json[std::to_string(i)]["z"].asDouble() / 100000.0;

        while (block.getConfiguration() != BlockConfiguration::REGULAR)
        {
            rotateBlockStandardPosition();
        }

        Vector3 finalPos;
        finalPos << xMax - xDes, yMax - yDes, 0;
        RPY finalRpy(0, 0, yaw);

        pickAndPlaceBlock(finalPos, finalRpy, zDes);
        presentBlocks[index].setProcessed(true);
    }

    std::cout << "Castle process completed" << std::endl;
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
        return 0.0;
    else if (rot == 1)
        return M_PI / 2;
    else if (rot == 2)
        return M_PI;
    else
        return -M_PI / 2;
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

void JointStatePublisher::setBlock(Block b)
{
    block = b;
}

void JointStatePublisher::updateJstate()
{
    ros::spinOnce();
    usleep(500000);
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
            class2gripsize = std::pair<int, int>(85, 73);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(95, 73);
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
            class2gripsize = std::pair<int, int>(100, 83);
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
            class2gripsize = std::pair<int, int>(100, 85);
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
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y2_Z1:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y2_Z2:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y2_Z2_CHAMFER:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y2_Z2_TWINFILLET:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y3_Z2:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y3_Z2_FILLET:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y4_Z1:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X1_Y4_Z2:
            class2gripsize = std::pair<int, int>(70, 33);
            break;
        case X2_Y2_Z2:
            class2gripsize = std::pair<int, int>(100, 75);
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
    trajectory.times.push_back(1.0);
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
    trajectory.times.push_back(2.0);
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
