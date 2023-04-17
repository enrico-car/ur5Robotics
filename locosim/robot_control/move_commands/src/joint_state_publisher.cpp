#include "joint_state_publisher.h"

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
        pubtrajJstate = n.advertise<trajectory_msgs::JointTrajectory>(Constants::pubtrajJstateRealRobot, 10);
    }
    else
    {
        pubtrajJstate = n.advertise<trajectory_msgs::JointTrajectory>(Constants::pubtrajJstate, 10);
    }
    //* set sub topic
    // grasp menager topic
    // self.grasp_event = ros.Subscriber('/grasp_event_republisher/grasp_events', GazeboGraspEvent, self.grasp_manager)
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
}

void JointStatePublisher::sendDesJState(std::vector<double> q_des)
{
    // if (q_des.size() < 6)
    // {
    //     std::cout << "Wrong Jstate!!!" << std::endl;
    // }
    // else if (q_des.size() == 6)
    // {
    //     // q_des.push_back(0.0);
    //     // q_des.push_back(0.0);
    // }
    // std_msgs::Float64MultiArray msg;
    // msg.data = q_des;
    // pub_des_joint_state.publish(msg);
}

void JointStatePublisher::sendDesTrajectory(const Trajectory &trajectory) const
{
    // trajectory_msgs::JointTrajectory jt;
    // jt.joint_names = Constants::joint_names;
    // jt.header.stamp = ros::Time().now();

    // trajectory_msgs::JointTrajectoryPoint jtp;

    // for (int i = 0; i < trajectory.positions.size(); i++)
    // {
    //     // std::cout << "position" << i << ":" << std::endl;
    //     // for (int j = 0; j < trajectory.positions[i].size(); j++)
    //     // {
    //     //     std::cout << trajectory.positions[i][j] << " " << std::endl;
    //     // }
    //     // std::cout << "velocities" << i << ":" << std::endl;
    //     // for (int j = 0; j < trajectory.velocities[i].size(); j++)
    //     // {
    //     //     std::cout << trajectory.velocities[i][j] << " " << std::endl;
    //     // }
    //     jtp.positions = trajectory.positions[i];
    //     jtp.velocities = trajectory.velocities[i];
    //     jtp.time_from_start = ros::Duration(trajectory.times[i]);
    //     jt.points.push_back(jtp);
    // }

    // pub_traj_jstate.publish(jt);
}

void JointStatePublisher::receiveJstate(const sensor_msgs::JointState &state)
{
    q.setZero();
    qGripper.setZero();
    q << state.position[0], state.position[1], state.position[2], state.position[3], state.position[4], state.position[5];
    if (softGripper)
    {
        qGripper << state.position[6], state.position[7];
    }
    else
    {
        qGripper << state.position[6], state.position[7], state.position[8];
    }
}