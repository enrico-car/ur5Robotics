#!/usr/bin/env python
import rospy
import rospy as ros
import numpy as np
import params as conf
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from base_controllers.components.controller_manager import ControllerManager
from kinematics import *
import keyboard
import time
import os
from pprint import pprint
from vision.srv import *


class JointStatePublisher:

    def __init__(self):
        self.robot_name = 'ur5'
        self.q_des = np.zeros(6)
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)
        self.frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_position = conf.robot_params[self.robot_name]['q_0']
        self.q = np.zeros(6)

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
            self.gripper_type = conf.robot_params[self.robot_name]['gripper_type']
            self.soft_gripper = conf.robot_params[self.robot_name]['soft_gripper']
            if self.soft_gripper:
                self.gripper_joint_names = conf.robot_params[self.robot_name]['soft_gripper_joint_names']
            else:
                self.gripper_joint_names = conf.robot_params[self.robot_name]['gripper_joint_names']
        else:
            self.gripper = False

        self.vel_limits = conf.robot_params[self.robot_name]['vel_limit']
        self.ds = 0.05
        self.samples = 200
        self.des_jstates = []
        self.des_jvels = []
        self.times = []
        self.t = 0

        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()

    def send_des_jstate(self, q_des, q_des_gripper):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = Float64MultiArray()
        if self.gripper and not self.real_robot:
            msg.data = np.append(q_des, q_des_gripper)
            #print(msg.data)  # aperto: 0.45, chiuso: -0.25
        else:
            msg.data = q_des
        self.pub_des_jstate.publish(msg)

    def send_des_trajectory(self, positions, velocities, durations):
        jt = JointTrajectory()

        if self.gripper:
            jt.joint_names = self.joint_names + self.gripper_joint_names
        else:
            jt.joint_names = self.joint_names

        jt.header.stamp = rospy.Time.now()

        if velocities is not None:
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.velocities = velocities[i]
                jtp.time_from_start = rospy.Duration(durations[i] + 1)
                jt.points.append(jtp)
        else:
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.time_from_start = rospy.Duration(durations[i] + 1)
                jt.points.append(jtp)

        self.pub_traj_jstate.publish(jt)

    def receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]

    def nthRoot(self, x, n):
        if x > 0:
            return math.pow(x, float(1) / n)
        elif x < 0:
            return -math.pow(abs(x), float(1) / n)
        else:
            return 0

    def jquintic(self, T, qi, qf, vi, vf, ai, af):
        # traiettoria nello spazio dei joint
        h = qf - qi
        a0 = np.ndarray.copy(qi)
        a1 = np.ndarray.copy(vi)
        a2 = np.ndarray.copy(ai) / 2
        Tinv = np.linalg.inv(
            mat([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4], [6 * T, 12 * T ** 2, 20 * T ** 3]]))
        a3 = Tinv[0, 0] * (h - vf * T) + Tinv[0, 1] * (vf - vi - ai * T) + Tinv[0, 2] * (af - ai)
        a4 = Tinv[1, 0] * (h - vf * T) + Tinv[1, 1] * (vf - vi - ai * T) + Tinv[1, 2] * (af - ai)
        a5 = Tinv[2, 0] * (h - vf * T) + Tinv[2, 1] * (vf - vi - ai * T) + Tinv[2, 2] * (af - ai)

        times = np.linspace(0, T, 50)

        points = []
        velocities = []
        # calcolo i punti della traiettoria e relative velocità attraverso la quintica
        for t in times:
            points.append(a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5)
            velocities.append(a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4)

        self.des_jstates = points
        self.des_jvels = velocities
        self.times = times

    def jcubic(self, T, qi, qf, vi, vf):
        h = qf - qi  # vettore - displacement sui 6 theta tra pos iniziale e finale
        # calcolo i parametri della cubica, per ogni joint
        a0 = np.ndarray.copy(qi)  # vettore [qi1, qi2, qi3, qi4, qi5, qi6]
        a1 = np.ndarray.copy(vi)  # vettore [qf1, qf2, qf3, qf4, qf5, qf6]
        Tinv = np.linalg.inv(mat([[T * T, T * T * T], [2 * T, 3 * T * T]]))
        a2 = Tinv[0, 0] * (h - vi * T) + Tinv[0, 1] * (vf - vi)
        a3 = Tinv[1, 0] * (h - vi * T) + Tinv[1, 1] * (vf - vi)

        times = np.linspace(0, T, 50)

        points = []
        velocities = []
        # calcolo i punti della traiettoria e relative velocità attraverso la cubica
        for t in times:
            points.append(a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3)
            velocities.append(a1 + 2 * a2 * t + 3 * a3 * t ** 2)

        self.des_jstates = points
        self.des_jvels = velocities
        self.times = times

    def quinticMovement(self, t, a0, a1, a2, a3, a4, a5):
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

    def getGripperJointState(self, diameter):
        if self.soft_gripper:
            D0 = 40
            L = 60
            delta = 0.5 * (diameter - D0)
            return math.atan2(delta, L) * np.ones(2)
        else:
            return ((diameter - 22) / (130 - 22) * (-np.pi) + np.pi)  * np.ones(3)  # D = 130-> q = 0, D = 22 -> q = 3.14

    def grip(self, jstate, times, gripper_pos, ungrip):
        if not ungrip:
            q_gripper = self.getGripperJointState(gripper_pos)
            final_q = np.append(jstate, q_gripper)
            self.send_des_trajectory([final_q], None, [times])

        time.sleep(5.)
        req = AttachRequest()

        req.model_name_1 = "ur5"
        req.link_name_1 = "hand_1_link"
        req.model_name_2 = "brick1"
        req.link_name_2 = "link"

        if not ungrip:
            rospy.loginfo("Attaching block")
            self.attach_srv.call(req)
        else:
            rospy.loginfo("Detaching block")
            self.detach_srv(req)

        if ungrip:
            q_gripper = self.getGripperJointState(gripper_pos)
            final_q = np.append(jstate, q_gripper)
            self.send_des_trajectory([final_q], None, [times])
            time.sleep(5.)

        return final_q

    def moveTo(self, jstate, final_pos, final_rotm, gripper_pos, old_time=0):
        poss, vels, times = differential_kin(jstate, final_pos, final_rotm)
        final_jstate = poss[-1]

        for i in range(len(poss)):
            q_gripper = self.getGripperJointState(gripper_pos)
            poss[i] = np.append(poss[i], q_gripper)
            # vels[i] = np.append(vels[i], np.array([0, 0]))

        print(poss)

        time.sleep(2.)
        times = [t + old_time for t in times]
        final_time = times[-1]

        self.send_des_trajectory(poss, vels, times)
        time.sleep(5.)

        return final_jstate, final_time

    def pickUpBlock(self, block_pos, block_orient, final_pos):
        # block_pos   : coordinate x,y del centro del blocco
        # block_orient: rotazione del blocco
        # final_pos   : posizione x,y finale dove deve venire posizionato il blocco
        print('moving robot to desired position: ', block_pos)

        # range gripper: 130-22
        gripper_opened = 100
        gripper_closed = 40

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        current_jstate, current_time = self.moveTo(self.homing_position, frontal_p, frontal_rotm, gripper_opened)

        time.sleep(5.)
        print('------- moving to block --------')
        block_pos = np.array([block_pos[0], block_pos[1], -0.83])
        block_rotm = eul2rotm([-pi, 0, block_orient])

        current_jstate, current_time = self.moveTo(current_jstate, block_pos, block_rotm, gripper_opened, current_time)

        time.sleep(3.)
        print('-------- gripping ---------')
        self.grip(current_jstate, current_time, gripper_closed, False)

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        current_jstate, current_time = self.moveTo(current_jstate, frontal_p, frontal_rotm, gripper_closed, current_time)

        print('------ moving to final position -------')
        final_p = np.array([final_pos[0], final_pos[1], -0.83])
        final_phi = np.array([-pi, 0, pi / 2])
        final_rotm = eul2rotm(final_phi)

        current_jstate, current_time = self.moveTo(current_jstate, final_p, final_rotm, gripper_closed, current_time)

        time.sleep(5.)
        print('------- un-gripping --------')
        self.grip(current_jstate, current_time, gripper_opened, True)

        print('----- moving to frontal position ------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        self.moveTo(current_jstate, frontal_p, frontal_rotm, gripper_opened, current_time)


def talker(mypub):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    mypub.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
    mypub.pub_traj_jstate = ros.Publisher("/ur5/pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
    mypub.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback=mypub.receive_jstate, queue_size=1)

    # init variables
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    rospy.wait_for_service('vision_service')
    try:
        print("server called")
        results = rospy.ServiceProxy('vision_service', vision)
        res = results()
        print(res)
        # for i in range(res.n_res):
        #     if res.xcentre[i] < 0.5 and res.ycentre[i] < 0.4:
        #         block_pos = [res.xcentre[i], res.ycentre[i]]
        #         angle = res.angle[i]

        mypub.pickUpBlock(block_pos, angle, [0.45, 0.3])
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
