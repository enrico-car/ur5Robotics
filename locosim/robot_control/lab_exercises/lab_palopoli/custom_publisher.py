#!/usr/bin/env python
import rospy
import rospy as ros
import numpy as np
import params as conf
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from base_controllers.components.controller_manager import ControllerManager
from kinematics import *
import keyboard
import time
import os
from pprint import pprint
from vision.srv import *
import cv2
from gazebo_msgs.msg import *


class JointStatePublisher:

    def __init__(self):
        self.robot_name = 'ur5'
        self.q_des = np.zeros(6)
        self.filter_1 = np.zeros(6)
        self.filter_2 = np.zeros(6)
        self.frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_position = conf.robot_params[self.robot_name]['q_0']

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        if conf.robot_params[self.robot_name]['gripper_sim']:
            self.gripper = True
            self.soft_gripper = conf.robot_params[self.robot_name]['soft_gripper']
            if self.soft_gripper:
                self.gripper_joint_names = conf.robot_params[self.robot_name]['soft_gripper_joint_names']
                self.q_gripper = np.zeros(2)
            else:
                self.gripper_joint_names = conf.robot_params[self.robot_name]['gripper_joint_names']
                self.q_gripper = np.zeros(3)
        else:
            self.gripper = False

        self.q = np.zeros(6)

        self.vel_limits = conf.robot_params[self.robot_name]['vel_limit']
        self.ds = 0.05
        self.samples = 200
        self.des_jstates = []
        self.des_jvels = []
        self.times = []
        self.t = 0

        # self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        # self.attach_srv.wait_for_service()
        # self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        # self.detach_srv.wait_for_service()

        self.vision_service = rospy.ServiceProxy('vision_service', vision)
        self.vision_service.wait_for_service()
        print('vision server started')

        self.marker_pub = ros.Publisher('/vis', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()
        self.id = 0

    def send_des_jstate(self, q_des, q_des_gripper):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = Float64MultiArray()
        if self.gripper and not self.real_robot:
            msg.data = np.append(q_des, q_des_gripper)
            # print(msg.data)  # aperto: 0.45, chiuso: -0.25
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
            for joint_idx in range(len(self.gripper_joint_names)):
                if self.gripper_joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q_gripper[joint_idx] = msg.position[msg_idx]

    def add_marker(self, pos, radius=0.1):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.x = 0.
        marker.pose.orientation.y = 0.
        marker.pose.orientation.z = 0.
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.lifetime = ros.Duration(0.0)

        marker.id = self.id
        self.id += 1
        self.marker_array.markers.append(marker)

    def publish_markers(self):
        if len(self.marker_array.markers) > 0:
            self.marker_pub.publish(self.marker_array)
            self.marker_array.markers.clear()
            self.id = 0

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

    def mapGripperJointState(self, diameter):
        if self.soft_gripper:
            D0 = 40
            L = 60
            delta = 0.5 * (diameter - D0)
            return math.atan2(delta, L) * np.ones(2)
        else:
            return ((diameter - 22) / (130 - 22) * (-np.pi) + np.pi) * np.ones(3)  # D = 130-> q = 0, D = 22 -> q = 3.14

    def grip(self, jstate, actual_time, gripper_pos, ungrip, classe):
        if not ungrip:
            q_gripper = self.mapGripperJointState(gripper_pos)
            final_q = np.append(jstate, q_gripper)
            self.send_des_trajectory([final_q], None, [actual_time + 0.1])
            time.sleep(1.)
            print('waiting to reach gripper position')
            while np.linalg.norm(self.q_gripper - q_gripper) > 0.01:
                pass

        # req = AttachRequest()
        #
        # req.model_name_1 = "ur5"
        # req.link_name_1 = "hand_1_link"
        # req.model_name_2 = "brick1"
        # req.link_name_2 = "link_" + str(classe)
        #
        # if not ungrip:
        #     rospy.loginfo("Attaching block")
        #     self.attach_srv.call(req)
        # else:
        #     rospy.loginfo("Detaching block")
        #     self.detach_srv.call(req)

        time.sleep(1.)
        if ungrip:
            q_gripper = self.mapGripperJointState(gripper_pos)
            final_q = np.append(jstate, q_gripper)
            self.send_des_trajectory([final_q], None, [actual_time + 0.1])
            print('waiting to reach gripper position')
            while np.linalg.norm(self.q_gripper - q_gripper) > 0.01:
                pass

    def moveTo(self, jstate, final_pos, final_rotm, gripper_pos, old_time=0, curve_type='bezier'):
        poss, vels, times = differential_kin(jstate, final_pos, final_rotm, curve_type=curve_type)
        final_jstate = poss[-1]

        for i in range(len(poss)):
            q_gripper = self.mapGripperJointState(gripper_pos)
            poss[i] = np.append(poss[i], q_gripper)
            # vels[i] = np.append(vels[i], np.array([0, 0]))

        time.sleep(1.)
        if old_time != 0:
            times = [t + old_time for t in times]
        final_time = times[-1]

        self.send_des_trajectory(poss, vels, times)

        return final_jstate, final_time

    def pickUpBlock(self, block_pos, block_orient, classe, final_pos, initial_jstate=None, initial_time=0):
        # block_pos   : coordinate x,y del centro del blocco
        # block_orient: rotazione del blocco
        # final_pos   : posizione x,y finale dove deve venire posizionato il blocco
        print('moving robot to desired position: ', block_pos)

        # range gripper: 130-22
        gripper_opened = 65
        gripper_closed = 31

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        if initial_jstate is not None:
            current_jstate, current_time = self.moveTo(initial_jstate, frontal_p, frontal_rotm, gripper_opened, 0.1)
        else:
            current_jstate, current_time = self.moveTo(self.homing_position, frontal_p, frontal_rotm, gripper_opened, 0.1)

        print('------- moving to block --------')
        block_pos = np.array([block_pos[0]+0.01, block_pos[1], -0.835])
        block_rotm = eul2rotm([-pi, 0, -block_orient])

        current_jstate, current_time = self.moveTo(current_jstate, block_pos, block_rotm, gripper_opened, 0.1)

        print('waiting to reach position')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('-------- gripping ---------')
        self.grip(current_jstate, 0.1, gripper_closed, False, classe)

        time.sleep(1.)

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        current_jstate, current_time = self.moveTo(current_jstate, frontal_p, frontal_rotm, gripper_closed, 0.1)
        input("Press Enter to continue...")
        print('------ moving to final position -------')
        final_p = np.array([final_pos[0], final_pos[1], final_pos[2]])
        final_phi = np.array([-pi, 0, pi / 2])
        final_rotm = eul2rotm(final_phi)

        current_jstate, current_time = self.moveTo(current_jstate, final_p, final_rotm, gripper_closed, 0.1)

        print('waiting to reach position')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('------- un-gripping --------')
        self.grip(current_jstate, 0.1, gripper_opened, True, classe)

        # print('----- moving to frontal position ------')
        # frontal_p = np.array([0, 0.4, -0.5])
        # frontal_phi = np.array([-pi, 0, 0])
        # frontal_rotm = eul2rotm(frontal_phi)
        #
        # current_jstate, current_time = self.moveTo(current_jstate, frontal_p, frontal_rotm, gripper_opened, 0.1)
        # print('waiting to reach position')
        # while np.linalg.norm(current_jstate - self.q) > 0.01:
        #     pass

        return current_jstate, current_time


class Res:
    def __init__(self):
        self.n_res = 0
        self.class_ = []
        self.xcentre = []
        self.ycentre = []
        self.angle = []



def talker(mypub):
    ros.init_node('custom_joint_pub_node', anonymous=True)
    mypub.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
    mypub.pub_traj_jstate = ros.Publisher("/ur5/pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
    mypub.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback=mypub.receive_jstate, queue_size=1)
    #mypub.sub_grasp_events = ros.Subscriber("~/grasp_events", ContactState, callback=test)
    # init variables

    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    print("vision server called")
    res = mypub.vision_service.call()
    print(res)

    distorted_points = mat([[-0.5 - 0.013, 0.45 - 0.016], [-0.5, -0.2 - 0.028],
                            [0.5 + 0.03, -0.2 + 0.023], [0.5 - 0.012, 0.45 + 0.017]])
    correct_points = mat([[-0.5, 0.45], [-0.5, -0.2], [0.5, -0.2], [0.5, 0.45]])
    h, status = cv2.findHomography(distorted_points, correct_points)

    # res = Res()
    # res.xcentre = [0.05, 0.95, 0.95]
    # res.ycentre = [0.75, 0.25, 0.7]
    # res.angle = [pi/2-1.117, pi/2-0.52, pi/2-1.57]
    # res.class_ = [0, 0, 0]
    # res.n_res = len(res.xcentre)

    for i in range(0, res.n_res):
        temp = h @ [res.xcentre[i] - 0.5, res.ycentre[i] - 0.35, 1]
        block_pos = [temp[0], temp[1]]
        if abs(block_pos[0]) < 1. and abs(block_pos[1]) < 0.8:
            angle = res.angle[i]
            classe = res.class_[i]

            if i == 0:
                current_jstate, current_time = mypub.pickUpBlock(block_pos, angle, classe,
                                                                 [0, 0.4, -0.835 + i * 0.04])
            else:
                current_jstate, current_time = mypub.pickUpBlock(block_pos, angle, classe,
                                                                 [0, 0.4, -0.835 + i * 0.04], current_jstate)




    # front = h @ [0, 0.4, 1]
    # front[2] = -0.5
    # fourth = h @ [0.5, 0.45, 1]
    # fourth[2] = -0.85
    # third = h @ [0.5, -0.25, 1]
    # third[2] = -0.7
    # second = h @ [-0.5, -0.25, 1]
    # second[2] = -0.7
    # first = h @ [-0.5, 0.45, 1]
    # first[2] = -0.85
    # angle = [-pi, 0, 0]
    # rotm = eul2rotm(angle)
    #
    # print(first)
    # print(second)
    # print(third)
    # print(fourth)
    #
    # input("Press Enter to continue...")
    #
    # print('moving robot to: ', front)
    #
    # # range gripper: 130-22
    # gripper_opened = 40
    #
    # current_jstate, current_time = mypub.moveTo(mypub.homing_position, front, rotm, gripper_opened)
    # print('moving to frontal position')
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    #
    # current_jstate, current_time = mypub.moveTo(current_jstate, first, rotm, gripper_opened)
    # print('moving to first position: ', first)
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    # input("Press Enter to continue...")
    #
    # current_jstate, current_time = mypub.moveTo(current_jstate, second, rotm, gripper_opened, curve_type='line')
    # print('moving to second position: ', second)
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    # input("Press Enter to continue...")
    #
    # current_jstate, current_time = mypub.moveTo(current_jstate, third, rotm, gripper_opened, curve_type='line')
    # print('moving to third position')
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    # input("Press Enter to continue...")
    #
    # current_jstate, current_time = mypub.moveTo(current_jstate, fourth, rotm, gripper_opened, curve_type='line')
    # print('moving to fourth position: ', third)
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    # input("Press Enter to continue...")
    #
    # current_jstate, current_time = mypub.moveTo(current_jstate, first, rotm, gripper_opened, curve_type='line')
    # print('moving to first position: ', fourth)
    # while np.linalg.norm(current_jstate - mypub.q) > 0.005:
    #     pass
    # input("Press Enter to continue...")

    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
