#!/usr/bin/env python
import random

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
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent
from base_controllers.components.controller_manager import ControllerManager
from kinematics import *
import keyboard
import time
import os
from pprint import pprint
from vision.srv import *
import cv2
from gazebo_msgs.msg import *
from gazebo_msgs.srv import SetModelState


class2name = {0: 'X1-Y1-Z2', 1: 'X1-Y2-Z1', 2:'X1-Y2-Z2', 3: 'X1-Y2-Z2-CHAMFER', 4: 'X1-Y2-Z2-TWINFILLET', 5: 'X1-Y3-Z2',
              6: 'X1-Y3-Z2-FILLET', 7: 'X1-Y4-Z1', 8: 'X1-Y4-Z2', 9: 'X1-Y2-Z2',
              10: 'X1-Y2-Z2-FILLET'}
class2Zheight = {0: -0.935, 1: -0.94, 2: -0.935, 3: -0.935, 4: -0.935, 5: -0.935, 6: -0.935, 7: -0.94,
                8: -0.935, 9: -0.915, 10: -0.915}
class2blockheight = {0: 0.0375, 1: 0.0185, 2: 0.0375, 3: 0.0375, 4: 0.0375, 5: 0.0375, 6: 0.0375, 7: 0.0185,
                8: 0.0375, 9: 0.0375, 10: 0.0375}
class2gripsize = {0: [60, 31.5], 1: [60, 31.5], 2: [60, 31.5], 3: [60, 31.5], 4: [60, 31.5], 5: [60, 31.5],
                  6: [60, 31.5], 7: [60, 31.5], 8: [60, 31.5], 9: [100, 68], 10: [100, 68]}


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
        if self.real_robot:  # se usiamo il real_robot, il gripper non si controlla da ROS ma con il programma del prof
            self.gripper = False
        else:
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
                self.soft_gripper = False

        self.q = np.zeros(6)

        self.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
        if self.real_robot:
            self.pub_traj_jstate = ros.Publisher("/ur5/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        else:
            self.pub_traj_jstate = ros.Publisher("/ur5/pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback=self.receive_jstate, queue_size=1)

        self.vel_limits = conf.robot_params[self.robot_name]['vel_limit']
        self.ds = 0.05
        self.samples = 200
        self.des_jstates = []
        self.des_jvels = []
        self.times = []
        self.t = 0

        self.attach_srv = ros.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        self.detach_srv = ros.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()

        self.setstatic_srv = ros.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
        self.setstatic_srv.wait_for_service()

        self.block_grasped = False
        self.grasped_block_name = None
        self.last_block = "tavolo"

        self.vision = conf.robot_params[self.robot_name]['vision']
        if self.vision:
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

        jt.header.stamp = ros.Time.now()

        if velocities is not None:
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.velocities = velocities[i]
                jtp.time_from_start = ros.Duration(durations[i] + 1)
                jt.points.append(jtp)
        else:
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.time_from_start = ros.Duration(durations[i] + 1)
                jt.points.append(jtp)

        self.pub_traj_jstate.publish(jt)

    def receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
            if self.gripper:
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

    def moveRealGripper(self, diameter):
        import socket

        HOST = "192.168.0.100"  # The UR IP address
        PORT = 30002  # UR secondary client
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        sock.settimeout(0.5)
        try:
            sock.connect((HOST, PORT))
        except:
            raise Exception("Cannot connect to end-effector socket") from None
        sock.settimeout(None)
        scripts_path = rospkg.RosPack().get_path('ur_description') + '/gripper/scripts'

        onrobot_script = scripts_path + "/onrobot_superminimal.script"
        file = open(onrobot_script, "rb")  # Robotiq Gripper
        lines = file.readlines()
        file.close()

        tool_index = 0
        blocking = True
        cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

        line_number_to_add = 446

        new_lines = lines[0:line_number_to_add]
        new_lines.insert(line_number_to_add + 1, str.encode(cmd_string))
        new_lines += lines[line_number_to_add::]

        offset = 0
        buffer = 2024
        file_to_send = b''.join(new_lines)

        if len(file_to_send) < buffer:
            buffer = len(file_to_send)
        data = file_to_send[0:buffer]
        while data:
            sock.send(data)
            offset += buffer
            if len(file_to_send) < offset + buffer:
                buffer = len(file_to_send) - offset
            data = file_to_send[offset:offset + buffer]
        sock.close()

        print("Gripper moved, now resend robot program")

        ros.sleep(1.5)
        ros.wait_for_service("/ur5/ur_hardware_interface/resend_robot_program")
        sos_service = ros.ServiceProxy('/ur5/ur_hardware_interface/resend_robot_program', Trigger)
        sos = TriggerRequest()
        result = sos_service(sos)
        # print(result)
        ros.sleep(0.1)

    def grasp_manager(self, data):
        print(data)
        if data.attached:
            print('******* block grasped ********')
            self.block_grasped = True
            self.grasped_block_name = data.object.split("::")[0]
            print('block name: ', self.grasped_block_name)
            time.sleep(0.2)
        else:
            time.sleep(0.2)
            self.block_grasped = False
        #     print('******* block un-grasped ********')
        #     self.grasped_block_name = None

    def grip(self, jstate, actual_time, gripper_pos, grip, attach_to_table):
        if grip:
            if not self.real_robot:
                q_gripper = self.mapGripperJointState(gripper_pos)
                final_q = np.append(jstate, q_gripper)
                self.send_des_trajectory([final_q], None, [actual_time + 0.1])
                time.sleep(1.)
                # print('waiting to reach gripper position')
                # while np.linalg.norm(self.q_gripper - q_gripper) > 0.1:
                #     pass
                start = time.time()
                print('waiting to grasp object')
                while not self.block_grasped:
                    end = time.time()
                    if end - start > 2.:
                        return False
                    pass
            else:
                self.moveRealGripper(gripper_pos)
                time.sleep(3.)

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

        if not grip:
            if not self.real_robot:
                q_gripper = self.mapGripperJointState(gripper_pos)
                final_q = np.append(jstate, q_gripper)

                self.send_des_trajectory([final_q], None, [actual_time + 0.1])

                print('waiting to reach gripper position')
                while np.linalg.norm(self.q_gripper - q_gripper) > 0.05:
                    pass

                if attach_to_table:
                    time.sleep(0.5)
                    # Attach block to table
                    req = AttachRequest()
                    req.model_name_1 = self.grasped_block_name
                    req.link_name_1 = "link"
                    req.model_name_2 = "tavolo"
                    req.link_name_2 = "link"
                    res = self.attach_srv.call(req)
                    if res.ok:
                        print('attached ', req.model_name_1, ' to ', req.model_name_2)

                    # # Set model static
                    # req = SetStaticRequest()
                    # req.model_name = self.grasped_block_name
                    # req.link_name = "link"
                    # req.set_static = True
                    # self.setstatic_srv.call(req)
                    # time.sleep(0.5)
                    #
                    # self.last_block = self.grasped_block_name
            else:
                self.moveRealGripper(gripper_pos)
                time.sleep(3.)

        return True

    def regripping(self, gripper_opened, gripper_closed, current_jstate, block_pos):
        # se il blocco non è stato grippato, apri e riprova il gripping girando di 180°
        self.grip(current_jstate, 0.1, gripper_opened, False, False)
        gripped = self.grip(current_jstate, 0.1, gripper_closed, True, False)
        time.sleep(0.3)
        if gripped:
            return True, current_jstate
        self.grip(current_jstate, 0.1, gripper_opened, False, False)
        current_pose = direct_kin(self.q)
        current_rotm = current_pose[0:3, 0:3]
        block_rotm2 = current_rotm @ mat([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        current_jstate, current_time = self.moveTo(current_jstate, block_pos + np.array([0, 0, 0.1]), block_rotm2,
                                                   gripper_opened, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass
        current_jstate, current_time = self.moveTo(current_jstate, block_pos, block_rotm2, gripper_opened,
                                                   curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        return False, current_jstate

    def moveTo(self, jstate, final_pos, final_rotm, gripper_pos, old_time=0.01, curve_type='bezier'):
        poss, vels, times = differential_kin(jstate, final_pos, final_rotm, curve_type=curve_type, vel=1)
        final_jstate = poss[-1]

        if not self.real_robot:
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

    def pickUpBlock(self, block_pos, block_orient, final_pos, block_class, initial_jstate=None, initial_time=0):
        # block_pos   : coordinate x,y del centro del blocco e z come altezza a cui gripparlo
        # block_orient: rotazione del blocco
        # final_pos   : posizione x,y,z finale dove deve venire posizionato il blocco
        #               (z può variare in base a che altezza della torre viene posizionato)
        print('moving robot to desired position: ', block_pos)

        gripper_opened, gripper_closed = class2gripsize[block_class]

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.6])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        if initial_jstate is not None:
            current_jstate, current_time = self.moveTo(initial_jstate, frontal_p, frontal_rotm, gripper_opened)
        else:
            current_jstate, current_time = self.moveTo(self.homing_position, frontal_p, frontal_rotm, gripper_opened)

        print('------- moving to block --------')
        block_rotm = eul2rotm([-pi, 0, -block_orient])

        current_jstate, current_time = self.moveTo(current_jstate, block_pos, block_rotm, gripper_opened)

        print('waiting to reach position')

        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass
        #input("Press Enter to continue...")

        print('-------- gripping ---------')
        if self.gripper:
            gripped = False
            while not gripped:
                gripped, current_jstate = self.regripping(gripper_opened, gripper_closed, current_jstate, block_pos)

        print('------- moving upwards --------')
        actual_pose = direct_kin(current_jstate)
        upwards_pos = np.array(actual_pose[0:3, 3].flat) + np.array([0, 0, 0.3])
        upwards_rotm = actual_pose[0:3, 0:3]

        current_jstate, current_time = self.moveTo(current_jstate, upwards_pos, upwards_rotm, gripper_closed,
                                                   curve_type='line')

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.6])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        # current_jstate, current_time = self.moveTo(current_jstate, frontal_p, frontal_rotm, gripper_closed)
        current_jstate, current_time = self.moveTo(self.q, frontal_p, frontal_rotm, gripper_closed)

        print('------ moving to final position: ', final_pos, ' -------')
        final_phi = np.array([-pi, 0, pi / 2])
        final_rotm = eul2rotm(final_phi)

        current_jstate, current_time = self.moveTo(current_jstate, final_pos, final_rotm, gripper_closed)

        print('waiting to reach position')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('------- un-gripping --------')
        if self.gripper:
            self.grip(current_jstate, 0.1, gripper_opened, False, True)

        print('------- moving upwards --------')
        current_pose = direct_kin(self.q)
        upwards_pos = np.array(current_pose[0:3, 3].flat) + np.array([0, 0, 0.15])
        upwards_rotm = current_pose[0:3, 0:3]

        current_jstate, current_time = self.moveTo(current_jstate, upwards_pos, upwards_rotm, gripper_opened, curve_type='line')

        return current_jstate, current_time

    def multipleBlocks(self):
        if self.vision:
            print("vision server called")
            res = mypub.vision_service.call()
            print(res)
        else:
            res = Res()
            res.n_res = 7
            res.xcentre = [0.05, 0.05, 0.05, 0.5, 0.8, 0.8, 0.8]
            res.ycentre = [0.25, 0.5, 0.75, 0.75, 0.75, 0.5, 0.25]
            res.angle = [0, 0, 0, 0, 0, 0, 0]
            res.class_ = [9, 2, 3, 4, 5, 7, 1]
            # I = [0, 1, 2, 3, 4, 5, 6]
            I = [6, 5, 4, 3, 2, 1, 0]
            # random.shuffle(I)

        final_block_pos = np.array([0.2, 0.4, 0])
        first = True
        height = 0
        for i in I:
            # block_pos = [res.xcentre[res.n_res-1-i]-0.51, res.ycentre[res.n_res-1-i]-0.345]
            block_class = res.class_[i]
            block_pos = [res.xcentre[i]-0.5, res.ycentre[i]-0.35, class2Zheight[block_class]]
            if abs(block_pos[0]) < 1. and abs(block_pos[1]) < 0.8:
                angle = pi/2 - res.angle[i]
                if first:
                    current_jstate, current_time = mypub.pickUpBlock(block_pos, angle, final_block_pos + np.array([0, 0, class2Zheight[block_class]]), block_class)
                    height = class2blockheight[block_class]
                    print('!!!!!! height of castle: ', height, '!!!!!!')
                    first = False
                else:
                    current_jstate, current_time = mypub.pickUpBlock(block_pos, angle, final_block_pos + np.array([0, 0, class2Zheight[block_class]+height]), block_class, current_jstate)
                    height = height + class2blockheight[block_class]
                    print('!!!!!! height of castle: ', height, '!!!!!!')
            # input('Press enter to continue ....')

        print('------ Moving to frontal position ------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        self.moveTo(self.q, frontal_p, frontal_rotm, 80)


    def rotateBlock(self, initial_jstate, block_pos, block_orient):
        gripper_opened = 65
        gripper_closed = 31

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.5])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        current_jstate, current_time = self.moveTo(initial_jstate, frontal_p, frontal_rotm, gripper_opened, 0.1)

        theta = pi/4
        rotY = mat([[cos(-theta), 0, sin(-theta)], [0, 1, 0], [-sin(-theta), 0, cos(-theta)]])
        block_rotm = eul2rotm(block_orient)
        initial_rotm = block_rotm @ rotY

        current_jstate, current_time = self.moveTo(current_jstate, block_pos, initial_rotm, gripper_opened, 0.1)
        while np.linalg.norm(current_jstate - self.q) > 0.01:
            pass
        block_pos = np.array([block_pos[0], block_pos[1], block_pos[2]-0.2])
        current_jstate, current_time = self.moveTo(current_jstate, block_pos, initial_rotm, gripper_opened, 0.1, curve_type='line')
        input("Press Enter to continue...")

        print('-------- gripping ---------')
        self.grip(current_jstate, 0.1, gripper_closed, False)

        rotY = mat([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
        final_rotm = block_rotm @ rotY
        current_jstate, current_time = self.moveTo(current_jstate, block_pos, final_rotm, gripper_closed, 0.1, curve_type='line')

        print('------- un-gripping --------')
        self.grip(current_jstate, 0.1, gripper_opened, True)


    def checkPosition(self):
        points = [[0, 0.4, -0.5], [-0.52, -0.2, -0.79], [-0.52, 0.45, -0.94], [0.5, 0.45, -0.94], [0.5, -0.2, -0.79]]
        angle = [-pi, 0, -pi / 2]
        rotm = eul2rotm(angle)

        current_jstate, current_time = mypub.moveTo(self.homing_position, points[0], rotm, 40, 0.1)
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        for i in range(1, len(points)):
            current_jstate, current_time = self.moveTo(current_jstate, points[i], rotm, 40, 0.1, curve_type='line')
            while np.linalg.norm(current_jstate - self.q) > 0.005:
                pass
            input("Press Enter to continue...")


class Res:
    def __init__(self):
        self.n_res = 0
        self.class_ = []
        self.xcentre = []
        self.ycentre = []
        self.angle = []



def talker(mypub):
    ros.init_node('custom_publisher_node', anonymous=True)

    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    ros.Subscriber('/grasp_event_republisher/grasp_events', GazeboGraspEvent, mypub.grasp_manager)

    mypub.multipleBlocks()
    print('------- task completato -------')


    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
