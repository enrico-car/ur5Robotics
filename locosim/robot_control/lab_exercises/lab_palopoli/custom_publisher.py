#!/usr/bin/env python

import random
from lab_exercises.lab_palopoli.kinematics import *
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
class2Zpos = {0: -0.935+0.013, 1: -0.94+0.013, 2: -0.935+0.013, 3: -0.935+0.013, 4: -0.935+0.013, 5: -0.935+0.013, 6: -0.935+0.013, 7: -0.94+0.013,
              8: -0.935+0.013, 9: -0.915+0.013, 10: -0.915+0.013}
class2gripsize = {0: [60, 31.7], 1: [60, 31.5], 2: [60, 31.5], 3: [60, 31.5], 4: [60, 31.5], 5: [60, 31.5],
                  6: [60, 31.5], 7: [60, 31.5], 8: [60, 31.5], 9: [100, 68], 10: [100, 68]}
class2blockdimensions = {0: [0.031, 0.031, 0.0375], 1: [0.031, 0.063, 0.0185], 2: [0.031, 0.063, 0.0375], 3: [0.031, 0.063, 0.0375],
                         4: [0.031, 0.063, 0.0375], 5: [0.031, 0.095, 0.0375], 6: [0.031, 0.095, 0.0375], 7: [0.031, 0.127, 0.0185],
                         8: [0.031, 0.127, 0.0375], 9: [0.063, 0.063, 0.0375], 10: [0.063, 0.063, 0.0375]}


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

        if conf.robot_params['ur5']['world_name'] == 'tavolo_brick.world':
            self.attach_srv = ros.ServiceProxy('/link_attacher_node/attach', Attach)
            self.attach_srv.wait_for_service()
            self.detach_srv = ros.ServiceProxy('/link_attacher_node/detach', Attach)
            self.detach_srv.wait_for_service()
            print('----- link attacher service started -----')

        #self.setstatic_srv = ros.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
        #self.setstatic_srv.wait_for_service()
        #print('----- gazebo ros link attacher service started -----')

        self.block_grasped = False
        self.grasped_block_name = None
        self.last_block = "tavolo"

        self.vision = conf.robot_params[self.robot_name]['vision']
        if self.vision:
            self.vision_service = ros.ServiceProxy('vision_service', vision)
            self.vision_service.wait_for_service()
            print('----- vision server started ------')

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

        if velocities is None:
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.time_from_start = ros.Duration(durations[i] + 1)
                jt.points.append(jtp)
        else:
            print('++++++++ using velocities ++++++++')
            for i in range(0, len(positions)):
                jtp = JointTrajectoryPoint()
                jtp.positions = positions[i]
                jtp.velocities = velocities[i]
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

    def grip(self, jstate, actual_time, gripper_pos, grip, attach_to_table):
        if grip:
            if not self.real_robot:
                q_gripper = self.mapGripperJointState(gripper_pos)
                final_q = np.append(jstate, q_gripper)
                self.send_des_trajectory([final_q], None, [actual_time + 0.1])
                time.sleep(1.)
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

        if not grip:
            if not self.real_robot:
                q_gripper = self.mapGripperJointState(gripper_pos)
                final_q = np.append(jstate, q_gripper)

                self.send_des_trajectory([final_q], None, [actual_time + 0.1])

                print('waiting to reach gripper position')
                while np.linalg.norm(self.q_gripper - q_gripper) > 0.01:
                    pass

                if attach_to_table:
                    time.sleep(1.)
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

    def activeGripping(self, gripper_opened, gripper_closed, current_jstate, block_pos):
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
        current_jstate = self.moveTo(current_jstate, block_pos + np.array([0, 0, 0.1]), block_rotm2, gripper_opened, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass
        current_jstate = self.moveTo(current_jstate, block_pos, block_rotm2, gripper_opened, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        return False, current_jstate

    def moveTo(self, jstate, final_pos, final_rotm, gripper_pos, old_time=0.01, curve_type='bezier'):
        poss, vels, times = differential_kin(jstate, final_pos, final_rotm, curve_type=curve_type, vel=1)
        vels = None
        final_jstate = poss[-1]

        if not self.real_robot:
            if vels is None:
                for i in range(len(poss)):
                    q_gripper = self.mapGripperJointState(gripper_pos)
                    poss[i] = np.append(poss[i], q_gripper)
            else:
                for i in range(len(poss)):
                    q_gripper = self.mapGripperJointState(gripper_pos)
                    poss[i] = np.append(poss[i], q_gripper)
                    vels[i] = np.append(vels[i], np.zeros(len(q_gripper)))

        time.sleep(1.)
        if old_time != 0:
            times = [t + old_time for t in times]

        self.send_des_trajectory(poss, vels, times)

        return final_jstate

    def pickAndPlaceBlock(self, block_pos, block_phi, block_class, final_pos, final_phi = np.array([-pi, 0, pi / 2]), initial_jstate=None, lock_block_to_table=True):
        # block_pos: coordinate x,y del centro del blocco e z come altezza a cui gripparlo
        # block_phi: euler angles che descrivono l'orientazione del blocco
        # final_pos: posizione x,y,z finale dove deve venire posizionato il blocco (z può variare in base a che altezza della torre viene posizionato)
        # final_phi: orientazione finale del blocco
        # initial_jstate: posizione iniziale dei joint del braccio (default a None -> homing position)
        # lock_block_to_table: crea un joint virtuale tra il blocco (nella posizione finale) e il tavolo da lavoro (attraverso gazebo_ros_link_attacher plugin)
        
        print('moving robot to desired position: ', block_pos)

        gripper_opened, gripper_closed = class2gripsize[block_class]

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.6])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        if initial_jstate is None:
            current_jstate = self.moveTo(self.homing_position, frontal_p, frontal_rotm, gripper_opened)
        else:
            current_jstate = self.moveTo(initial_jstate, frontal_p, frontal_rotm, gripper_opened)

        print('------- moving to block --------')
        block_rotm = eul2rotm([-pi, 0, -block_phi[2]])

        current_jstate = self.moveTo(current_jstate, block_pos, block_rotm, gripper_opened)

        print('waiting to reach position')

        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('-------- gripping ---------')
        if self.gripper:
            gripped = False
            while not gripped:
                gripped, current_jstate = self.activeGripping(gripper_opened, gripper_closed, current_jstate, block_pos)

        print('------- moving upwards --------')
        actual_pose = direct_kin(current_jstate)
        upwards_pos = np.array(actual_pose[0:3, 3].flat) + np.array([0, 0, 0.3])
        upwards_rotm = actual_pose[0:3, 0:3]

        current_jstate = self.moveTo(current_jstate, upwards_pos, upwards_rotm, gripper_closed, curve_type='line')

        print('------ moving frontal position -------')
        frontal_p = np.array([0, 0.4, -0.6])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        current_jstate = self.moveTo(self.q, frontal_p, frontal_rotm, gripper_closed)

        print('------ moving above final position: -------')
        final_rotm = eul2rotm(final_phi)

        current_jstate = self.moveTo(current_jstate, final_pos+np.array([0, 0, 0.15]), final_rotm, gripper_closed, curve_type='line')


        print('------ moving to final position: ', final_pos, ' -------')
        final_rotm = eul2rotm(final_phi)

        current_jstate = self.moveTo(current_jstate, final_pos, final_rotm, gripper_closed, curve_type='line')

        print('waiting to reach position')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('------- un-gripping --------')
        if self.gripper:
            self.grip(current_jstate, 0.1, gripper_opened, False, lock_block_to_table)

        print('------- moving upwards --------')
        current_pose = direct_kin(self.q)
        if lock_block_to_table:
            upwards_pos = np.array(current_pose[0:3, 3].flat) + np.array([0, 0, 0.15])
        else:
            upwards_pos = np.array(current_pose[0:3, 3].flat) + np.array([0, 0, 0.05])
        upwards_rotm = current_pose[0:3, 0:3]

        current_jstate = self.moveTo(current_jstate, upwards_pos, upwards_rotm, gripper_opened, curve_type='line')

        return current_jstate

    def rotateBlock(self, initial_jstate, block_pos, block_orient, block_class):
        [gripper_opened, gripper_closed] = class2gripsize[block_class]

        print('----- place block in rotation point facing outside ------')
        rotation_area_pos = np.array([0, 0.45-0.0375/2, block_pos[2]])
        rotation_area_phi = np.array([-pi, 0, -pi/2])
        current_jstate = self.pickAndPlaceBlock(block_pos, block_orient, block_class, rotation_area_pos, final_phi=rotation_area_phi, initial_jstate=initial_jstate, lock_block_to_table=False)
        
        print('------ moving to block at 45° -------')
        actual_pose = direct_kin(self.q)
        inclinated_pos = rotation_area_pos
        actual_rotm = actual_pose[0:3, 0:3]
        y_dir = np.array(actual_pose[0:3, 1].flat)
        # controllo in che direzione è la y (perchè può essere che il gripper si sia girato di 180° durante il gripping)
        # ed eseguo la rotazione di conseguenza
        if (np.dot(y_dir, np.array([1,0,0]))) < 0:
            theta = pi/4
        else:
            theta = -pi/4
        rotY = mat([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
        inclinated_rotm = actual_rotm @ rotY
        current_jstate = self.moveTo(current_jstate, inclinated_pos, inclinated_rotm, gripper_opened, 0.1, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass
        #input("Press Enter to continue...")

        print('-------- gripping ---------')
        if self.gripper:
            gripped = False
            while not gripped:
                gripped, current_jstate = self.activeGripping(gripper_opened, gripper_closed, current_jstate, inclinated_pos)

        print('------ rotating block -------')
        actual_pose = direct_kin(self.q)
        actual_rotm = actual_pose[0:3, 0:3]
        y_dir = np.array(actual_pose[0:3, 1].flat)
        # stesso controllo di prima sulla direzione dell'asse y attuale
        if (np.dot(y_dir, np.array([1,0,0]))) < 0:
            theta = -pi/2
        else:
            theta = pi/2
        rotY = mat([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])
        final_rotm = actual_rotm @ rotY
        final_pos = [rotation_area_pos[0], rotation_area_pos[1], class2Zpos[block_class]]
        final_pos = np.array(final_pos) + np.array([0, 0, 0.01])
        # muoviti in su di 10cm
        current_jstate = self.moveTo(current_jstate, inclinated_pos+np.array([0, 0, 0.06]), actual_rotm, gripper_closed, 0.1, curve_type='line')
        # ruota di 90°
        current_jstate = self.moveTo(current_jstate, inclinated_pos+np.array([0, 0, 0.06]), final_rotm, gripper_closed, 0.1, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass
        # appoggia il blocchetto
        current_jstate = self.moveTo(current_jstate, final_pos, final_rotm, gripper_closed, 0.1, curve_type='line')
        while np.linalg.norm(current_jstate - self.q) > 0.005:
            pass

        print('------- un-gripping --------')
        if self.gripper:
            self.grip(current_jstate, 0.1, gripper_opened, False, True)
        
        print('------- moving upwards --------')
        current_pose = direct_kin(self.q)
        upwards_pos = np.array(current_pose[0:3, 3].flat) + np.array([0, 0, 0.15])
        upwards_phi = np.array([pi, 0, -pi/2])
        upwards_rotm = eul2rotm(upwards_phi)

        current_jstate = self.moveTo(current_jstate, upwards_pos, upwards_rotm, gripper_opened, curve_type='line')

        while np.linalg.norm(current_jstate - self.q) > 0.05:
            pass
            
        print('****** block rotated ******')

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
            res.angle = [[pi/2, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
            res.class_ = [0, 2, 3, 4, 5, 7, 1]
            I = [0, 1]
            # I = [0, 1, 2, 3, 4, 5, 6]
            # I = [6, 5, 4, 3, 2, 1, 0]
            # random.shuffle(I)

        first = True
        height = 0
        final_tower_pos = np.array([0.2, 0.4, 0])
        for i in I:
            block_class = res.class_[i]
            block_orientation = res.angle[i]
            # valori del centro del blocco uscenti dalla vision già corretti (correzzione in base alla dimensione del blocco)
            block_pos = [res.xcentre[i]-0.5, res.ycentre[i]-0.35, class2Zpos[block_class]]
            # TODO: aggiungere nella vision controllo su detect sbagliate (tipo gripper o parti del tavolo o altro)
            
            if res.angle[i][0] != 0 or res.angle[i][1] != 0:
                # controllo se il blocchetto non è il posizione naturale (con la base sul tavolo)
                self.rotateBlock(self.q, block_pos, block_orientation, block_class)
        
            if first:
                current_jstate = mypub.pickAndPlaceBlock(block_pos, res.angle[i], final_tower_pos + np.array([0, 0, class2Zpos[block_class]]), block_class)
                height = class2blockdimensions[block_class][2]
                first = False
            else:
                final_block_pos = final_tower_pos + np.array([ -class2blockdimensions[block_class][0]/2, 0, class2Zpos[block_class]+height ])
                current_jstate = mypub.pickAndPlaceBlock(block_pos, res.angle[i], final_block_pos, block_class, current_jstate)
                height = height + class2blockdimensions[block_class][2]
            # input('Press enter to continue ....')

        print('------ Moving to frontal position ------')
        frontal_p = np.array([0, 0.4, -0.6])
        frontal_phi = np.array([-pi, 0, 0])
        frontal_rotm = eul2rotm(frontal_phi)

        self.moveTo(self.q, frontal_p, frontal_rotm, 80)

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
        self.roll = []



def talker(mypub):
    ros.init_node('custom_publisher_node', anonymous=True)

    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    ros.Subscriber('/grasp_event_republisher/grasp_events', GazeboGraspEvent, mypub.grasp_manager)

    #mypub.multipleBlocks()
    #print('------- task completato -------')

    cl = 0
    pos = np.array([0.8-0.5, 0.4-0.35-0.0365/2, class2Zpos[cl]])
    phi = np.array([pi/2, 0, pi/2])
    mypub.rotateBlock(mypub.q, pos, phi, cl)
    pos = np.array([0, 0.45-0.0375/2, class2Zpos[cl]])
    rotm = eul2rotm(np.array([pi, 0, pi/2]))
    mypub.moveTo(mypub.q, pos, rotm, 60, curve_type='line')
    
    # cl = 0
    # pos = np.array([0.5-0.5, 0.8-0.35-0.0375/2, class2Zpos[cl]])
    # phi = np.array([-pi, 0, pi/2])
    # rotm = eul2rotm(phi)
    # mypub.moveTo(mypub.q, pos, rotm, 40)
    # input('press enter...')
    # # cl = 0
    # pos = pos #+ np.array([0, 0, 0.005])
    # # phi = np.array([-pi, 0, pi/2])
    # # rotm = eul2rotm(phi)
    # theta = pi/4
    # rotY = mat([[cos(-theta), 0, sin(-theta)], [0, 1, 0], [-sin(-theta), 0, cos(-theta)]])
    # rotm = rotm @ rotY
    # mypub.moveTo(mypub.q, pos, rotm, 40, curve_type='line')

    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
