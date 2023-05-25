#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim"))
from lab_exercises.lab_palopoli.kinematics import *
from robot_control.vision.scripts.SpawnBlocks_temp import spawnBlocks
import lab_exercises.lab_palopoli.params as conf
import rospy as ros
import rospkg
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import *
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent
import time
from pprint import pprint
from vision.srv import *
import json
import random
np.set_printoptions(precision=4, suppress=True)


altezza_tavolo = -0.935+0.05
class2name = {0: 'X1-Y1-Z2', 1: 'X1-Y2-Z1', 2:'X1-Y2-Z2', 3: 'X1-Y2-Z2-CHAMFER', 4: 'X1-Y2-Z2-TWINFILLET', 5: 'X1-Y3-Z2',
              6: 'X1-Y3-Z2-FILLET', 7: 'X1-Y4-Z1', 8: 'X1-Y4-Z2', 9: 'X2-Y2-Z2', 10: 'X2-Y2-Z2-FILLET'}
class2dimensions = {0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                    4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                    8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057]}
IN_PIEDI = 'in_piedi'; SDRAIATO = 'sdraiato'; SOTTOSOPRA = 'sottosopra'; NORMALE = 'normale'


class Block:
    def __init__(self, classe=None, position=None, rpy=None):
        self.name = ''
        self.classe = classe
        self.position = position
        self.rpy = rpy
        self.processed = False

        if classe is not None and position is not None and rpy is not None:
            self.update()
        
    def update(self):
        [l, L, h] = class2dimensions[self.classe]

        # blocco in piedi (sulla faccia piccola)
        if pi/2-0.01 < self.rpy[0] < pi/2+0.01:
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+L/2])
            self.configuration = IN_PIEDI
            self.height = L
            self.radius = max(l,h)/2
        
        # blocco sul fianco (sulla faccia grande)    
        elif pi/2-0.01 < self.rpy[1] < pi/2+0.01:
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+l/2])
            self.configuration = SDRAIATO
            self.height = l
            self.radius = max(L,h)/2

        else:
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+h/2])
            self.height = h
            self.radius = max(l,L)/2

            if pi-0.01 < self.rpy[1] < pi+0.01:  # blocco sottosopra
                self.configuration = SOTTOSOPRA

            else:  # blocco in posizione normale
                self.configuration = NORMALE

        self.top = altezza_tavolo + self.height
        self.distance_from_shoulder = sqrt((self.position[0]-0.5)**2+(self.position[1]-0.35)**2)

        # assegno il quadrante in base alla posizione per poi gestire in modo diverso i vari casi
        if self.position[0] > 0.5 and self.position[1] <= 0.475:
            self.quadrant = 1
            if self.classe in [0, 9] and self.configuration == SOTTOSOPRA:
                self.rpy += np.array([0, 0, pi/2])
        
        elif self.position[0] > 0.5 and self.position[1] > 0.475:
            self.quadrant = 2
            if self.classe in [0, 9] and self.configuration == SOTTOSOPRA:
                self.rpy += np.array([0., 0., pi])
        
        elif self.position[0] <= 0.5 and self.position[1] > 0.475:
            self.quadrant = 3
            if self.classe in [0, 9] and self.configuration == SOTTOSOPRA:
                self.rpy += np.array([0., 0., pi])
        
        else:
            self.quadrant = 4
            if self.classe in [0, 9] and self.configuration == SOTTOSOPRA:
                self.rpy += np.array([0, 0, pi])

        print('!!! blocco aggiornato !!!')
        print(self)
        print('------------------------')

    def computeApproachAndLandPose(self, xy_land_pos=None, final_rpy=None, z_offset=0, y_approach_angle=90):
        approach_rpy = np.array([pi, 0., self.rpy[2]+pi/2])
        
        if final_rpy is None:
            final_rpy = self.getFinalRPY()
        
        land_rpy = np.array([pi, 0., final_rpy[2]+pi/2])
        
        # se il blocco è in posizione naturale o lo si vuole prendere con gripper perpendicolare al tavolo
        if self.configuration == NORMALE or y_approach_angle == 90:
            approach_rotm = eul2rotm(approach_rpy)
            approach_pos = self.position
            
            # correzione posizione Z per blocchi non X2-Yx-Zx
            if self.configuration == NORMALE and self.classe not in [9, 10]:
                approach_pos += np.array([0, 0, -0.008])

            # se voglio prendere il blocco con il gripper perpendicolare al tavolo ma il blocco è in piedi devo cambiare la quota Z a cui prenderlo
            if y_approach_angle == 90 and self.configuration == IN_PIEDI:
                approach_pos[2] = self.top - 0.03
                if self.classe == 0:
                    approach_pos[2] = self.top - 0.015
            
            # per prendere il blocco dal lato largo invece di quello corto
            if self.classe in [1, 2, 3, 4] and self.configuration == SDRAIATO:
                approach_rpy += np.array([0, 0, -pi/2])
            
            land_rotm = eul2rotm(land_rpy)
            
            if xy_land_pos is None:
                land_pos = np.array([self.position[0], self.position[1], approach_pos[2]])
            else:
                land_pos = np.array([xy_land_pos[0], xy_land_pos[1], approach_pos[2]])
            
            land_pos_no_correction = land_pos

        # se l'angolo di approccio è 45° -> si vuole girare il blocco
        elif y_approach_angle == 45:
            theta = pi/4
            
            if self.configuration == IN_PIEDI:
                # per far piegare il braccio sempre verso l'esterno
                if self.quadrant in [1, 2]:
                    land_rpy += np.array([0., 0., -pi])
                
                # offset nella direzione z del blocco - per evitare che il gripper collida con il blocco
                d = 0.
                if self.classe == 9:
                    d = 0.012
                elif self.classe == 10:
                    d = 0.01
                
                correction_approach45 = np.array([sin(self.rpy[2])*d, -cos(self.rpy[2])*d, 0])
                correction_land45 = np.array([0, 0, d])
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][2]/2 + 0.004
            
            elif self.configuration == SDRAIATO:
                d = 0.01
                correction_approach45 = np.array([0., 0., d])
                correction_land45 = np.array([cos(final_rpy[2])*d, sin(final_rpy[2])*d, 0.004])

                # per ruotare in un colpo solo le classi X1-Y2-Z*
                if self.classe in [1, 2, 3, 4]:
                    approach_rpy += np.array([0., 0., pi/2])
                    land_rpy += np.array([0., 0., pi/2])
                    correction_land45 = np.array([-cos(final_rpy[2])*d, sin(final_rpy[2])*d, 0.004])
                
                elif self.classe not in [3, 4, 6, 10]:  # blocchi quadrati e rettangolari
                    if not pi/2 <= self.rpy[2] <= 3/2*pi:
                        theta = -pi/4
                    
                    # per far piegare il braccio sempre verso l'esterno
                    land_rpy += np.array([0., 0., -pi/2])
                    correction_land45 = np.array([np.sign(theta)*cos(final_rpy[2])*d, np.sign(theta)*sin(final_rpy[2])*d, 0.004])
                
                else:  # tutti gli altri blocchi
                    land_rpy += np.array([0., 0., -pi/2])
                
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][1]/2
            
            else:  # sottosopra
                correction_approach45 = np.array([0., 0., 0.])
                
                d = 0.
                if self.classe in [9, 10]:
                    d = 0.025
                    correction_approach45 = np.array([0., 0., d])
                
                correction_land45 = np.array([-d*sin(final_rpy[2]), d*cos(final_rpy[2]), 0.004])
                
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][1]/2
            
            approach_pos = self.position + correction_approach45

            approach_rotm90 = eul2rotm(approach_rpy)
            approach_rotm = approach_rotm90 @ rotY(theta)
            
            land_pos_no_correction = np.array([self.position[0], self.position[1], land_Z_pos])
            if xy_land_pos is not None:
                land_pos_no_correction = np.array([xy_land_pos[0], xy_land_pos[1], land_Z_pos])
            land_pos = land_pos_no_correction + correction_land45
            
            land_rotm90 = eul2rotm(land_rpy)
            land_rotm = land_rotm90 @ rotY(-theta)

        self.approach_pos = approach_pos
        self.approach_rotm = approach_rotm
        self.land_pos = land_pos + np.array([0., 0., z_offset])
        self.land_rotm = land_rotm

        # print('approach pos: ', approach_pos)
        # print('approach rpy: ', approach_rpy)
        # print('land  rpy: ', land_rpy)
        # print('final rpy: ', final_rpy)
        
        self.final_pos = land_pos_no_correction
        self.final_rpy = final_rpy

    def autoUpdate(self):
        self.position = self.final_pos
        self.rpy = self.final_rpy

        self.update()

    def getFinalRPY(self):
        if not -0.01 < self.rpy[1] < 0.01:  # blocco sdraiato o sottosopra
            roll = pi/2
            yaw = pi

            if pi/2-0.01 < self.rpy[1] < pi/2+0.01:  # blocco sdraiato
                # per ruotare in un colpo solo le classi X1-Y2-Zx-xxx
                if self.classe in [1, 2, 3, 4]:
                    roll = 0
                    if self.quadrant == 1:
                        yaw = pi
                    elif self.quadrant == 4:
                        yaw = 0
                    else:
                        yaw = pi/2
            
            else:  # blocco sottosopra
                if self.quadrant in [1, 2]:
                    yaw = 3*pi/2
                else:
                    yaw = pi/2
            
            final_rpy = np.array([roll, 0, yaw])

        elif not -0.01 < self.rpy[0] < 0.01:  # blocco in piedi
            final_rpy = np.array([0., 0., pi/2])
        
        else:  # blocco normale
            final_rpy = np.array([0., 0., pi/2])
        
        return final_rpy

    def __str__(self):
        return '---- Block info -----' + '\nclass: ' + str(class2name[self.classe]) + '\nposition: ' + str(self.position) + \
                '\nrpy: ' + str(self.rpy) + '\nheight: ' + str(self.height) + '\nconfiguration: ' + self.configuration + '\n---------------------'
                #'\nspan: x: ' + str([self.xmin, self.xmax]) + ' - y: ' + str([self.ymin, self.ymax]) + ' -> ' + str([round(self.xmax-self.xmin, 4), round(self.ymax-self.ymin, 4)]) + \

    def __repr__(self):
        return str(self)


class JointStatePublisher:

    def __init__(self):
        self.robot_name = 'ur5'
        self.q = np.zeros(6)
        self.jstate = np.zeros(6)
        self.frame_name = conf.robot_params[self.robot_name]['ee_frame']
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        self.homing_position = conf.robot_params[self.robot_name]['q_0']

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        if self.real_robot:  # se usiamo il real_robot, il gripper non si controlla da ROS
            self.gripper = False
        else:
            if conf.robot_params[self.robot_name]['gripper_sim']:
                self.gripper = True
                self.soft_gripper = conf.robot_params[self.robot_name]['soft_gripper']
                self.use_grasp_plugin = conf.robot_params[self.robot_name]['use_grasp_plugin']
                if self.soft_gripper:
                    self.gripper_joint_names = conf.robot_params[self.robot_name]['soft_gripper_joint_names']
                    self.q_gripper = np.zeros(2)
                else:
                    self.gripper_joint_names = conf.robot_params[self.robot_name]['gripper_joint_names']
                    self.q_gripper = np.zeros(3)
            else:
                self.gripper = False
                self.use_grasp_plugin = False
        
        # topic del plugin gazebo grasp plugin ros su cui vengono pubblicati gli eventi di grasping (per ora non necessario)
        if self.use_grasp_plugin:
            self.grasp_event = ros.Subscriber('/grasp_event_republisher/grasp_events', GazeboGraspEvent, self.grasp_manager) 

        self.pub_des_jstate = ros.Publisher("/ur5/joint_group_pos_controller/command", Float64MultiArray, queue_size=10)
        if self.real_robot:
            self.pub_traj_jstate = ros.Publisher("/ur5/scaled_pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        else:
            #self.pub_traj_jstate = ros.Publisher("/ur5/vel_joint_traj_controller/command", JointTrajectory, queue_size=10)
            self.pub_traj_jstate = ros.Publisher("/ur5/pos_joint_traj_controller/command", JointTrajectory, queue_size=10)
        self.sub_jstate = ros.Subscriber("/ur5/joint_states", JointState, callback=self.receiveJstate, queue_size=1)

        self.vel_limits = conf.robot_params[self.robot_name]['vel_limit']
        self.t = 0

        if conf.robot_params['ur5']['world_name'] == 'tavolo_brick.world':
            self.attach_srv = ros.ServiceProxy('/link_attacher_node/attach', Attach)
            self.attach_srv.wait_for_service()
            self.detach_srv = ros.ServiceProxy('/link_attacher_node/detach', Attach)
            self.detach_srv.wait_for_service()
            self.setstatic_srv = ros.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
            self.setstatic_srv.wait_for_service()
            print('----- link attacher service started -----')
        
        self.vision = conf.robot_params[self.robot_name]['vision']
        if self.vision:
            self.vision_service = ros.ServiceProxy('vision_service', vision)
            self.vision_service.wait_for_service()
            print('----- vision server started ------')

        self.block = Block()
        self.present_blocks = []

    def sendDesJstate(self, q_des, q_des_gripper):
        # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
        msg = Float64MultiArray()
        if self.gripper and not self.real_robot:
            msg.data = np.append(q_des, q_des_gripper)
            # print(msg.data)  # aperto: 0.45, chiuso: -0.25
        else:
            msg.data = q_des
        self.pub_des_jstate.publish(msg)

    def sendDesTrajectory(self, positions, velocities, durations):
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

    def receiveJstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
            if self.gripper:
                for joint_idx in range(len(self.gripper_joint_names)):
                    if self.gripper_joint_names[joint_idx] == msg.name[msg_idx]:
                        self.q_gripper[joint_idx] = msg.position[msg_idx]

    def registerBlocks(self, vision_results):
        for i in range(0, vision_results.n_res):
            self.present_blocks.append(Block(vision_results.classe[i], [vision_results.x_center[i], vision_results.y_center[i]], [vision_results.roll[i], vision_results.pitch[i], vision_results.yaw[i]]))
        
        models = ros.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        
        for i in range(len(models.name)):
            if 'brick' in models.name[i]:
                model_pos = [models.pose[i].position.x, models.pose[i].position.y]

                for block in self.present_blocks:
                    if block.position[0]-0.1 < model_pos[0] < block.position[0]+0.1 and block.position[1]-0.1 < model_pos[1] < block.position[1]+0.1:
                        block.name = models.name[i]
    
    def homingProcedure(self):
        print('homing procedure')
        poss, vels, times = jcubic(self.q, self.homing_position)
        
        if not self.real_robot:
            if vels is None:
                for i in range(len(poss)):
                    poss[i] = np.append(poss[i], self.q_gripper)
            else:
                for i in range(len(poss)):
                    poss[i] = np.append(poss[i], self.q_gripper)
                    vels[i] = np.append(vels[i], np.zeros(len(self.q_gripper)))
        
        times = [t + 0.1 for t in times]

        time.sleep(1.)
        self.sendDesTrajectory(poss, vels, times)

        while np.linalg.norm(self.q - self.homing_position) > 0.005:
            pass

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

    def getGripPositions(self):
        if self.block.configuration == SDRAIATO:
            class2gripsize = {0: [70, 33], 1: [85, 73], 2: [95, 73], 3: [95, 72], 4: [95, 73], 5: [95, 70],
                              6: [95, 67], 7: [70, 45], 8: [95, 70], 9: [95, 67], 10: [95, 67]}
        elif self.block.configuration == SOTTOSOPRA:
            class2gripsize = {0: [70, 35], 1: [70, 35], 2: [70, 35], 3: [70, 35], 4: [70, 35], 5: [70, 35],
                              6: [70, 35], 7: [70, 35], 8: [70, 35], 9: [100, 83], 10: [100, 79]}
        elif self.block.configuration == IN_PIEDI:
            class2gripsize = {0: [70, 35], 1: [70, 35], 2: [70, 36], 3: [70, 35], 4: [70, 35], 5: [70, 36],
                              6: [70, 35], 7: [70, 35], 8: [70, 35], 9: [100, 85], 10: [100, 78]}
        else:  # blocco in posizione normale
            class2gripsize = {0: [70, 33], 1: [70, 33], 2: [70, 33], 3: [70, 33], 4: [70, 33], 5: [70, 33],
                              6: [70, 33], 7: [70, 33], 8: [70, 33], 9: [100, 75], 10: [100, 72]}
        
        return class2gripsize[self.block.classe]

    def ungripping(self, gripper_pos, attach_to_table=False):
        q_gripper = self.mapGripperJointState(gripper_pos)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [1])
        
        if attach_to_table:
            print('attaching to ground plane')
            self.attach_srv(
                model_name_1 = "my_ground_plane",
                link_name_1 = "link",
                model_name_2 = self.block.name,
                link_name_2 = "link",
            )

        print('detaching block')
        self.detach_srv(
            model_name_1 = "ur5",
            link_name_1 = "wrist_3_link",
            model_name_2 = self.block.name,
            link_name_2 = "link"
        )

        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.1:
            pass

    def gripping(self, gripper_pos):
        input('.,.,.,.,.')
        print('attaching block')
        q_gripper = self.mapGripperJointState(gripper_pos+2)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [2.])
        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
            pass
        time.sleep(0.3)

        self.attach_srv(
            model_name_1 = "ur5",
            link_name_1 = "wrist_3_link",
            model_name_2 = self.block.name,
            link_name_2 = "link"
        )
        time.sleep(0.3)

    def moveTo(self, final_pos, final_rotm, gripper_pos, wait_for_end=False, curve_type='bezier', vel=1, use_IK=False):
        if use_IK:
            print('----- USING IK -----')
            poss, vels, times = IKTrajectory(self.jstate, final_pos+np.array([-0.5, -0.35, 0]), final_rotm, curve_type=curve_type, vel=vel)
        else:
            poss, vels, times = differential_kin(self.jstate, final_pos+np.array([-0.5, -0.35, 0]), final_rotm, curve_type=curve_type, vel=vel)
        
        vels = None
        final_jstate = poss[-1]
        
        if final_jstate[5] > 2*pi:
            for p in poss:
                p[5] = p[5] - 2*pi
        if final_jstate[5] < -2*pi:
            for p in poss:
                p[5] = p[5] + 2*pi

        if not self.real_robot:
            final_q_gripper = self.mapGripperJointState(gripper_pos)
            if vels is None:
                for i in range(len(poss)):
                    poss[i] = np.append(poss[i], final_q_gripper)
            else:
                for i in range(len(poss)):
                    poss[i] = np.append(poss[i], final_q_gripper)
                    vels[i] = np.append(vels[i], np.zeros(len(self.q_gripper)))

        times = [t + 0.1 for t in times]
        time.sleep(2.)

        self.sendDesTrajectory(poss, vels, times)

        self.jstate = final_jstate.copy()

        if wait_for_end:
            while np.linalg.norm(self.jstate - self.q) > 0.005:
                pass

    def pickAndPlaceBlock(self, final_pos, final_rpy, z_offset=0, attach_to_table=True):
        # block_pos: coordinate x,y del centro del blocco e z come altezza a cui gripparlo
        # block_phi: euler angles che descrivono l'orientazione del blocco
        # final_pos: posizione x,y,z finale dove deve venire posizionato il blocco (z può variare in base a che altezza della torre viene posizionato)
        # final_phi: orientazione finale del blocco
        # initial_jstate: posizione iniziale dei joint del braccio (default a None -> homing position)
        # attach_to_table: crea un joint virtuale tra il blocco (nella posizione finale) e il tavolo da lavoro (attraverso gazebo_ros_link_attacher plugin)
        
        print('moving robot to desired position: ', self.block.position)

        gripper_opened, gripper_closed = self.getGripPositions()
        
        self.block.computeApproachAndLandPose(final_pos, final_rpy, z_offset)

        print('------- moving to block --------')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_opened)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.5)

        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:    
            self.gripping(gripper_closed)
        
        print('------- moving upwards --------')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_closed, curve_type='line')

        print('------ moving above final position: -------')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.10]), self.block.land_rotm, gripper_closed)

        print('------ moving to final position: ', final_pos, ' -------')
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.3)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.ungripping(gripper_opened, attach_to_table)
        
        self.block.autoUpdate()

        print('------- moving upwards --------')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.15]), self.block.land_rotm, gripper_opened, curve_type='line')
        # self.zeroWrist()

        if attach_to_table:
            print('setting ', self.block.name, ' static')
            self.setstatic_srv(
                model_name = self.block.name,
                link_name = "link",
                set_static = True
            )

    def ruotaBlocco(self, new_block_rpy, new_block_pos=None):
        gripper_opened, gripper_closed = self.getGripPositions()

        if new_block_pos is None:
            self.block.computeApproachAndLandPose(self.block.position, new_block_rpy)
            curve_type = 'line'
        else:
            self.block.computeApproachAndLandPose(new_block_pos, new_block_rpy)
            curve_type = 'bezier'

        print('----- moving to block -----')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.approach_rotm, gripper_opened, wait_for_end=True)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:
            self.gripping(gripper_closed)

        print('----- rotating block -----')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)
        self.moveTo(self.block.land_pos+np.array([0,0,0.05]), self.block.land_rotm, gripper_closed, curve_type=curve_type)
        self.moveTo(self.block.land_pos+np.array([0,0,0.002]), self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.ungripping(gripper_opened)
        
        self.block.autoUpdate()
        
        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_opened, curve_type='line')

    def preparaBloccoPerRotazione(self):
        # per alcune zone del tavolo, per il braccio è impossibile girare il blocco in un movimento solo, 
        # quindi per certe aree dobbiamo preparare il blocco effettuando una rotazione aggiuntiva

        # se il blocco è nella zona critica, lo sposto in una posizione più comoda per girarlo
        if self.block.distance_from_shoulder < 0.3:
            if self.block.configuration == IN_PIEDI:
                new_block_rpy = np.array([pi/2, 0., pi])
            elif self.block.configuration == SDRAIATO:
                new_block_rpy = np.array([0., pi/2, pi])
            elif self.block.configuration == SOTTOSOPRA:
                new_block_rpy = np.array([0., pi, pi/2])
            
            x, y = self.findFreeSpot()
            new_block_pos = np.array([x, y, 0])
            self.ruotaBlocco(new_block_rpy, new_block_pos)

        else:
            # divido i casi per posizione sul tavolo
            if self.block.quadrant == 1:
                # controllo se come è posizionato da problemi
                if not pi/2-0.1 <= self.block.rpy[2] <= 3/2*pi+0.1:
                    print('**** preparazione blocco ****')

                    if self.block.configuration == IN_PIEDI:
                        new_block_rpy = np.array([pi/2, 0, pi])
                        self.ruotaBlocco(new_block_rpy)

                    # se il blocco è sul fianco (pitch=pi/2)
                    elif self.block.configuration == SDRAIATO: 
                        # per la classe 3, dato che viene preso dal lato lungo, bisogna fare un check diverso sulla posizione
                        if not -0.1 < self.block.rpy[2] < pi/2+0.1 and self.block.classe == 3:
                            new_block_rpy = np.array([0, pi/2, pi/2])
                            self.ruotaBlocco(new_block_rpy)
                        elif self.block.classe in [4, 6, 10]:  # se NON è rettangolare/quadrato
                            new_block_rpy = np.array([0, pi/2, pi])
                            self.ruotaBlocco(new_block_rpy)
                    
                    # blocco sottosopra
                    elif self.block.configuration == SOTTOSOPRA:
                        if self.block.classe in [0, 9]:
                            self.block.rpy = np.array([0, pi, pi+self.block.rpy[2]])
                        elif self.block.classe in [1, 2, 5, 7, 8]:
                            self.block.rpy = np.array([0, pi, pi+self.block.rpy[2]])
                        else:
                            new_block_rpy = np.array([0, pi, pi])
                            self.ruotaBlocco(new_block_rpy)

            elif self.block.quadrant == 4:
                # controllo se come è posizionato da problemi
                if not pi/2-0.1 < self.block.rpy[2] < 3*pi/2+0.1:
                    print('**** preparazione blocco ****')
                    
                    if self.block.configuration == IN_PIEDI:
                        new_block_rpy = np.array([pi/2, 0, pi])
                        self.ruotaBlocco(new_block_rpy)
                    
                    # se il blocco è sul fianco (pitch=pi/2) e NON è rettangolare/quadrato
                    elif self.block.configuration == SDRAIATO:
                        # per la classe 3, dato che viene preso dal lato lungo, bisogna fare un check diverso sulla posizione
                        if not -0.1 < self.block.rpy[2] < pi/2+0.1 and self.block.classe == 3:
                            new_block_rpy = np.array([0, pi/2, pi/2])
                            self.ruotaBlocco(new_block_rpy)
                        elif self.block.classe in [4, 6, 10]:
                            new_block_rpy = np.array([0, pi/2, pi])
                            self.ruotaBlocco(new_block_rpy)
        
                    # blocco sottosopra
                    elif self.block.configuration == SOTTOSOPRA:
                        # se il blocco è quadrato, non c'è bisogno di correggere la posizione, basta solo cambiare il valore in cui pensa di essere
                        if self.block.classe in [0, 9]:
                            self.block.rpy = np.array([0, pi, pi/2+self.block.rpy[2]])
                        elif self.block.classe in [1, 2, 5, 7, 8]:
                            self.block.rpy = np.array([0, pi, pi+self.block.rpy[2]])
                        else:
                            new_block_rpy = np.array([0, pi, 0])
                            self.ruotaBlocco(new_block_rpy)
    
    def ruotaBloccoInPosizioneNormale(self, landing_pos=None, landing_rpy=None):
        gripper_opened, gripper_closed = self.getGripPositions()
        
        self.preparaBloccoPerRotazione()
        
        print('------- moving to block at 45° --------')
        if landing_pos is None and self.block.distance_from_shoulder < 0.35:
            x, y = self.findFreeSpot()
            landing_pos = np.array([x, y, 0])
        self.block.computeApproachAndLandPose(landing_pos, landing_rpy, y_approach_angle=45)

        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_opened)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.3)
        
        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:
            self.gripping(gripper_closed)
        
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)

        print('----- moving block to landing pos ', self.block.land_pos ,' -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.15]), self.block.land_rotm, gripper_closed, vel=0.5)
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.15)

        print('------- un-gripping --------')
        if self.gripper:
            self.ungripping(gripper_opened)

        self.block.autoUpdate()

        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.22]), self.block.land_rotm, gripper_opened, curve_type='line', vel=0.5)

    def check_collision(self, x, y):
        for b in self.present_blocks:
            if b.name != self.block.name:
                dist = sqrt((x - b.position[0])**2+(y - b.position[1])**2)
                if dist < (self.block.radius + b.radius + 0.07):
                    return True
        return False

    def findFreeSpot(self, castle_x_min=0.75, castle_y_min=0.6):
        x = random.uniform(0.07, 0.93)
        y = random.uniform(0.4, 0.73)
        dist_from_shoulder = math.sqrt((x-0.50)**2 + (y-0.35)**2)
        
        while dist_from_shoulder < 0.4 or (castle_x_min < x < 1. and castle_y_min < y < 0.8) or self.check_collision(x, y):
            x = random.uniform(0.07, 0.93)
            y = random.uniform(0.4, 0.73)
            dist_from_shoulder = math.sqrt((x-0.50)**2 + (y-0.35)**2)
        
        return (x,y)

    def getCastleBound(self, json_file):
        n = json_file["size"]

        castle_x_min, castle_y_min = 1., 1.
        for i in range(1, n+1):
            classe = list(class2name.keys())[list(class2name.values()).index(json_file[str(i)]["class"])]
            dims = class2dimensions[classe]
            x_c = json_file[str(i)]["x"]/100000
            y_c = json_file[str(i)]["y"]/100000
            yaw = get_yaw(json_file[str(i)]["r"])
            
            x_min, y_min = 0.98-x_c-dims[0]*cos(yaw)/2-dims[1]*sin(yaw)/2, 0.78-y_c-dims[1]*cos(yaw)/2-dims[0]*sin(yaw)/2

            castle_x_min, castle_y_min = min(x_min, castle_x_min), min(y_min, castle_y_min)
        
        return castle_x_min-0.07, castle_y_min-0.07

    def castle(self, json_file):
        # spazio occupato dal castello
        castle_x_max, castle_y_max = (0.99, 0.79)  
        castle_x_min, castle_y_min = self.getCastleBound(json_file)

        # controllo se blocchi sono nella zona finale del castello, se si li sposto
        for b in self.present_blocks:
            if castle_x_min < b.position[0] < 1. and castle_y_min < b.position[1] < 0.8:
                self.block = b
                self.block.update()

                (x, y) = self.findFreeSpot(castle_x_min, castle_y_min)
                self.ruotaBloccoInPosizioneNormale(landing_pos=np.array([x, y, 0]))

        n = json_file["size"]

        # posiziona ogni blocco in base alle direttive del json
        for iter in range(1, n+1):
            block_class = list(class2name.keys())[list(class2name.values()).index(json_file[str(iter)]["class"])]
            
            block_to_process = None
            for block in self.present_blocks:
                if block.classe == block_class and not block.processed:
                    block_to_process = block
            
            if not block_to_process:
                print('Nessun blocco presente della classe indicata!')
                continue

            self.block = block_to_process
            self.block.update()

            yaw = get_yaw(json_file[str(iter)]["r"])
            x_des = json_file[str(iter)]["x"]/100000
            y_des = json_file[str(iter)]["y"]/100000
            z_des = json_file[str(iter)]["z"]/100000
            
            # giro il blocco finchè non si trova in posizione normale
            while self.block.configuration != NORMALE:
                self.ruotaBloccoInPosizioneNormale()
            
            final_pos = np.array([castle_x_max-x_des, castle_y_max-y_des, 0])
            final_rpy = final_rpy = np.array([0., 0., yaw])
            self.pickAndPlaceBlock(final_pos, final_rpy, z_offset=z_des, attach_to_table=True)
            self.block.processed = True
        
        print('--- TASK FINISHED ---')

def readJSON():
    json_fd = open(os.path.join(os.path.expanduser("~"),"ros_ws","src","castle_build_path","output.json"))
    json_file = json_fd.read()
    json_fd.close()
    json_file = json.loads(json_file)

    return json_file

def get_yaw(rot):
    if rot == 0:
        return 0.
    elif rot == 1:
        return pi/2
    elif rot == 2:
        return pi
    else:
        return -pi/2

def talker(p):
    ros.init_node('custom_publisher_node', anonymous=True)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    p.jstate = p.q

    p.homingProcedure()
    input('.-.-.-')

    print('--- spawning blocks ---')
    json_file = readJSON()
    # spawnBlocks(json_file=json_file)
    # input('-+-+-')

    print("calling vision server...")
    vision_results = p.vision_service.call()
    print(vision_results)
    input('-+-+-')

    p.moveTo(np.array([0.5, 0.7, -0.6]), eul2rotm([pi, 0., 0.]), 60)

    p.registerBlocks(vision_results)
    
    p.castle(json_file)
    
    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
