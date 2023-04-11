#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim"))
from lab_exercises.lab_palopoli.kinematics import *
from robot_control.vision.scripts.SpawnBlocks_temp import spawnBlocks
import rospy as ros
import rospkg
import numpy as np
import lab_exercises.lab_palopoli.params as conf
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from gazebo_ros_link_attacher.srv import *
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent
from base_controllers.components.controller_manager import ControllerManager
import time
import cmath
from pprint import pprint
from vision.srv import *
import json
import random
from ros_impedance_controller.srv import set_pids
from ros_impedance_controller.srv import set_pidsRequest
from ros_impedance_controller.msg import pid
from gazebo_ros.gazebo_interface import GetModelProperties, GetJointProperties, GetLinkProperties, GetPhysicsProperties
np.set_printoptions(precision=4, suppress=True)


altezza_tavolo = -0.935+0.05
class2name = {0: 'X1-Y1-Z2', 1: 'X1-Y2-Z1', 2:'X1-Y2-Z2', 3: 'X1-Y2-Z2-CHAMFER', 4: 'X1-Y2-Z2-TWINFILLET', 5: 'X1-Y3-Z2',
              6: 'X1-Y3-Z2-FILLET', 7: 'X1-Y4-Z1', 8: 'X1-Y4-Z2', 9: 'X2-Y2-Z2', 10: 'X2-Y2-Z2-FILLET'}
class2dimensions = {0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                    4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                    8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057]}
configurations = ['in_piedi', 'sdraiato', 'sottosopra', 'normale']

def get_yaw(rot): #è corretto?
    if(rot==0):
        return pi
    elif (rot==2):
        return 0
    elif (rot==1):
        return pi/2
    else:
        return -pi/2

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
        [self.l, self.L, self.h] = class2dimensions[self.classe]

        # blocco in piedi (sulla faccia piccola)
        if pi/2-0.01 < self.rpy[0] < pi/2+0.01:
            s1 = abs(self.l*cos(self.rpy[2])) + abs(self.h*sin(self.rpy[2]))
            s2 = abs(self.l*sin(self.rpy[2])) + abs(self.h*cos(self.rpy[2]))
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+self.L/2])
            self.height = self.L
            self.configuration = configurations[0]
        
        # blocco sul fianco (sulla faccia grande)    
        elif pi/2-0.01 < self.rpy[1] < pi/2+0.01:
            s1 = abs(self.L*sin(self.rpy[2])) + abs(self.h*cos(self.rpy[2]))
            s2 = abs(self.L*cos(self.rpy[2])) + abs(self.h*sin(self.rpy[2]))
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+self.l/2])
            self.height = self.l
            self.configuration = configurations[1]

        else:
            s1 = abs(self.L*sin(self.rpy[2])) + abs(self.l*cos(self.rpy[2]))
            s2 = abs(self.L*cos(self.rpy[2])) + abs(self.l*sin(self.rpy[2]))
            self.position = np.array([self.position[0], self.position[1], altezza_tavolo+self.h/2])
            self.height = self.h
            
            # blocco sottosopra
            if pi-0.01 < self.rpy[1] < pi+0.01:
                self.configuration = configurations[2]

            # blocco in posizione normale
            else:
                self.configuration = configurations[3]

        [self.xmin, self.ymin] = np.round(np.array([self.position[0], self.position[1]]) - np.array([s1/2, s2/2]), 4)
        [self.xmax, self.ymax] = np.round(np.array([self.position[0], self.position[1]]) + np.array([s1/2, s2/2]), 4)

        # posizione Z (rispetto al BaseLink) del centro del corpo rettangolare del blocco
        # (può risultare comodo quando si deve prendere o appoggiare in posizione naturale)
        self.zpos = altezza_tavolo+class2dimensions[self.classe][0]/2
        self.top = altezza_tavolo+self.height

        # divido i casi per posizione sul tavolo
        if self.position[0] > 0.5 and self.position[1] <= 0.475:
            self.quadrant = 1
        elif self.position[0] > 0.5 and self.position[1] > 0.475:
            self.quadrant = 2
        elif self.position[0] <= 0.5 and self.position[1] > 0.475:
            self.quadrant = 3
        else:
            self.quadrant = 4

        print('!!! blocco aggiornato !!!')
        print(self)
        print('------------------------')

    def computeApproachAndLandPose(self, xy_land_pos, final_rpy, z_offset=0, y_approach_angle=90):
        approach_rpy = np.array([pi, 0, self.rpy[2]+pi/2])
        
        if final_rpy is None:
            final_rpy = self.getFinalRPY()
        
        land_rpy = np.array([pi, 0, final_rpy[2]+pi/2])
        
        print('approach rpy: ', approach_rpy, 'yaw: ', self.rpy[2]+pi/2)
        print('land rpy    : ', land_rpy, 'yaw: ', final_rpy[2]+pi/2)

        # se il blocco è in posizione naturale o lo si vuole prendere con gripper perpendicolare al tavolo
        if self.configuration == configurations[3] or y_approach_angle == 90:
            approach_rotm = eul2rotm(approach_rpy)
            approach_pos = self.position

            if self.configuration == configurations[3] and self.classe in [1, 7]:
                print('******* MODIFICA ALTEZZA BLOCCO BASSO *******')
                approach_pos += np.array([0, 0, -0.008])

            # se voglio prendere il blocco con il gripper perpendicolare al tavolo 
            # ma il blocco è in piedi devo cambiare la quota Z a cui prenderlo
            if y_approach_angle == 90 and self.configuration == configurations[0]:
                approach_pos[2] = self.top - 0.015
            
            land_rotm = eul2rotm(land_rpy)
            
            land_pos = np.array([self.position[0], self.position[1], approach_pos[2]])
            if xy_land_pos is not None:
                land_pos = np.array([xy_land_pos[0], xy_land_pos[1], approach_pos[2]])

        # se l'angolo di approccio è 45° -> si vuole girare il blocco
        elif y_approach_angle == 45:
            # blocco sulla faccia piccola - in piedi
            if self.configuration == configurations[0]:
                land_rpy += np.array([0, 0, -pi])
                # si lascia un po di spazio in piu atrimenti il gripper tocca il pezzo
                d = 0
                if self.classe == 9:
                    d = 0.016
                elif self.classe == 10:
                    d = 0.01
                correction_approach45 = np.array([sin(self.rpy[2])*d, -cos(self.rpy[2])*d, 0])
                correction_land45 = np.array([0, 0, d])
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][2]/2 + 0.0015
            
            # blocco sulla faccia grande - sdraiato
            elif self.configuration == configurations[1]:
                if self.classe in [1, 2, 3, 4]:
                    approach_rpy += np.array([0, 0, +pi/2])

                land_rpy += np.array([0, 0, -pi/2])
                d = 0.01
                correction_approach45 = np.array([0, 0, d])
                correction_land45 = np.array([cos(final_rpy[2])*d, sin(final_rpy[2])*d, 0.002])
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][1]/2
            
            # blocco sottosopra
            else:
                correction_approach45 = np.array([0, 0, 0])
                if self.classe in [9, 10]:
                    correction_approach45 = np.array([0, 0, 0.025])
                correction_land45 = np.array([0, 0, 0])
                land_Z_pos = altezza_tavolo + class2dimensions[self.classe][1]/2
            
            approach_pos = self.position + correction_approach45

            approach_rotm90 = eul2rotm(approach_rpy)
            theta = pi/4
            approach_rotm = approach_rotm90 @ rotY(theta)
            
            land_pos = np.array([self.position[0], self.position[1], land_Z_pos]) + correction_land45
            if xy_land_pos is not None:
                land_pos = np.array([xy_land_pos[0], xy_land_pos[1], land_Z_pos]) + correction_land45

            land_rotm90 = eul2rotm(land_rpy)
            land_rotm = land_rotm90 @ rotY(-theta)

        self.approach_pos = approach_pos
        self.approach_rotm = approach_rotm
        self.land_pos = land_pos + np.array([0, 0, z_offset])
        self.land_rotm = land_rotm
        self.final_rpy = final_rpy

    def autoUpdate(self):
        self.position = self.land_pos
        self.rpy = self.final_rpy

        self.update()

    def getFinalRPY(self):
        if not -0.01 < self.rpy[1] < 0.01:
            roll = pi/2
            if self.classe in [1,2,3,4]:
                roll = 0
            
            if self.quadrant in [1, 4]:
                final_rpy = np.array([roll, 0, pi])
            else:
                if self.position[0] < 0.5:
                    final_rpy = np.array([roll, 0, pi/2])
                else:
                    final_rpy = np.array([roll, 0, 3*pi/2])
    
        if not -0.01 < self.rpy[0] < 0.01:
            final_rpy = np.array([0, 0, pi/2])
        
        return final_rpy

    def __str__(self):
        return '---- Block info -----' + '\nclass: ' + str(class2name[self.classe]) + '\nposition: ' + str(self.position) + \
                '\nrpy: ' + str(self.rpy) + '\nheight: ' + str(self.height) + '\nconfiguration: ' + self.configuration + '\n---------------------'
                #'\nspan: x: ' + str([self.xmin, self.xmax]) + ' - y: ' + str([self.ymin, self.ymax]) + ' -> ' + str([round(self.xmax-self.xmin, 4), round(self.ymax-self.ymin, 4)]) + \

    def __repr__(self):
        return str(self)


class Block2:
    name = ''

    def __init__(self, classe, position, rpy):
        self.classe = classe
        self.rpy = rpy
        self.computeHeight()
        self.position = np.array([position[0], position[1], altezza_tavolo+self.height/2])

    def computeHeight(self):
        # blocco in piedi sulla faccia piccola
        if pi/2-0.1 < self.rpy[0] < pi/2+0.1:
            self.height = class2dimensions[self.classe][1]
            self.configuration = configurations[0]
        # blocco sdraiato sulla faccia grande
        elif pi/2-0.1 < self.rpy[1] < pi/2+0.1 or 3*pi/2-0.1 < self.rpy[1] < 3*pi/2+0.1:
            self.height = class2dimensions[self.classe][0]
            self.configuration = configurations[1]
        # blocco sottosopra o normale
        else:
            self.height = class2dimensions[self.classe][2]
            
            # blocco normale
            if -0.1 < self.rpy[0] < 0.1 and  -0.1 < self.rpy[1] < 0.1:
                self.configuration = configurations[3]
            # blocco sottosopra
            else:
                self.configuration = configurations[2]
        # TODO: gestire casi speciali per i fillet/chamfer

    def __str__(self):
        return '---- Block info -----' + '\nclass: ' + str(class2name[self.classe]) + '\nposition: ' + str(self.position) \
               + '\nrpy: ' + str(self.rpy) + '\nheight: ' + str(self.height) + '\n---------------------'

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
        if self.real_robot:  # se usiamo il real_robot, il gripper non si controlla da ROS ma con il programma del prof
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
        
        ros.wait_for_service('/gazebo/get_world_properties')
        self.get_world_properties = ros.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        ros.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = ros.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.vision = conf.robot_params[self.robot_name]['vision']
        if self.vision:
            self.vision_service = ros.ServiceProxy('vision_service', vision)
            self.vision_service.wait_for_service()
            print('----- vision server started ------')

        self.marker_pub = ros.Publisher('/vis', MarkerArray, queue_size=1)
        self.marker_array = MarkerArray()
        self.id = 0

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

    def getModelStates(self):
        models = ros.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        res = Res()
        for i in range(len(models.name)):
            if 'brick' in models.name[i]:
                block_type = models.name[i].split("_")[1]
                block_class = list(class2name.keys())[list(class2name.values()).index(block_type)]
                q1, q2, q3, q0 = models.pose[i].orientation.x, models.pose[i].orientation.y, models.pose[i].orientation.z, models.pose[i].orientation.w
                roll = np.round(math.atan2(2*(q0*q1+q2*q3), q0**2 - q1**2 - q2**2 + q3**2), 4)
                pitch = np.round(np.real(cmath.asin(2*(q0*q2-q1*q3))), 4)
                if pi/2-0.01 < pitch < pi/2+0.01 or -pi/2-0.01 < pitch < -pi/2+0.01:
                    roll = 0
                    yaw = np.round(-2*np.sign(pitch)*math.atan2(q1, q0), 4)
                else:
                    yaw = np.round(math.atan2(2*(q0*q3+q1*q2), q0**2 + q1**2 - q2**2 - q3**2), 4)

                if pi/2-0.01 < roll < pi/2+0.01:
                    if yaw < 0.:
                        yaw = 2*pi - yaw
                
                if -pi/2-0.01 < roll < -pi/2+0.01:
                    roll = pi/2
                    yaw = yaw + pi
                
                if pi-1. < roll < pi+1. or -pi-1. < roll < -pi+1.:
                    roll = 0
                    pitch = pi
                
                if pi-0.01 < pitch < pi+0.01:
                    yaw = yaw + pi
                
                yaw = yaw%(2*pi)

                res.classe.append(block_class)
                res.x_centre.append(np.round(models.pose[i].position.x, 4))
                res.y_centre.append(np.round(models.pose[i].position.y, 4))
                res.z_centre.append(np.round(models.pose[i].position.z, 4))
                res.angle.append([roll, pitch, yaw])

        res.n_res = len(res.x_centre)
        print(res.n_res)
        return res

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

    def graspManager(self, data):
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

    def getGripPositions(self):
        # blocco sdraiato sulla faccia grande
        if self.block.configuration == configurations[1]:
            class2gripsize = {0: [70, 33], 1: [70, 72], 2: [95, 72], 3: [95, 72], 4: [95, 72], 5: [95, 67],
                              6: [95, 67], 7: [70, 45], 8: [95, 68], 9: [95, 67], 10: [95, 67]}
        # blocco sottosopra
        elif self.block.configuration == configurations[2]:
            class2gripsize = {0: [70, 34], 1: [70, 34], 2: [70, 34], 3: [70, 34], 4: [70, 34], 5: [70, 34],
                              6: [70, 34], 7: [70, 34], 8: [70, 34], 9: [100, 79], 10: [100, 79]}
        # blocco in piedi sulla faccia piccola
        elif self.block.configuration == configurations[0]:
            class2gripsize = {0: [70, 35], 1: [70, 35], 2: [70, 35], 3: [70, 35], 4: [70, 35], 5: [70, 36],
                              6: [70, 35], 7: [70, 35], 8: [70, 35], 9: [100, 90], 10: [100, 78]}
        # blocco in posizione normale
        else:
            class2gripsize = {0: [70, 33], 1: [70, 33], 2: [70, 33], 3: [70, 33], 4: [70, 33], 5: [70, 33],
                              6: [70, 33], 7: [70, 33], 8: [70, 33], 9: [100, 73], 10: [100, 72]}
        
        return class2gripsize[self.block.classe]

    def ungripping(self, gripper_pos, attach_to_table):
        q_gripper = self.mapGripperJointState(gripper_pos)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        
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
        print('attaching block')
        input('..')
        q_gripper = self.mapGripperJointState(gripper_pos+2)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
            pass
        
        # q_gripper = self.mapGripperJointState(gripper_pos+20)
        # final_q = np.append(self.jstate, q_gripper)
        # self.sendDesTrajectory([final_q], None, [0.1])
        # print('waiting to reach gripper position')
        # while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
        #     pass

        # # self.moveTo(self.block.center+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_pos+20, curve_type='line')
        # # rotm90 = self.block.land_rotm @ rotZ(pi/2)
        # # self.moveTo(self.block.center+np.array([0, 0, 0.1]), rotm90, gripper_pos+20, curve_type='line')
        # # self.moveTo(self.block.center, self.block.land_rotm, gripper_pos+20, curve_type='line', wait_for_end=True)

        # q_gripper = self.mapGripperJointState(gripper_pos)
        # final_q = np.append(self.jstate, q_gripper)
        # self.sendDesTrajectory([final_q], None, [0.1])
        # print('waiting to reach gripper position')
        # while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
        #     pass

        self.attach_srv(
            model_name_1 = "ur5",
            link_name_1 = "wrist_3_link",
            model_name_2 = self.block.name,
            link_name_2 = "link"
        )
        time.sleep(0.3)

    def gripSim(self, grip, gripper_pos, attach_to_table=False):
        # procedura di gripping
        if grip:
            # TODO: aggiungere controllo su res.ok e gestire se va male 
            self.gripping(gripper_pos)

        # procedura di un-gripping
        else:
            self.ungripping(gripper_pos, attach_to_table)

    def moveTo(self, final_pos, final_rotm, gripper_pos, wait_for_end=False, curve_type='bezier', vel=1, use_IK=False):
        if use_IK:
            print('----- USING IK -----')
            poss, vels, times = IKTrajectory(self.jstate, final_pos+np.array([-0.5, -0.35, 0]), final_rotm, curve_type=curve_type, vel=vel)
            input('..')
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

        time.sleep(1.)
        times = [t + 0.1 for t in times]

        self.sendDesTrajectory(poss, vels, times)

        self.jstate = final_jstate.copy()

        if wait_for_end:
            while np.linalg.norm(self.jstate - self.q) > 0.005:
                pass

    def zeroWrist(self):
        if self.jstate[5] > pi:
            zeroWrist = np.zeros(8)
            zeroWrist[0:5] = self.jstate[0:5]
            jstate = np.append(self.jstate, self.q_gripper)
            traj = np.linspace(jstate, zeroWrist, 20)
            t = np.linspace(0, 2, 20)
            self.sendDesTrajectory(traj, None, t)
            while np.linalg.norm(self.q-zeroWrist[0:6]) > 0.005:
                pass
            self.jstate = self.q

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
            self.gripSim(True, gripper_closed)
        
        print('------- moving upwards --------')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_closed, curve_type='line')

        print('------ moving above final position: -------')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.10]), self.block.land_rotm, gripper_closed)

        print('------ moving to final position: ', final_pos, ' -------')
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.3)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.gripSim(False, gripper_opened, attach_to_table)
        
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

    def ruotaBlocco(self, new_block_rpy):
        gripper_opened, gripper_closed = self.getGripPositions()

        self.block.computeApproachAndLandPose(self.block.position, new_block_rpy)
        
        print('----- moving to block -----')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.approach_rotm, gripper_opened, wait_for_end=True)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:    
            self.gripSim(True, gripper_closed)

        print('----- rotating block -----')
        above_pos = self.block.approach_pos+np.array([0, 0, 0.05])
        self.moveTo(above_pos, self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)
        self.moveTo(above_pos, self.block.land_rotm, gripper_closed, curve_type='line')
        self.moveTo(self.block.land_pos+np.array([0,0,0.002]), self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.gripSim(False, gripper_opened)
        
        self.block.rpy[2] = new_block_rpy[2]
        self.block.update()
        
        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_opened, curve_type='line')
        # self.zeroWrist()

    def preparaBloccoPerRotazione(self):
        # per alcune zone del tavolo, per il braccio è impossibile girare il blocco in un movimento solo, 
        # quindi per certe aree dobbiamo preparare il blocco effettuando una rotazione aggiuntiva

        # divido i casi per posizione sul tavolo
        if self.block.quadrant == 1:

            # controllo se come è posizionato da problemi
            if not 3/4*pi <= self.block.rpy[2] <= 3/2*pi:
                print('**** preparazione blocco ****')

                # guardo se il blocco è in piedi (roll=pi/2):
                if self.block.configuration == configurations[0]:
                    new_block_rpy = np.array([pi/2, 0, pi])
                    self.ruotaBlocco(new_block_rpy)
            
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    if self.block.classe in [0, 9]:
                        self.block.rpy = np.array([0, pi, pi+self.block.rpy[2]])
                    else:
                        new_block_rpy = np.array([0, pi, pi])
                        self.ruotaBlocco(new_block_rpy)

        # elif self.block.quadrant == 2:
        #     # controllo se come è posizionato da problemi
        #     if pi/4 <= self.block.rpy[2] <= 5/4*pi:
        #         print('**** preparazione blocco ****')

        #         # guardo se il blocco è in piedi (roll=pi/2):
        #         if self.block.configuration == configurations[0]:
        #             new_block_rpy = np.array([pi/2, 0, 0])
        #             self.ruotaBlocco(new_block_rpy)
            
        #         # se il blocco è sul fianco (pitch=pi/2)
        #         elif self.block.configuration == configurations[1]:
        #             new_block_rpy = np.array([0, pi/2, 0])
        #             self.ruotaBlocco(new_block_rpy)
                
        #         # blocco sottosopra
        #         else:
        #             if self.block.classe in [0, 9]:
        #                 self.block.rpy = np.array([0, pi, 3*pi/2+self.block.rpy[2]])
        #             else:
        #                 new_block_rpy = np.array([0, pi, 0])
        #                 self.ruotaBlocco(new_block_rpy)
                
        # elif self.block.quadrant == 3:
                
        #     # controllo se come è posizionato da problemi
        #     if 3/4*pi <= self.block.rpy[2] <= 7/4*pi:
        #         print('**** preparazione blocco ****')
                    
        #         # guardo se il blocco è in piedi (roll=pi/2):
        #         if self.block.configuration == configurations[0]:
        #             new_block_rpy = np.array([pi/2, 0, 0])
        #             self.ruotaBlocco(new_block_rpy)
                
        #         # se il blocco è sul fianco (pitch=pi/2)
        #         elif self.block.configuration == configurations[1]:
        #             new_block_rpy = np.array([0, pi/2, 0])
        #             self.ruotaBlocco(new_block_rpy)
                
        #         # blocco sottosopra
        #         else:
        #             # se il blocco è quadrato, non c'è bisogno di correggere la posizione, basta solo cambiare il valore in cui pensa di essere
        #             if self.block.classe in [0, 9]:
        #                 self.block.rpy = np.array([0, pi, 3*pi/2+pi+self.block.rpy[2]])
        #             else:
        #                 new_block_rpy = np.array([0, pi, 0])
        #                 self.ruotaBlocco(new_block_rpy)
        elif self.block.quadrant == 4:
            # controllo se come è posizionato da problemi
            if not pi/2+0.01 < self.block.rpy[2] < 5/4*pi-0.01:
                print('**** preparazione blocco ****')
                
                # guardo se il blocco è in piedi (roll=pi/2):
                if self.block.configuration == configurations[0]:
                    new_block_rpy = np.array([pi/2, 0, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    # se il blocco è quadrato, non c'è bisogno di correggere la posizione, basta solo cambiare il valore in cui pensa di essere
                    if self.block.classe in [0, 9]:
                        self.block.rpy = np.array([0, pi, pi/2+self.block.rpy[2]])
                    else:
                        new_block_rpy = np.array([0, pi, 0])
                        self.ruotaBlocco(new_block_rpy)
        
    def ruotaBloccoInPosizioneNormale(self, landing_pos=None, landing_rpy=None):
        gripper_opened, gripper_closed = self.getGripPositions()

        # TODO: fare questa procedura solo quando serve (cioè per i blocchi fillet o chamfer)
        # perchè quando il blocco è sdraiato sul lato ed è rettangolare o quadrato, non serve farla
        self.preparaBloccoPerRotazione()
        
        print('------- moving to block at 45° --------')
        self.block.computeApproachAndLandPose(landing_pos, landing_rpy, y_approach_angle=45)

        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_opened)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.3)

        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:
            self.gripSim(True, gripper_closed)
        
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)
        input('...')

        print('----- moving block to landing pos ', self.block.land_pos ,' -----')
        # TODO: controllare che in destinazione non ci siano blocchi e in caso cercare una nuova posizione
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_closed, vel=0.5)
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.3)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.gripSim(False, gripper_opened)

        self.block.autoUpdate()

        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.22]), self.block.land_rotm, gripper_opened, curve_type='line', vel=0.5)
        
    def freeCastelSpace(self):
        print("-------------free castle space if needed-----------")
        for block in self.present_blocks:
            # controllo se il blocco è dentro l'aera riservata
            if 0.7 < block.xmin < 1. and 0.6 < block.ymin < 0.8 or 0.7 < block.xmax < 1. and 0.6 < block.ymax < 0.8:
                print("----block to remove-----")
                self.block = block
                self.block.update()

                new_block_rpy = np.array([0, 0, 0])
                landing_pos = self.findFreeSpot()
                print(landing_pos)
                landing_pos = np.array([landing_pos[0], landing_pos[1], 0])
                
                if self.block.configuration != configurations[3]:
                    self.ruotaBloccoInPosizioneNormale(landing_pos, new_block_rpy)
                else:
                    self.pickAndPlaceBlock(landing_pos, new_block_rpy, attach_to_table=False)
        
    def check_presence(self, x, y):
        for block in self.present_blocks:
            if x-0.05 < block.xmin < x-0.05 and y-0.05 < block.ymax < y+0.05:
                return block
        return None

    def findFreeSpot(self):
        x=0
        y=0
        raggio=0
        while(raggio<0.15 or self.check_presence(x,y) is not None or (0.7 < x < 1. and 0.6 < y < 0.8)):
            x=random.randint(5,95)/100 
            y=random.randint(25,75)/100
            raggio=math.sqrt((x-0.50)**2+(y-0.35)**2)
        return (x,y)


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

    def multipleBlocks(self, res):
        print(res)

        for i in range(res.n_res):
            b = Block('', res.classe[i], Point(x=res.xcentre[i], y=res.ycentre[i]), np.array([res.roll[i],res.pitch[i],res.yaw[i]]))
            self.present_blocks.append(b)

        input('...')

        height = 0
        final_tower_pos = np.array([0.8, 0.7, 0])
        final_tower_rpy = np.array([0, 0, 0])
        for i in range(res.n_res):
            self.block = self.present_blocks[i]

            # controllo se il blocchetto non è il posizione naturale (con la base sul tavolo)
            if not -0.01 < self.block.rpy[1] < 0.01:
                print('mettendo il blocco in piedi...')
                landing_pos = np.array([0.5, 0.75])
                landing_rpy = np.array([pi/2, 0, 0])
                self.ruotaBloccoInPosizioneNormale(landing_pos, landing_rpy)
            
            if not -0.01 < self.block.rpy[0] < 0.01:
                print('mettendo il blocco da verticale in posizione normale...')
                landing_pos = np.array([0.5, 0.75])
                landing_rpy = np.array([0, 0, pi/2])
                self.ruotaBloccoInPosizioneNormale(landing_pos, landing_rpy)

            print('spostando il blocco in posizione finale...')
            final_block_pos = final_tower_pos + np.array([0, 0, self.block.zpos+height])
            self.pickAndPlaceBlock(final_block_pos, final_tower_rpy, z_offset=height, attach_to_table=True)
            height = height + class2dimensions[self.block.classe][2] - 0.0195
            #input('Press enter to continue ....')
        
        print('***** TASK COMPLETED *****')

    def castle(self, json_file):
        castle_pos_max = (1., 0.7) #angolo in basso a sx del quadrato
        x_max, y_max = castle_pos_max

        n = json_file["size"]

        #posiziona ogni blocco in base alle direttive del json
        for iter in range(1, n+1):
            block_class = list(class2name.keys())[list(class2name.values()).index(json_file[str(iter)]["class"])]
            
            block_to_process = None
            for block in self.present_blocks:
                if block.classe == block_class and not block.processed:
                    block_to_process = block
            
            if not block_to_process:
                continue

            self.block = block_to_process
            self.block.update()

            yaw = get_yaw(json_file[str(iter)]["r"])
            x_des = json_file[str(iter)]["x"]/100000
            y_des = json_file[str(iter)]["y"]/100000
            z_des = json_file[str(iter)]["z"]/100000
            
            while self.block.configuration != configurations[3]:
                self.ruotaBloccoInPosizioneNormale()
            
            final_pos = np.array([x_max-x_des, y_max-y_des, 0])
            final_rpy = final_rpy = np.array([0, 0, yaw])
            self.pickAndPlaceBlock(final_pos, final_rpy, z_offset=z_des, attach_to_table=True)
            self.block.processed = True

def readJSON():
    json_fd = open(os.path.join(os.path.expanduser("~"),"ros_ws","src","castle_build_path","output.json"))
    json_file = json_fd.read()
    json_fd.close()
    json_file = json.loads(json_file)

    return json_file

class Res:
    def __init__(self):
        self.n = 0
        self.classe = []
        self.x_centre = []
        self.y_centre = []
        self.z_centre = []
        self.roll = []
        self.pitch = []
        self.yaw = []
    
    def find(self, classe):
        for i in range(0, self.n):
            if (self.classe[i]==classe):
                return i
        print("block not found")
        return -1
    
    def remove(self, index):
        self.n = self.n-1
        self.classe.pop(index)
        self.x_centre.pop(index)
        self.y_centre.pop(index)
        self.z_centre.pop(index)
        self.roll.pop(index)
        self.pitch.pop(index)
        self.yaw.pop(index)

    def __str__(self):
        out = '--- blocks info ----\n'
        for i in range(self.n):
            out += ' - [' + str(self.x_centre[i]) + ', ' + str(self.y_centre[i]) + ', ' + str(self.z_centre[i]) + '], rpy: [' + str(self.roll[i]) + str(self.pitch[i]) + str(self.yaw[i]) + '] class: ' + str(self.classe[i]) + '\n'
        return out
    
    def __repr__(self):
        return str(self)

## vision_results:
# int64 n_res
# int64[] classe
# float64[] x_center
# float64[] y_center
# float64[] roll
# float64[] pitch
# float64[] yaw
# bool[] processed

def takeBlock(p):
    # test con blocchi in piedi (sulla faccia piccola), del tipo [pi/2, 0, phi]
    for i, block in enumerate(p.present_blocks):
        p.block = block
        gripper_opened, gripper_closed = p.getGripPositions()

        block_pos_above = block.position + np.array([0, 0, 0.05])
        block_rotm = eul2rotm([pi, 0, block.rpy[2]+pi/2])
        p.moveTo(block_pos_above, block_rotm, gripper_opened, wait_for_end=True)

        block_rotm_45 = block_rotm @ rotY(pi/4)
        p.moveTo(block_pos_above, block_rotm_45, gripper_opened, wait_for_end=True, curve_type='line')

        p.moveTo(block.position, block_rotm_45, gripper_opened, wait_for_end=True, curve_type='line')
        input('grip?')
        p.gripSim(True, gripper_closed)

        p.moveTo(block_pos_above, block_rotm_45, gripper_closed, wait_for_end=True, curve_type='line')
        block_rotm_45 = eul2rotm([pi, 0, 0]) @ rotY(-pi/4)
        p.moveTo(block_pos_above, block_rotm_45, gripper_closed, wait_for_end=True, curve_type='line')


def talker(p):
    ros.init_node('custom_publisher_node', anonymous=True)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    p.jstate = p.q

    p.homingProcedure()
    input('...')
    
    print('--- spawning blocks ---')
    json_file = readJSON()
    spawnBlocks(json_file=json_file)
    
    input('...')

    print("vision server called")
    vision_results = p.vision_service.call()
    print(vision_results)
    input('..')

    p.moveTo(np.array([0.5, 0.7, -0.6]), eul2rotm([pi, 0, 0]), 40)

    p.registerBlocks(vision_results)
    # pprint(p.present_blocks)

    #takeBlock(p)
    
    #p.multipleBlocks(vision_results)

    p.castle(json_file)
    
    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
