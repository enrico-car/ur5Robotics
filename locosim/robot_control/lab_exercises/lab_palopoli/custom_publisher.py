#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim"))
from kinematics import *
from robot_control.vision.scripts.SpawnBlocks_temp import spawnBlocks
import rospy as ros
import rospkg
import numpy as np
import params as conf
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
from ros_impedance_controller.srv import set_pids
from ros_impedance_controller.srv import set_pidsRequest
from ros_impedance_controller.msg import pid
from gazebo_ros.gazebo_interface import GetModelProperties, GetJointProperties, GetLinkProperties, GetPhysicsProperties
np.set_printoptions(precision=4, suppress=True)


altezza_tavolo = -0.935
class2name = {0: 'X1-Y1-Z2', 1: 'X1-Y2-Z1', 2:'X1-Y2-Z2', 3: 'X1-Y2-Z2-CHAMFER', 4: 'X1-Y2-Z2-TWINFILLET', 5: 'X1-Y3-Z2',
              6: 'X1-Y3-Z2-FILLET', 7: 'X1-Y4-Z1', 8: 'X1-Y4-Z2', 9: 'X2-Y2-Z2', 10: 'X2-Y2-Z2-FILLET'}
class2dimensions = {0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                    4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                    8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057]}
configurations = ['in_piedi', 'sul_lato', 'sottosopra', 'normale']

def get_yaw(rot): #è corretto?
    if(rot==0):
        return 0
    elif (rot==2):
        return pi
    elif (rot==1):
        return pi/2
    else:
        return -pi/2

class Block:
    def __init__(self, name=None, class_=None, position=None, rpy=None, orientation=None):
        if name is not None:
            self.name = name

        if class_ is not None:
            self.class_ = class_
        
        self.position = position
        if position is not None:
            self.position = np.array([position.x, position.y])
        
        self.orientation = orientation
        if self.orientation is not None:
            self.rpy = quat2rpy(orientation.x, orientation.y, orientation.z, orientation.w)
        
        if rpy is not None:
            self.rpy = rpy.copy()
        
        if self.position is not None and self.rpy is not None:
            self.update()
        
    def update(self):
        [self.l, self.L, self.h] = class2dimensions[self.class_]

        # blocco in piedi (sulla faccia piccola)
        if pi/2-0.01 < self.rpy[0] < pi/2+0.01 or -pi/2-0.01 < self.rpy[0] < -pi/2+0.01:
            s1 = abs(self.l*cos(self.rpy[2])) + abs(self.h*sin(self.rpy[2]))
            s2 = abs(self.l*sin(self.rpy[2])) + abs(self.h*cos(self.rpy[2]))
            self.center = np.array([self.position[0], self.position[1], altezza_tavolo+self.L/2])
            self.height = self.L
            self.configuration = configurations[0]
        
        # blocco sul fianco (sulla faccia grande)    
        elif pi/2-0.01 < self.rpy[1] < pi/2+0.01:
            s1 = abs(self.L*sin(self.rpy[2])) + abs(self.h*cos(self.rpy[2]))
            s2 = abs(self.L*cos(self.rpy[2])) + abs(self.h*sin(self.rpy[2]))
            self.center = np.array([self.position[0], self.position[1], altezza_tavolo+self.l/2])
            self.height = self.l
            self.configuration = configurations[1]

        else:
            s1 = abs(self.L*sin(self.rpy[2])) + abs(self.l*cos(self.rpy[2]))
            s2 = abs(self.L*cos(self.rpy[2])) + abs(self.l*sin(self.rpy[2]))
            # blocco sottosopra
            if pi-0.01 < self.rpy[1] < pi+0.01:
                self.center = np.array([self.position[0], self.position[1], altezza_tavolo+self.h/2])
                self.height = self.h
                self.configuration = configurations[2]

            # blocco in posizione normale
            else:
                self.center = np.array([self.position[0], self.position[1], altezza_tavolo+self.h/2])
                print('centro: ', self.center)
                self.height = self.h
                self.configuration = configurations[3]
        
        if self.class_ == 9 or self.class_ == 10:
            self.center += np.array([0, 0, 0.01])

        [self.xmin, self.ymin] = np.round(self.position - np.array([s1/2, s2/2]), 4)
        [self.xmax, self.ymax] = np.round(self.position + np.array([s1/2, s2/2]), 4)

        # posizione Z (rispetto al BaseLink) del centro del corpo rettangolare del blocco
        # (può risultare comodo quando si deve prendere o appoggiare in posizione naturale)
        self.zpos = altezza_tavolo+class2dimensions[self.class_][0]/2
        self.top = altezza_tavolo+self.height

        if self.rpy[2] < 0:
            self.rpy[2] = 2*pi - self.rpy[2]

        np.round(self.position, 4)
        np.round(self.center, 4)
        np.round(self.rpy, 4)

        # divido i casi per posizione sul tavolo
        if 0.5 < self.position[0] < 1. and 0.15 < self.position[1] <= 0.475:
            self.quadrant = 1
        elif 0.5 < self.position[0] < 1. and 0.475 < self.position[1] < 0.8:
            self.quadrant = 2
        elif 0. < self.position[0] <= 0.5 and 0.475 < self.position[1] < 0.8:
            self.quadrant = 3
        else:
            self.quadrant = 4

        print('!!! blocco aggiornato !!!')
        print(self)
        print('------------------------')

    def computeApproachAndLandPose(self, xy_land_pos, final_rpy, z_offset=0, y_approach_angle=90):
        approach_rpy = np.array([pi, 0, (self.rpy[2]+pi/2)%(2*pi)])
        print('approach rpy: ', approach_rpy, 'yaw: ', self.rpy[2]+pi/2)
        land_rpy = np.array([pi, 0, (final_rpy[2]+pi/2)%(2*pi)])
        print('land rpy: ', land_rpy, 'yaw: ', final_rpy[2]+pi/2)

        # se il blocco è in posizione naturale o lo si vuole prendere con gripper perpendicolare al tavolo
        if self.configuration == configurations[3] or y_approach_angle == 90:
            approach_rotm = eul2rotm(approach_rpy)
            approach_pos = self.center

            if self.configuration == configurations[3] and self.class_ in [1, 7]:
                print('******* MODIFICA ALTEZZA BLOCCO BASSO *******')
                approach_pos += np.array([0, 0, -0.008])

            # se voglio prendere il blocco con il gripper perpendicolare al tavolo 
            # ma il blocco è in piedi devo cambiare la quota Z a cui prenderlo
            if y_approach_angle == 90 and self.configuration == configurations[0]:
                approach_pos[2] = self.top - 0.015
            
            final_rotm = eul2rotm(land_rpy)
            final_pos = np.array([xy_land_pos[0], xy_land_pos[1], approach_pos[2]])

        # se l'angolo di approccio è 45° -> si vuole girare il blocco
        elif y_approach_angle == 45:
            approach_rotm90 = eul2rotm(approach_rpy)

            theta = pi/4

            approach_rotm = approach_rotm90 @ rotY(theta)

            # blocco in piedi
            if self.configuration == configurations[0]:
                # si lascia un po di spazio in piu atrimenti il gripper tocca il pezzo
                if pi/2-0.01 < self.rpy[0] < pi/2+0.01:
                    land_rpy += np.array([0, 0, -pi])
                d = 0.01
                correction_approach45 = np.array([sin(self.rpy[2])*d, -cos(self.rpy[2])*d, 0])
                correction_land45 = np.array([0, 0, d])
                land_Z_pos = altezza_tavolo + class2dimensions[self.class_][2]/2
            
            # blocco sul fianco
            elif self.configuration == configurations[1]:
                land_rpy += np.array([0, 0, -pi/2])
                d = 0.01
                correction_approach45 = np.array([0, 0, d])
                correction_land45 = np.array([cos(final_rpy[2])*d, sin(final_rpy[2])*d, 0.002])
                land_Z_pos = altezza_tavolo + class2dimensions[self.class_][1]/2
            
            # blocco sottosopra
            else:
                correction_approach45 = np.array([0, 0, 0])
                correction_land45 = np.array([0, 0, 0])
                land_Z_pos = altezza_tavolo + class2dimensions[self.class_][1]/2
            
            approach_pos = self.center + correction_approach45

            theta = -theta
            land_rotm90 = eul2rotm(land_rpy)
            final_rotm = land_rotm90 @ rotY(theta)
            final_pos = np.array([xy_land_pos[0], xy_land_pos[1], land_Z_pos]) + correction_land45
        

        self.approach_pos = approach_pos
        self.approach_rotm = approach_rotm
        self.land_pos = final_pos + np.array([0, 0, z_offset])
        self.land_rotm = final_rotm
        print('approach pos: ', approach_pos)

    def __str__(self):
        return '---- Block info -----' + '\nclass: ' + str(class2name[self.class_]) + '\nposition: ' + str(self.position) + '\nspan: x: ' + str([self.xmin, self.xmax]) \
               + ' - y: ' + str([self.ymin, self.ymax]) + ' -> ' + str([round(self.xmax-self.xmin, 4), round(self.ymax-self.ymin, 4)]) + '\nrpy: ' + str(self.rpy) + \
               '\nheight: ' + str(self.height) + '\nconfiguration: ' + self.configuration + '\n---------------------'

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

                res.class_.append(block_class)
                res.x_centre.append(np.round(models.pose[i].position.x, 4))
                res.y_centre.append(np.round(models.pose[i].position.y, 4))
                res.z_centre.append(np.round(models.pose[i].position.z, 4))
                res.angle.append([roll, pitch, yaw])

        res.n_res = len(res.x_centre)
        print(res.n_res)
        return res

    def addMarker(self, pos, radius=0.1):
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

    def publishMarkers(self):
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
        if pi/2-0.01 < self.block.rpy[1] < pi/2+0.01:
            class2gripsize = {0: [70, 32], 1: [70, 45], 2: [95, 67], 3: [95, 67], 4: [95, 67], 5: [95, 67],
                              6: [95, 67], 7: [70, 45], 8: [95, 67], 9: [95, 67], 10: [95, 67]}
        # blocco sottosopra
        elif pi-0.01 < self.block.rpy[1] < pi+0.01:
            class2gripsize = {0: [70, 32], 1: [70, 32], 2: [70, 32], 3: [70, 32], 4: [70, 32], 5: [70, 32],
                              6: [70, 32], 7: [70, 32], 8: [70, 32], 9: [100, 74], 10: [100, 74]}
        # blocco in piedi sulla faccia piccola
        elif pi/2-0.01 < self.block.rpy[0] < pi/2+0.01:
            class2gripsize = {0: [70, 32], 1: [70, 32], 2: [70, 32], 3: [70, 32], 4: [70, 32], 5: [70, 32],
                              6: [70, 32], 7: [70, 32], 8: [70, 32], 9: [100, 74], 10: [80, 60]}
        # blocco in posizione normale
        else:
            class2gripsize = {0: [70, 32], 1: [70, 32], 2: [70, 32], 3: [70, 32], 4: [70, 32], 5: [70, 32],
                              6: [70, 32], 7: [70, 32], 8: [70, 32], 9: [100, 66], 10: [100, 71]}
        
        return class2gripsize[self.block.class_]

    def ungripping(self, gripper_pos, attach_to_table):
        q_gripper = self.mapGripperJointState(gripper_pos)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        
        if attach_to_table:
            print('attaching to ground plane')
            self.attach_srv(
                model_name_1 = "ground_plane",
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
        q_gripper = self.mapGripperJointState(gripper_pos)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
            pass
        
        q_gripper = self.mapGripperJointState(gripper_pos+20)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
            pass

        # self.moveTo(self.block.center+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_pos+20, curve_type='line')
        # rotm90 = self.block.land_rotm @ rotZ(pi/2)
        # self.moveTo(self.block.center+np.array([0, 0, 0.1]), rotm90, gripper_pos+20, curve_type='line')
        # self.moveTo(self.block.center, self.block.land_rotm, gripper_pos+20, curve_type='line', wait_for_end=True)

        q_gripper = self.mapGripperJointState(gripper_pos)
        final_q = np.append(self.jstate, q_gripper)
        self.sendDesTrajectory([final_q], None, [0.1])
        print('waiting to reach gripper position')
        while np.linalg.norm(self.q_gripper - q_gripper) > 0.005:
            pass

        self.attach_srv(
            model_name_1 = "ur5",
            link_name_1 = "wrist_3_link",
            model_name_2 = self.block.name,
            link_name_2 = "link"
        )
        time.sleep(0.3)

    def gripSim(self, grip, gripper_pos, attach_to_table=False):
        # TODO: migliorare questi if (magari usando tutto sotto self.block invece di usare altre due variabili, self.block_grasped e self.grasped_block_name)
        # procedura di gripping
        if grip:
            models = ros.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
            for i in range(len(models.name)):
                if 'brick' in models.name[i]:
                    model_pos = np.array([models.pose[i].position.x, models.pose[i].position.y])
                    # controllo per trovare il giusto blocco da attaccare al gripper
                    if np.linalg.norm(model_pos - self.block.position) < 0.04:
                        print('#### model found ####')
                        self.block.name = models.name[i]
            
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
        
        print('moving robot to desired position: ', self.block.center)

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
        
        self.block.position = np.array([self.block.land_pos[0], self.block.land_pos[1]])
        self.block.rpy[2] = final_rpy[2]
        self.block.update()

        print('------- moving upwards --------')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.15]), self.block.land_rotm, gripper_opened, curve_type='line')
        self.zeroWrist()

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
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.approach_rotm, gripper_opened)
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:    
            self.gripSim(True, gripper_closed)

        print('----- rotating block -----')
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.05]), self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line')
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.5)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.gripSim(False, gripper_opened)
        
        self.block.rpy[2] = new_block_rpy[2]
        self.block.update()
        
        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_opened, curve_type='line')
        self.zeroWrist()

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
                    new_block_rpy = np.array([0, 0, pi])
                    self.ruotaBlocco(new_block_rpy)
            
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    new_block_rpy = np.array([0, pi, pi])
                    self.ruotaBlocco(new_block_rpy)

        elif self.block.quadrant == 2:
            # controllo se come è posizionato da problemi
            if pi/4 <= self.block.rpy[2] <= 5/4*pi:
                print('**** preparazione blocco ****')

                # guardo se il blocco è in piedi (roll=pi/2):
                if self.block.configuration == configurations[0]:
                    new_block_rpy = np.array([0, 0, 0])
                    self.ruotaBlocco(new_block_rpy)
            
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, 0])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    new_block_rpy = np.array([0, pi, 0])
                    self.ruotaBlocco(new_block_rpy)
                
        elif self.block.quadrant == 3:
                
            # controllo se come è posizionato da problemi
            if 3/4*pi <= self.block.rpy[2] <= 7/4*pi:
                print('**** preparazione blocco ****')
                    
                # guardo se il blocco è in piedi (roll=pi/2):
                if self.block.configuration == configurations[0]:
                    new_block_rpy = np.array([0, 0, 0])
                    self.ruotaBlocco(new_block_rpy)
                
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, 0])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    new_block_rpy = np.array([0, pi, 0])
                    self.ruotaBlocco(new_block_rpy)
        else:
            # controllo se come è posizionato da problemi
            if not pi/2 < self.block.rpy[2] < 5/4*pi:
                print('**** preparazione blocco ****')
                
                # guardo se il blocco è in piedi (roll=pi/2):
                if self.block.configuration == configurations[0]:
                    new_block_rpy = np.array([0, 0, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # se il blocco è sul fianco (pitch=pi/2)
                elif self.block.configuration == configurations[1]:
                    new_block_rpy = np.array([0, pi/2, pi])
                    self.ruotaBlocco(new_block_rpy)
                
                # blocco sottosopra
                else:
                    new_block_rpy = np.array([0, pi, pi])
                    self.ruotaBlocco(new_block_rpy)
        
    def ruotaBloccoInPosizioneNormale(self, landing_pos, landing_rpy):
        gripper_opened, gripper_closed = self.getGripPositions()

        # TODO: fare questa procedura solo quando serve (cioè per i blocchi fillet o chamfer)
        # perchè quando il blocco è sdraiato sul lato ed è rettangolare o quadrato, non serve farla
        self.preparaBloccoPerRotazione()
        
        print('------- moving to block at 45° --------')
        self.block.computeApproachAndLandPose(landing_pos, landing_rpy, y_approach_angle=45)

        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_opened, curve_type='line')
        self.moveTo(self.block.approach_pos, self.block.approach_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.3)

        input('...')

        print('-------- gripping ', gripper_closed, ' ---------')
        if self.gripper:
            self.gripSim(True, gripper_closed)
        
        self.moveTo(self.block.approach_pos+np.array([0, 0, 0.15]), self.block.approach_rotm, gripper_closed, curve_type='line', vel=0.5)

        print('----- moving block to landing pos ', self.block.land_pos ,' -----')
        # TODO: controllare che in destinazione non ci siano blocchi e in caso cercare una nuova posizione
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.1]), self.block.land_rotm, gripper_closed, vel=0.5)
        self.moveTo(self.block.land_pos, self.block.land_rotm, gripper_closed, wait_for_end=True, curve_type='line', vel=0.3)
        
        print('------- un-gripping --------')
        if self.gripper:
            self.gripSim(False, gripper_opened)
        
        self.block.position = np.array([landing_pos[0], landing_pos[1]])
        self.block.rpy = landing_rpy.copy()
        self.block.update()

        print('----- moving up -----')
        self.moveTo(self.block.land_pos+np.array([0, 0, 0.15]), self.block.land_rotm, gripper_opened, wait_for_end=True, curve_type='line', vel=0.5)
        self.zeroWrist()

    def freeCastelSpace(self):
        for block in self.present_blocks:
            # controllo se il blocco è dentro l'aera riservata
            if 0.7 < block.xmin < 1. and 0.6 < block.ymin < 0.8 or 0.7 < block.xmax < 1. and 0.6 < block.ymax < 0.8:
                self.block = block
                new_block_approx_pos = np.array([0.5, 0.7])
                new_block_rpy = np.array([0, 0, 0])
                landing_pos = self.findFreeSpot(new_block_approx_pos, new_block_rpy)
                landing_pos = np.array([landing_pos[0], landing_pos[1], self.block.zpos])
                
                if self.block.configuration != configurations[3]:
                    self.ruotaBloccoInPosizioneNormale(landing_pos, new_block_rpy)
                else:
                    self.pickAndPlaceBlock(landing_pos, new_block_rpy, attach_to_table=False)

    def findFreeSpot(self, block_approx_pos, block_new_rpy):
        [l, L, h] = class2dimensions[self.block.class_]

        # blocco in piedi (sulla faccia piccola)
        if pi/2-0.01 < block_new_rpy[0] < pi/2+0.01 or -pi/2-0.01 < block_new_rpy[0] < -pi/2+0.01:
            s1 = abs(l*cos(block_new_rpy[2])) + abs(h*sin(block_new_rpy[2]))
            s2 = abs(l*sin(block_new_rpy[2])) + abs(h*cos(block_new_rpy[2]))
        
        # blocco sul fianco (sulla faccia grande)    
        elif pi/2-0.01 < block_new_rpy[1] < pi/2+0.01:
            s1 = abs(L*sin(block_new_rpy[2])) + abs(h*cos(block_new_rpy[2]))
            s2 = abs(L*cos(block_new_rpy[2])) + abs(h*sin(block_new_rpy[2]))

        else:
            s1 = abs(L*sin(block_new_rpy[2])) + abs(l*cos(block_new_rpy[2]))
            s2 = abs(L*cos(block_new_rpy[2])) + abs(l*sin(block_new_rpy[2]))

        s1 = s1*1.3
        s2 = s2*1.3
        [main_xmin, main_ymin] = np.round(block_approx_pos - np.array([s1/2, s2/2]), 4)
        [main_xmax, main_ymax] = np.round(block_approx_pos + np.array([s1/2, s2/2]), 4)

        print(main_xmin, main_xmax)
        print(main_ymin, main_ymax)
        print(self.present_blocks)

        while True:
            busy_intervals = []
            for i in range(1, len(self.present_blocks)):
                if block_approx_pos[1]-0.07 < self.present_blocks[i].position[1] and self.present_blocks[i].position[1] < block_approx_pos[1]+0.07:
                    busy_intervals.append((self.present_blocks[i].xmin, self.present_blocks[i].xmax))
            
            print('busy intervals:')
            print(busy_intervals)

            if len(busy_intervals) != 0:
                sorted(busy_intervals)
                free_intervals = []
                (xmin, xmax) = busy_intervals[0]
                free_intervals.append((0., xmin))
                for i in range(1, len(busy_intervals)):
                    (xmin1, xmax1) = busy_intervals[i-1]
                    (xmin2, xmax2) = busy_intervals[i]
                    free_intervals.append((xmax1, xmin2))
                (xmin, xmax) = busy_intervals[-1]
                free_intervals.append((xmax, 1.-0.15))
                print(free_intervals)

                for interval in free_intervals:
                    (xmin, xmax) = interval
                    if xmax - xmin > main_xmax - main_xmin:
                        block_approx_pos[0] = (xmax-xmin)/2
                        return block_approx_pos
                
                    block_approx_pos[1] += -0.03
            else:
                return block_approx_pos


            # for block in self.present_blocks:
            #     if block.name != self.block.name:
            #         dx_x = block.xmin <= xmin <= block.xmax
            #         sx_x = block.xmin <= xmax <= block.xmax
            #         cx_x = (xmin <= block.xmin and xmax >= block.xmax)
            #         dx_y = block.ymin <= ymin <= block.ymax
            #         sx_y = block.ymin <= ymax <= block.ymax
            #         cx_y = (ymin <= block.ymin and ymax >= block.ymax)

            #         if (dx_x or sx_x or cx_x) and (dx_y or sx_y or cx_y):
            #             if dx_x or cx_x:
            #                 block_approx_pos += np.array([-0.03, 0, 0])
            #             elif sx_x:
            #                 block_approx_pos += np.array([0.03, 0, 0])

    def multipleBlocks(self, res):
        print(res)

        for i in range(res.n_res):
            b = Block('', res.class_[i], Point(x=res.xcentre[i], y=res.ycentre[i]), np.array([res.roll[i],res.pitch[i],res.yaw[i]]))
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
            height = height + class2dimensions[self.block.class_][2] - 0.0195
            #input('Press enter to continue ....')
        
        print('***** TASK COMPLETED *****')

    def castle(self, vision_results, json_file):
        castle_pos_max = (1., 0.7) #angolo in basso a sx del quadrato
        x_max, y_max = castle_pos_max

        n = json_file["size"]

        #posiziona ogni blocco in base alle direttive del json
        for iter in range(1, n+1):
            block_class = list(class2name.keys())[list(class2name.values()).index(json_file[str(iter)]["class"])]
            
            for i in range(0, vision_results.n_res):
                if vision_results.class_[i] == block_class and not vision_results.processed[i]:
                    index = i
            vision_results.processed[index] = True
            
            self.block = Block('block'+str(iter), vision_results.class_[index], Point(x=vision_results.xcentre[index], y=vision_results.ycentre[index]), np.array([vision_results.roll[index],vision_results.pitch[index],vision_results.yaw[index]]))
            
            yaw = get_yaw(json_file[str(iter)]["r"])
            x_des = json_file[str(iter)]["x"]/100000
            y_des = json_file[str(iter)]["y"]/100000
            z_des = json_file[str(iter)]["z"]/100000
            
            if not -0.01 < self.block.rpy[1] < 0.01:
                print('mettendo il blocco in piedi...')
                landing_pos = np.array([0.5, 0.7])
                landing_rpy = np.array([pi/2, 0, pi/2])
                self.ruotaBloccoInPosizioneNormale(landing_pos, landing_rpy)
            
            if not -0.01 < self.block.rpy[0] < 0.01:
                print('mettendo il blocco in posizione normale...')
                landing_pos = np.array([0.5, 0.7])
                landing_rpy = np.array([0, 0, pi/2])
                self.ruotaBloccoInPosizioneNormale(landing_pos, landing_rpy) 
                
            self.pickAndPlaceBlock(np.array([x_max-x_des, y_max-y_des, 0]) , final_rpy = np.array([0, 0, yaw]), z_offset=z_des, attach_to_table=True)

class Res:
    def __init__(self):
        self.n = 0
        self.class_ = []
        self.x_centre = []
        self.y_centre = []
        self.z_centre = []
        self.roll = []
        self.pitch = []
        self.yaw = []
    
    def find(self, class_):
        for i in range(0, self.n):
            if (self.class_[i]==class_):
                return i
        print("block not found")
        return -1
    
    def remove(self, index):
        self.n = self.n-1
        self.class_.pop(index)
        self.x_centre.pop(index)
        self.y_centre.pop(index)
        self.z_centre.pop(index)
        self.roll.pop(index)
        self.pitch.pop(index)
        self.yaw.pop(index)

    def __str__(self):
        out = '--- blocks info ----\n'
        for i in range(self.n):
            out += ' - [' + str(self.x_centre[i]) + ', ' + str(self.y_centre[i]) + ', ' + str(self.z_centre[i]) + '], rpy: [' + str(self.roll[i]) + str(self.pitch[i]) + str(self.yaw[i]) + '] class: ' + str(self.class_[i]) + '\n'
        return out
    
    def __repr__(self):
        return str(self)


def talker(p):
    ros.init_node('custom_publisher_node', anonymous=True)
    loop_frequency = 1000.
    loop_rate = ros.Rate(loop_frequency)

    p.jstate = p.q

    # p.moveTo(np.array([0.5, 0.75, -0.7]), eul2rotm([pi, 0, pi/2]), 40, curve_type='line')
    # input('...')

    json_fd = open(os.path.join(os.path.expanduser("~"),"ros_ws","src","castle_build_path","output.json"))
    json_file = json_fd.read()
    json_fd.close()
    json_file = json.loads(json_file)

    # print('--- spawning blocks ---')
    # spawnBlocks(json_file=json_file)
    
    # input('...')

    print("vision server called")
    vision_results = p.vision_service.call()
    print(vision_results)
    input('..')
    
    p.moveTo(np.array([0.5, 0.75, -0.7]), eul2rotm([pi, 0, pi/2]), 40, curve_type='line')
    
    p.multipleBlocks(vision_results)

    #p.castle(vision_results, json_file)
    
    ros.spin()
    loop_rate.sleep()


if __name__ == '__main__':
    mypub = JointStatePublisher()
    try:
        talker(mypub)
    except ros.ROSInterruptException:
        pass
