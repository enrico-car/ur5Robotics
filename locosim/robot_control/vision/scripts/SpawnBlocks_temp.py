#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion
import random
from math import pi as pi
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import sqrt as sqrt
from cmath import asin
import json
import os
from gazebo_ros_link_attacher.srv import *
np.set_printoptions(precision=5, suppress=True)

altezza_tavolo = 1.8-0.935
block_names = [ 'X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 
                'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET' ]
name2class = { block_names[0]: 0, block_names[1]: 1, block_names[2]: 2, block_names[3]: 3, block_names[4]: 4, block_names[5]: 5, 
               block_names[6]: 6, block_names[7]: 7, block_names[8]: 8, block_names[9]: 9, block_names[10]: 10 }
class2dimensions = { 0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                     4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                     8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057] }

class Block:
    def __init__(self, name, rpy, position=None):
        self.name = name
        self.rpy = rpy
        self.position = position
        self.classe = name2class[name]
        self.h, self.l, self.L = class2dimensions[self.classe]
        [l, L, h] = class2dimensions[self.classe]
        if pi/2-0.01 < self.rpy[0] < pi/2+0.01:
            self.radius = sqrt(l**2+h**2)/2
        elif pi/2-0.01 < self.rpy[1] < pi/2+0.01:
            self.radius = sqrt(L**2+h**2)/2
        else:
            self.radius = sqrt(l**2+L**2)/2

    def __str__(self):
        return '---- Block info ----- \n' + str(self.name) + '\nposition:\n' + str(self.position) + '\n' + str(self.span_x) \
               + ' - ' + str(self.span_y) + '\norientation:\n' + str(self.orientation) + '\nrotation: ' + str(self.rotation) + '\n---------------------'

    def __repr__(self):
        return str(self)

blocks_spawned = []

def quat2rpy(x, y, z, w):
    roll = np.round(atan2(2*(w*x+y*z), w**2 - x**2 - y**2 + z**2), 4)
    pitch = np.round(np.real(asin(2*(w*y-x*z))), 4)
    if pi/2-0.001 < pitch < pi/2+0.001 or -pi/2-0.001 < pitch < -pi/2+0.001:
        roll = 0
        yaw = np.round(-2*np.sign(pitch)*atan2(x, w), 4)
    else:
        yaw = np.round(atan2(2*(w*z+x*y), w**2 + x**2 - y**2 - z**2), 4)
    
    if yaw < 0:
        yaw = 2*pi+yaw
    
    return [roll, pitch, yaw]

def readBlocks():
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        
    for i in range(len(models.name)):
        if 'brick' in models.name[i]:
            name = models.name[i].split('_')[2]
            pos = Point(models.pose[i].position.x, models.pose[i].position.y, 0)
            rpy = quat2rpy(models.pose[i].orientation.x, models.pose[i].orientation.y, models.pose[i].orientation.z, models.pose[i].orientation.w)
            block = Block(name, rpy, pos)
            blocks_spawned.append(block)
            # print(name)
            # print(models.pose[i].orientation)
            # print(rpy)
            # print(block.radius)
            # input('')

def checkCollision(block):
    for b in blocks_spawned:
        dist = sqrt((block.position.x - b.position.x)**2 + (block.position.y - b.position.y)**2)
        if dist < (block.radius + b.radius + 0.06):
            return True
    return False

def getPosition(block, isCastle=False):
    x = random.uniform(0.04, 0.96)
    y = random.uniform(0.23, 0.76)

    # controllo se sono dentro un raggio di 15cm dalla posizione della base del robot (zona di singolarità di spalla)
    ok = False
    while not ok:
        block.position = Point(x, y, 0)

        dist = sqrt((x-0.5)**2 + (y-0.35)**2)
        
        if not isCastle and dist > 0.15 and not (x > 0.6 and y > 0.5) and not checkCollision(block):
            ok = True
        elif isCastle and dist > 0.15 and not checkCollision(block):
            ok = True
        else:
            x = random.uniform(0.07, 0.93)
            y = random.uniform(0.27, 0.73)

    return Point(x, y, altezza_tavolo + class2dimensions[block.classe][1]/2 + 0.03)

def spawnOneBlockBase(stl_name, i=1):
    readBlocks()
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    print('spawning ', stl_name)
    model_name = 'brick_' + str(i) + '_' + stl_name

    orientation, rpy = randomQuaternionOnYaw()
    block = Block(stl_name, rpy)
    position = getPosition(block)

    spawn_model_client(
        model_name = model_name,
        model_xml = open(os.path.join(os.path.expanduser("~"),'ros_ws','src','locosim','ros_impedance_controller','worlds','models',stl_name,'model.sdf'), 'r').read(),
        robot_namespace = '',
        initial_pose = Pose(position=position, orientation=orientation),
        reference_frame = 'world'
        )

def spawnOneBlock(stl_name, i=1):
    readBlocks()
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    print('spawning ', stl_name)
    model_name = 'brick_' + str(i) + '_' + stl_name

    orientation, rpy = randomQuaternion()
    block = Block(stl_name, rpy)
    position = getPosition(block)

    spawn_model_client(
        model_name = model_name,
        model_xml = open(os.path.join(os.path.expanduser("~"),'ros_ws','src','locosim','ros_impedance_controller','worlds','models',stl_name,'model.sdf'), 'r').read(),
        robot_namespace = '',
        initial_pose = Pose(position=position, orientation=orientation),
        reference_frame = 'world'
        )

def spawnBlocksForCastle(json_file):
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    n = json_file["size"]

    for i in range(1, n+1):
        readBlocks()
        stl_name = json_file[str(i)]["class"]
        print('spawning ', stl_name)
        model_name = 'brick_' + str(i) + '_' + stl_name

        orientation, rpy = randomQuaternion()
        block = Block(stl_name, rpy)
        position = getPosition(block)

        spawn_model_client(
            model_name = model_name,
            model_xml = open(os.path.join(os.path.expanduser("~"),'ros_ws','src','locosim','ros_impedance_controller','worlds','models',stl_name,'model.sdf'), 'r').read(),
            robot_namespace = '',
            initial_pose = Pose(position=position, orientation=orientation),
            reference_frame = 'world'
            )

def randomQuaternion():
    roll = random.choice([0., -pi/2])
    if roll == 0.:
        pitch = random.choice([0., pi/2, pi])
    else:
        pitch = 0.
    yaw = random.uniform(0.0, 2*pi)
    
    angles = [roll, pitch, yaw]
    print("quaternion", roll,pitch,yaw)

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)
    orientation = Quaternion(x, y, z, w)

    return orientation, angles

def randomQuaternionOnYaw():
    roll = 0.
    pitch = 0.
    yaw = random.uniform(0.0, 2*pi)
    
    angles = [roll, pitch, yaw]
    print("quaternion",roll,pitch,yaw)

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)
    orientation = Quaternion(x, y, z, w)

    return orientation, angles

# helper node that spawns required blocks in random position and orientation on the table
def talker():
    rospy.init_node('block_spawner_node', anonymous=True)

    json_fd = open(os.path.join(os.path.expanduser("~"),"ros_ws","src","castle_build_path","output.json"))
    json_file = json_fd.read()
    json_fd.close()
    json_file = json.loads(json_file)

    spawnBlocksForCastle(json_file=json_file)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
