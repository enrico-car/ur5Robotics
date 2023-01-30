#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import Pose, Point, Quaternion
from pprint import pprint
import random
from math import pi as pi
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
import time
import os

np.set_printoptions(precision=5, suppress=True)

block_names = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET',
               'X1-Y4-Z2', 'X1-Y4-Z1', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
blocks_info = {block_names[0]: (0.03, 0.03), block_names[1]: (0.03, 0.03), block_names[2]: (3, 6), block_names[3]: (0.03, 0.06),
               block_names[4]: (0.03, 0.06), block_names[5]: (0.03, 0.09), block_names[6]: (3, 9), block_names[7]: (0.03, 0.12),
               block_names[8]: (0.03, 0.12), block_names[9]: (0.06, 0.06), block_names[10]: (0.06, 0.06)}
blocks_to_spawn = {'block1': block_names[0], 'block2': block_names[1], 'block3': block_names[2], 'block4': block_names[3],
                   'block5': block_names[4], 'block6': block_names[5], 'block7': block_names[6], 'block8': block_names[7],
                   'block9': block_names[8], 'block10': block_names[9], 'block11': block_names[10]}


class Block:
    def __init__(self, name, position, orientation):
        self.name = name
        self.position = position
        self.orientation = orientation
        self.rotation = np.round(atan2(orientation.z, orientation.w), 5) * 2
        block_type = blocks_to_spawn[name]
        l, L = blocks_info[block_type]
        span_x = np.round(L*sin(self.rotation) + l*cos(self.rotation), 5) * 1.5
        span_y = np.round(L*cos(self.rotation) + l*sin(self.rotation), 5) * 1.5
        self.span_x = [(position.x - span_x/2), (position.x + span_x/2)]
        self.span_y = [(position.y - span_y/2), (position.y + span_y/2)]


    def __str__(self):
        return '---- Block info ----- \n' + str(self.name) + '\nposition:\n' + str(self.position) + '\n' + str(self.span_x) \
               + ' - ' + str(self.span_y) + '\norientation:\n' + str(self.orientation) + '\nrotation: ' + str(self.rotation) + '\n---------------------'

    def __repr__(self):
        return str(self)


def spawnBlocks():
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for block in blocks_to_spawn:
        model_name = block
        spawn_model_client(
            model_name=model_name,
            model_xml=open('/home/carro/ros_ws/src/locosim/ros_impedance_controller/worlds/models/'+str(blocks_to_spawn[model_name])+'/model.sdf', 'r').read(),
            robot_namespace='',
            initial_pose=Pose(position=Point(0.5, 0.35, 0.925), orientation=Quaternion(0, 0, 0, 0)),
            reference_frame='world'
        )


def getBlocksInfo():
    # gazebo service: /gazebo/get_world_properties "{}"
    rospy.wait_for_service('/gazebo/get_world_properties')
    gwp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    res = gwp.call()

    present_blocks = []
    for model in res.model_names:
        print(model)
        if model in blocks_to_spawn:
            present_blocks.append(model)

    # gazebo service: /gazebo/get_model_properties nome_del_model
    rospy.wait_for_service('/gazebo/get_model_state')
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    blocks = []
    for block in present_blocks:
        model_state = gms.call(block, 'link')
        b = Block(block, model_state.pose.position, model_state.pose.orientation)
        blocks.append(b)

    return blocks


def createQuaternion():
    f = random.random()
    if f < 0.25:
        pitch = 0
    elif f < 0.5:
        pitch = pi/2
    elif f < 0.75:
        pitch = pi
    else:
        pitch = -pi/2

    yaw = random.uniform(0.0, 2*pi)

    angles = [0, pitch, yaw]

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)

    return x, y, z, w, yaw


def moveBlock(blocks):
    print('moving blocks')

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    for block in blocks:
        ms = ModelState()
        ms.model_name = block.name

        x, y, z, w, a = createQuaternion()

        ms.pose.orientation.x = x
        ms.pose.orientation.y = y
        ms.pose.orientation.z = z
        ms.pose.orientation.w = w

        pose_ok = False
        while not pose_ok:
            x = np.round(random.uniform(0.05, 0.95), 5)
            y = np.round(random.uniform(0.15, 0.78), 5)
            block_type = blocks_to_spawn[block.name]
            l, L = blocks_info[block_type]
            span_x = np.round(L * sin(2*a) + l * cos(2*a), 5) * 2
            span_y = np.round(L * cos(2*a) + l * sin(2*a), 5) * 2
            x_interval = [x - span_x/2, x + span_x/2]
            y_interval = [y - span_y/2, y + span_y/2]

            for b in blocks:
                if (b.span_x[0] <= x_interval[0] <= b.span_x[1] or b.span_x[0] <= x_interval[1] <= b.span_x[1]) and\
                        (b.span_y[0] <= y_interval[0] <= b.span_y[1] or b.span_y[0] <= y_interval[1] <= b.span_y[1]):
                    break
                else:
                    continue

            ms.pose.position.x = x
            ms.pose.position.y = y
            pose_ok = True

        ms.pose.position.z = 0.925

        set_state.call(ms)

    print('blocks moved')


def talker():
    rospy.init_node('block_spawner_node', anonymous=True)

    rate = rospy.Rate(500)

    spawnBlocks()

    while not rospy.is_shutdown():
        input("Press Enter to continue...")
        blocks = getBlocksInfo()
        pprint(blocks)
        moveBlock(blocks)

        time.sleep(5.)

        os.system('python3 take_photo.py &')

        time.sleep(2.)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
