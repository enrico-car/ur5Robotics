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
block_num=10
name=""


block_names = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET',
               'X1-Y4-Z2', 'X1-Y4-Z1', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
blocks_info = {block_names[0]: (0.03, 0.03), block_names[1]: (0.03, 0.03), block_names[2]: (3, 6), block_names[3]: (0.03, 0.06),
               block_names[4]: (0.03, 0.06), block_names[5]: (0.03, 0.09), block_names[6]: (3, 9), block_names[7]: (0.03, 0.12),
               block_names[8]: (0.03, 0.12), block_names[9]: (0.06, 0.06), block_names[10]: (0.06, 0.06)}
blocks_to_spawn = {'brick1': block_names[block_num]}
''', 'block2': block_names[block_num], 'block3': block_names[block_num], 'block4': block_names[block_num],
                   'block5': block_names[block_num], 'block6': block_names[block_num], 'block7': block_names[block_num], 'block8': block_names[block_num],
                   'block9': block_names[block_num]}#, block10': block_names[9], 'block11': block_names[10]}'''

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


def namedef(cl, posx, posy, r, p, y):
    global name
    name=""
    name=str(cl)+"_"+str(posx)+"_"+str(posy)+"_"+str(r)+"_"+str(p)+"_"+str(y)+".jpg"
    print("name:" +name+ "\n")


def spawnBlocks():
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for block in blocks_to_spawn:
        model_name = block
        spawn_model_client(
            model_name=model_name,
            model_xml=open('/home/annachiara/ros_ws/src/locosim/ros_impedance_controller/worlds/models/'+str(blocks_to_spawn[model_name])+'/model.sdf', 'r').read(),
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


def createQuaternion(iter):

    #BLOCCO 1
    # if(iter<72):
    #     pitch= 0
    #     yaw=int(iter/9)*(pi/8)
    #     roll= 0
    # elif(iter<144):
    #     pitch=pi
    #     yaw=int((iter%72)/9)*(pi/8)
    #     roll= 0
    # elif(iter<288):
    #     pitch=pi/2
    #     roll= 0
    #     yaw=int((iter%144)/9)*(pi/8)
    # else:
    #     pitch=0
    #     roll= pi/2
    #     yaw=int((iter%288)/9)*(pi/8)

    #BLOCCO 9
    # if(iter<36):
    #     pitch= 0
    #     yaw=int(iter/9)*(pi/8)
    #     roll= 0
    # elif(iter<72):
    #     pitch=pi
    #     yaw=int((iter%36)/9)*(pi/8)
    #     roll= 0
    # else:
    # pitch=pi/2
    # roll= 0
    # yaw=int((iter)/9)*(pi/8)+pi

    #BLOCCO CHAMFER/FILLET
    if(iter<144): #in piedi
        pitch= 0
        yaw=int(iter/9)*(pi/8)
        roll= 0
    elif(iter<288): #pi, storto
        pitch=pi
        yaw=int((iter%144)/9)*(pi/8)
        roll= 0
    elif(iter<432):   #sul lato lungo
        pitch=pi/2
        roll= 0
        yaw=int((iter%288)/9)*(pi/8)
    else:       #sul lato corto
        pitch=0
        roll= 3*(pi/2)
        yaw=int((iter%432)/9)*(pi/8)



    

    angles = [roll, pitch, yaw]

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)

    
    return x, y, z, w, roll, pitch, yaw


def moveBlock(blocks, iter):
    print('moving blocks')

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    x_in=[0.25,0.5, 0.75]
    y_in=[0.3125,0.475, 0.6375]
    i=int(iter/3)%3
    j=iter%3

    for block in blocks:
        ms = ModelState()
        ms.model_name = block.name

        x, y, z, w, roll, pitch, yaw= createQuaternion(iter)

        ms.pose.orientation.x = x
        ms.pose.orientation.y = y
        ms.pose.orientation.z = z
        ms.pose.orientation.w = w


        
        x = np.round(x_in[i], 5)
        y = np.round(y_in[j], 5)
        ms.pose.position.x = x
        ms.pose.position.y = y
        

        ms.pose.position.z = 0.925
        print(ms)
        namedef(block_num, x, y, roll, pitch, yaw)
        print(set_state.call(ms))

    print('blocks moved')


def talker():
    rospy.init_node('block_spawner_node', anonymous=True)

    rate = rospy.Rate(500)

    spawnBlocks()
    i=0
    while not rospy.is_shutdown():
        #input("Press Enter to continue...")
        blocks = getBlocksInfo()
        pprint(blocks)
        moveBlock(blocks, i)
        i=i+1
        print("i:"+str(i))
        #if(i==433):
        #if(i==217):
        if(i==577):
            exit(0)
        else:
            time.sleep(3.)
            os.system("python3 take_photo_temp.py "+ name)

        time.sleep(2.)


        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
