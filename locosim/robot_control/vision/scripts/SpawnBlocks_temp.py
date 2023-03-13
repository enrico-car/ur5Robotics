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
import cv2
from gazebo_ros_link_attacher.srv import *
np.set_printoptions(precision=5, suppress=True)

block_class=0

global name
name = ""

altezza_tavolo = 1.8-0.935
block_names = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET',
               'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
blocks_info = {block_names[0]: (0.03, 0.03), block_names[1]: (0.03, 0.03), block_names[2]: (3, 6), block_names[3]: (0.03, 0.06),
               block_names[4]: (0.03, 0.06), block_names[5]: (0.03, 0.09), block_names[6]: (3, 9), block_names[7]: (0.03, 0.12),
               block_names[8]: (0.03, 0.12), block_names[9]: (0.06, 0.06), block_names[10]: (0.06, 0.06)}
class2dimensions = {0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                    4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                    8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057]}
blocks_to_spawn = {'brick1_'+block_names[block_class]: block_names[block_class]}

class2niter = {0: 216, 1: 432, 2: 432, 3: 576, 4: 576, 5: 432, 6: 576, 7: 432, 8: 432, 9: 216, 10: 576}

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
    name = ""
    name = str(cl)+"_"+str(posx)+"_"+str(posy)+"_"+str(np.round(r, 5))+"_"+str(np.round(p, 5))+"_"+str(np.round(y, 5))+".jpg"
    print("name: " + name + "\n")


def spawnBlocks():
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for block in blocks_to_spawn:
        model_name = block
        angles = [0, pi/2, pi]
        w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
        x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
        y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
        z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)
        orientation = Quaternion(x, y, z, w)
        
        spawn_model_client(
            model_name=model_name,
            model_xml=open('/home/carro/ros_ws/src/locosim/ros_impedance_controller/worlds/models/'+str(blocks_to_spawn[model_name])+'/model.sdf', 'r').read(),
            robot_namespace='',
            initial_pose=Pose(position=Point(0.8, 0.5, altezza_tavolo + class2dimensions[block_class][1]/2), orientation=orientation),
            reference_frame='world'
        )


def getBlocksInfo():
    # gazebo service: /gazebo/get_world_properties "{}"
    rospy.wait_for_service('/gazebo/get_world_properties')
    gwp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    res = gwp.call()

    present_blocks = []
    for model in res.model_names:
        if model in blocks_to_spawn:
            print(model)
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

    #BLOCCHI RETTANGOLARI
    if block_class in [1, 2, 5, 7, 8]:
        if(iter<72):
            pitch= 0
            yaw=int(iter/9)*(pi/8)
            roll= 0
        elif(iter<144):
            pitch=pi
            yaw=int((iter%72)/9)*(pi/8)
            roll= 0
        elif(iter<288):
            pitch=pi/2
            roll= 0
            yaw=int((iter%144)/9)*(pi/8)
        else:
            pitch=0
            roll= pi/2
            yaw=int((iter%288)/9)*(pi/8)

    #BLOCCHI QUADRATI
    elif block_class in [0, 9]:
        if(iter<36):
            pitch= 0
            yaw=int(iter/9)*(pi/8)
            roll= 0
        elif(iter<72):
            pitch=pi
            yaw=int((iter%36)/9)*(pi/8)
            roll= 0
        else:
            pitch=pi/2
            roll= 0
            yaw=int((iter-72)/9)*(pi/8)+pi

    #BLOCCO CHAMFER/FILLET
    else:
        if(iter<144): #in piedi
            roll= 0
            pitch= 0
            yaw=int(iter/9)*(pi/8)%(2*pi)
        elif(iter<288): #pi, storto
            roll= 0
            pitch=pi
            yaw=int((iter%144)/9)*(pi/8)%(2*pi)
        elif(iter<432):   #sul lato lungo
            pitch=pi/2
            roll= 0
            yaw=int((iter%288)/9)*(pi/8)%(2*pi)
        else:       #sul lato corto
            pitch=0
            roll= 3*(pi/2)
            yaw=int((iter%432)/9)*(pi/8)%(2*pi)

    angles = [roll, pitch, yaw]

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)

    return x, y, z, w, roll, pitch, yaw


def moveBlock(blocks, xc, yc, x, y, z, w):
    print('moving blocks')

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    for block in blocks:
        ms = ModelState()
        ms.model_name = block.name

        ms.pose.orientation.x = x
        ms.pose.orientation.y = y
        ms.pose.orientation.z = z
        ms.pose.orientation.w = w
        
        ms.pose.position.x = xc
        ms.pose.position.y = yc
        
        ms.pose.position.z = 0.925

        print(set_state.call(ms))

    print('blocks moved')


def checkTemplate():
    global name
    block_class = name.split('_')[0]

    path = os.path.join(os.path.expanduser("~"), "template", block_class, name)

    # controllo se esiste il template, se si controllo che sia a posto
    if os.path.isfile(path):
        print('template trovato')
        # importo il template
        templ = cv2.imread(os.path.join(os.path.expanduser("~"), "template", block_class, name), cv2.IMREAD_GRAYSCALE)
        print(templ.shape)
        height, width  = templ.shape

        # controllo che i bordi siano tutti bianchi (controllo che max 5 pixel di fila non siano bianchi)
        soglia1 = 3
        soglia2 = 3
        print('sinistra - destra')
        for i in range(height):
            # prima colonna a sx
            if templ[i, 0] < 250:
                soglia1 -= 1
            else:
                soglia1 = 3

            # ultima colonna a dx
            if templ[i, width-1] < 250:
                soglia2 -= 1
            else:
                soglia2 = 3
            
            #print(templ[i, 0], '-', templ[i, width-1])
            
            if soglia1 == 0 or soglia2 == 0:
                print('** template incorretto, creazione di un nuovo template in corso... **')
                return False
        print('*** sinistra - destra OK ***')

        soglia1 = 3
        soglia2 = 3
        print('sopra - sotto')
        for j in range(width):
            # prima riga in alto
            if templ[0, j] < 250:
                soglia1 -= 1
            else:
                soglia1 = 3

            #ultima riga in basso
            if templ[height-1, j] < 250:
                soglia2 -= 1
            else:
                soglia2 = 3
            
            #print(templ[0, j], '-', templ[height-1, j])
            
            if soglia1 == 0 or soglia2 == 0:
                print('** template incorretto, creazione di un nuovo template in corso... **')
                return False
        print('*** sopra - sotto OK ***')

        return True
    
    # se il template non esiste, lo devo fare
    else:
        print('** template non ancora creato, creazione in corso... **')
        return False


def talker():
    rospy.init_node('block_spawner_node', anonymous=True)

    rate = rospy.Rate(500)

    spawnBlocks()
    input('..')

    x_in = [0.25,0.5, 0.75]
    y_in = [0.3125,0.475, 0.6375]

    iter = 0
    end = class2niter[block_class]
    print('numero di iterazioni: ', end)

    while not rospy.is_shutdown():
        blocks = getBlocksInfo()

        i = int(iter/3)%3
        j = iter%3

        xc = np.round(x_in[i], 5)
        yc = np.round(y_in[j], 5)
        x, y, z, w, roll, pitch, yaw= createQuaternion(iter)

        namedef(block_class, xc, yc, roll, pitch, yaw)
        print('template: ', name)

        if not checkTemplate():
            print('--- (ri)facimento template ---')
            # se il template non c'è o è fatto male -> da (ri)fare
            moveBlock(blocks, xc, yc, x, y, z, w)
            time.sleep(2.)
            #os.system("python3 vision/scripts/yolov8/yolov8_test.py " + name)
            os.system("python3 vision/scripts/take_photo_temp.py " + name)
        
        time.sleep(1.)
        print('template ', name,' -> OK')

        iter += 1
        print("iter: ", iter)

        if(iter==end):
            exit(0)
        
        #  input('fine iterazione, premere invio per continuare...')

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
