#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim"))
from robot_control.vision.scripts.yolov5 import detect
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
import ast
import cv2
from cv_bridge import CvBridge
from gazebo_ros_link_attacher.srv import *
from sensor_msgs.msg import Image
np.set_printoptions(precision=5, suppress=True)

path_yolo5_images = os.path.join(os.path.expanduser("~"), "yolo5_images")
path_template = os.path.join(os.path.expanduser("~"), "template")
path_yolo5 = os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim","robot_control","vision","scripts","yolov5")

bridge = CvBridge()

block_class = 2

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
    if (-(pi/2)-0.01<=r<=-(pi/2)+0.01) :
        name = str(cl)+"_"+str(posx)+"_"+str(posy)+"_"+str(abs(np.round(r,5)))+"_"+str(np.round(p,5))+"_"+str(np.round(y-pi,5))+".jpg"
    else:
        name = str(cl)+"_"+str(posx)+"_"+str(posy)+"_"+str(abs(np.round(r,5)))+"_"+str(np.round(p,5))+"_"+str(np.round(y,5))+".jpg"
    print("name: " + name + "\n")

    return name


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
            initial_pose=Pose(position=Point(0.35, 0.5, altezza_tavolo + class2dimensions[block_class][1]/2), orientation=orientation),
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
            roll= -pi/2
            yaw=(int((iter%288)/9)*(pi/8)+pi)

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
            pitch=0
            roll= -pi/2
            yaw=(int((iter-72)/9)*(pi/8)+pi)

    #BLOCCO CHAMFER/FILLET
    else:
        if(iter<144): #in piedi
            roll= 0
            pitch= 0
            yaw=int(iter/9)*(pi/8)%(2*pi)
        elif(iter<288): #pi, storto
            roll= 0
            pitch=pi
            yaw=int((iter%144)/9)*(pi/8)
        elif(iter<432):   #sul lato lungo
            pitch=pi/2
            roll= 0
            yaw=int((iter%288)/9)*(pi/8)
        else:       #sul lato corto
            pitch=0
            roll= -(pi/2)
            yaw=(int((iter%432)/9)*(pi/8)+pi)
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
        
        ms.pose.position.z = 0.930

        print(set_state.call(ms))

    print('blocks moved')

    if block_class in [3, 4, 6, 10]:
        time.sleep(3.5)
    else:
        time.sleep(1.)


def removeBackground(img, height, width):
    for i in range(0, height):
        for j in range(0, width):
            if img[i][j] > 210:
                img[i][j] = 255


def checkTemplate(img_original, xmin, ymin, xmax, ymax):
    # print('controllo template')
    h, w = img_original.shape

    img = img_original[ymin:ymax, xmin:xmax]
    height, width = img.shape

    removeBackground(img, height, width)
    
    if xmin <= 0 or xmax >= w-1 or ymin <= 0 or ymax >= h-1:
        return True, xmin, ymin, xmax, ymax, img

    # print(xmin, xmax, ymin, ymax)
    # print(height, width)

    # cv2.imshow('.', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    n = 3

    # controllo che i bordi siano tutti bianchi (max 3 pixel di fila)
    soglia1 = n
    soglia2 = n
    #p rint('sinistra - destra')
    for i in range(height):
        # prima colonna a sx
        if img[i, 0] < 210:
            soglia1 -= 1
        else:
            soglia1 = n

        # ultima colonna a dx
        if img[i, width-1] < 210:
            soglia2 -= 1
        else:
            soglia2 = n
        
        #print(img[i, 0], '-', img[i, width-1])
        
        # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
        if soglia1 == 0:
            xmin -= 1
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax, None

        if soglia2 == 0:
            xmax += 1
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax, None
                      
    #print('*** sinistra - destra OK ***')

    soglia1 = n
    soglia2 = n
    # print('sopra - sotto')
    for j in range(width):
        # prima riga in alto
        if img[0, j] < 210:
            soglia1 -= 1
        else:
            soglia1 = n

        #ultima riga in basso
        if img[height-1, j] < 210:
            soglia2 -= 1
        else:
            soglia2 = n
        
        # print(img[0, j], '-', img[height-1, j])

        # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
        if soglia1 == 0:
            ymin -= 1
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax, None

        if soglia2 == 0:
            ymax += 1
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax, None
        
        # se il controllo riesce ad arrivare a questo punto (ha fatto entrambi i cicli for senza uscire e ricominciare) allora il template è ok
    #print('*** sopra - sotto OK ***')

    return True, xmin, ymin, xmax, ymax, img


def take_photo(name, dir_name):
    print('--- taking photo ---')
    img = rospy.wait_for_message('/ur5/zed_node/left/image_rect_color', Image)
    time.sleep(0.5)

    cv_image = bridge.imgmsg_to_cv2(img, "mono8")

    cv_image_cropped = cv_image[400:1000, 560:1350]
    height, width = cv_image_cropped.shape

    # cv2.imshow('.', cv_image_cropped)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    cv2.imwrite(os.path.join(path_yolo5_images, dir_name, name), cv_image_cropped)

    print('--- running yolo ---')
    detect.run(source=os.path.join(path_yolo5_images, dir_name, name), imgsz=(width, height))
    
    print("post detection")
    with open(os.path.join(path_yolo5, "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    
    if len(det_res):
        xmin, ymin, xmax, ymax, p, c = det_res[0]

        c_x, c_y = int((xmin+xmax)/2), int((ymin+ymax)/2)
        xmin_0, xmax_0 = c_x-5, c_x+5
        ymin_0, ymax_0 = c_y-5, c_y+5

        print('--- correcting template ---')
        ok = False
        while not ok:
            ok, xmin_0, ymin_0, xmax_0, ymax_0, img = checkTemplate(cv_image_cropped, xmin_0, ymin_0, xmax_0, ymax_0)
        
        # cv2.imshow('.', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        cv2.imwrite(os.path.join(path_template, dir_name, name), img)


def talker():
    rospy.init_node('block_spawner_node', anonymous=True)

    rate = rospy.Rate(500)

    spawnBlocks()
    input('..')

    x_in = [0.1667, 0.5, 0.8333]
    y_in = [0.2584, 0.475, 0.6917]

    iter = 0
    end = class2niter[block_class]
    print('numero di iterazioni: ', end)

    while not rospy.is_shutdown():
        blocks = getBlocksInfo()

        i = int(iter/3)%3
        j = iter%3

        xc = np.round(x_in[i], 5)
        yc = np.round(y_in[j], 5)
        x, y, z, w, roll, pitch, yaw = createQuaternion(iter)

        name = namedef(block_class, xc, yc, roll, pitch, yaw)
        print('template: ', name)

        posx, posy = name.split('_')[1:3]
        dir_name = posx+'_'+posy

        path = os.path.join(path_template, dir_name, name)

        # controllo se esiste il template, se non esiste, lo creo
        if not os.path.isfile(path):
            print('--- creazione template ---')
            moveBlock(blocks, xc, yc, x, y, z, w)

            take_photo(name, dir_name)
            
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