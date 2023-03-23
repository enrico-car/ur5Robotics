#!/usr/bin/env python

from __future__ import print_function
import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim"))
from robot_control.vision.scripts.yolov5 import detect
from vision.srv import vision, visionResponse
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import cv2
import ast
import numpy as np
from stl import mesh
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import open3d as o3d
from simpleicp import SimpleICP, PointCloud
import math
from math import pi as pi
from math import cos as cos
from math import sin as sin
import time
import glob
from pprint import pprint
import copy
from lab_exercises.lab_palopoli.kinematics import rotX, rotY, rotZ

altezza_tavolo_da_gound_plane_gazebo = 0.8675

path_stls = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "ros_impedance_controller", "worlds", "models")
path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "yolov5")
path_vision = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision")
path_template = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "template")
path_template_home = os.path.join(os.path.expanduser("~"), "template")

FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

class Listener:

    def __init__(self):
        self.cl = []
        self.x_tavolo = []
        self.y_tavolo = []
        self.x_tavolo_def = []
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.processed = []
        self.pointCloud = 0
    
    def changeBackground(self, img, height, width):
        print('changing background')
        for i in range(0, height):
            for j in range (0, width):
                # if i in range(0, int(height/10)) or i in range(int(9*height/10), height) \
                #         or j in range(0, int(width/7)) or j in range(int(6*width/7), width):
                pixel = img[i][j]
                if pixel in [148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 235]:
                    img[i][j] = 255
        
        return img

    def createImageForTemplate(self, img_original, x_pos, y_pos, xmin, ymin, xmax, ymax):
        print('controllo template')
        img = img_original[ymin:ymax, xmin:xmax].copy()
        height, width = img.shape

        if x_pos > 0.75 or y_pos > 0.65:
            img = self.changeBackground(img, height, width)
        
        # cv2.imshow('.', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        # controllo che i bordi siano tutti bianchi (max 3 pixel di fila)
        soglia1 = 3
        soglia2 = 3
        print('sinistra - destra')
        for i in range(height):
            # prima colonna a sx
            if img[i, 0] < 250:
                soglia1 -= 1
            else:
                soglia1 = 3

            # ultima colonna a dx
            if img[i, width-1] < 250:
                soglia2 -= 1
            else:
                soglia2 = 3
            
            #print(img[i, 0], '-', img[i, width-1])
            
            # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
            if soglia1 == 0:
                xmin -= 1
                print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax, None

            if soglia2 == 0:
                xmax += 1
                print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax, None
                        
        print('*** sinistra - destra OK **')

        soglia1 = 3
        soglia2 = 3
        print('sopra - sotto')
        for j in range(width):
            # prima riga in alto
            if img[0, j] < 250:
                soglia1 -= 1
            else:
                soglia1 = 3

            #ultima riga in basso
            if img[height-1, j] < 250:
                soglia2 -= 1
            else:
                soglia2 = 3
            
            # print(img[0, j], '-', img[height-1, j])

            # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
            if soglia1 == 0:
                ymin -= 1
                print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax, None

            if soglia2 == 0:
                ymax += 1
                print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax, None
            
            # se il controllo riesce ad arrivare a questo punto (ha fatto entrambi i cicli for senza uscire e ricominciare) allora il template è ok
        print('*** sopra - sotto OK **')

        return True, xmin, ymin, xmax, ymax, img

    def getTemplatePosition(self, xpos, ypos):
        if 0. < xpos < 0.333:
            template_x = 0.25
        elif 0.333 <= xpos <= 0.667:
            template_x = 0.5
        else:
            template_x = 0.75
        
        if 0. < ypos < 0.367:
            template_y = 0.3125
        elif 0.367 <= ypos <= 0.5834:
            template_y = 0.475
        else:
            template_y = 0.6375
        
        return template_x, template_y

    def call(self, xmin, xmax, ymin, ymax, classe=None):
        rospy.loginfo('--- reading pointcloud to get first positions ---')
        print("-----------------------------getting distances----------------------------")
        # calcolo il centro del bb di yolo e leggo la pointcloud in quel punto per ottenere una posizione approssimativa del blocco
        self.pointCloud = rospy.wait_for_message("/ur5/zed2/point_cloud/cloud_registered", PointCloud2, timeout=None)
        field_names = [field.name for field in self.pointCloud.fields]
        data_out = list(pc2.read_points(self.pointCloud, field_names=field_names, skip_nans=False, uvs=[[int((xmax+xmin)/2), int(((ymax+ymin)/2))]]))
        xyz = np.array(np.array(data_out).flat)

        rot_z = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        rot_x = np.matrix([[1, 0, 0], [0, -0.5, 0.866], [0, -0.866, -0.5]])
        res = np.dot(rot_z @ rot_x, np.array([xyz[0], xyz[1], xyz[2]])) + np.array([-0.4, 0.475, 1.45])

        x_pos = round(res[0, 0], 4)
        y_pos = round(res[0, 1], 4)

        # controllo se un blocco in una posizione vicina è già stato aggiunto, se si, non serve processare questo blocco
        pos_ok = True
        for x,y in zip(self.x_tavolo, self.y_tavolo):
            if x-0.03 < x_pos < x+0.03 and y-0.03 < y_pos < y+0.03:
                pos_ok = False
        
        # controllo sulla posizione data dalla pointcloud
        if res[0, 2] > 0.9:
            pos_ok = False
        
        if not pos_ok:
            print('terminazione per pericolo di processazione dello stesso blocco - detection multiple sullo stesso blocco')
            return

        rospy.loginfo('--- running template matching ---')
        print("-----------------------------template matching----------------------------")
        # dato il bb di yolo, controllo se va bene e in caso lo sistemo (aumentando le dimensioni) nel caso in cui il blocco venisse tagliato male aggiornando xmin, xmax, ymin, ymax
        img_original = cv2.imread(os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim","robot_control","vision","scripts","yolov5","cv_img.jpg"), 0)

        ok = False
        while not ok:
            ok, xmin, ymin, xmax, ymax, img = self.createImageForTemplate(img_original, x_pos, y_pos, xmin, ymin, xmax, ymax)

        method = cv2.TM_CCORR_NORMED

        # ottengo il quadrante di appartenenza del blocco
        temp_x, temp_y = self.getTemplatePosition(x_pos, y_pos)

        template_path = os.path.join(path_template, str(temp_x)+'_'+str(temp_y))
        print("path: ", template_path)

        val_name = {}
        max_corr_index = ""
        max_val_rel = -1
        # provo tutti i template in quel quadrante e tengo quello con matching migliore per ricavare classe e orientazione
        for t in os.listdir(template_path):
            if classe is not None:
                template_class = int(t.split('_')[0])
                if classe != template_class:
                    continue

            temp = cv2.imread(os.path.join(template_path, t), 0)
            
            h, w = temp.shape
            image_ratio = h/w
            if image_ratio*0.9 <= ((ymax-ymin)/(xmax-xmin)) <= image_ratio*1.1:
                #resize immagine
                img2 = img.copy()

                if(h > ymax-ymin):
                    temp = cv2.resize(temp, (int(xmax-xmin), int(ymax-ymin)-2))
                else:
                    img2 = cv2.resize(img2, (w+2,h+2))

                # cv2.namedWindow("res")
                # cv2.namedWindow("temp")
                # cv2.moveWindow("res", 40,30)
                # cv2.moveWindow("temp", 340,30)
                # cv2.imshow("res", img2)
                # cv2.imshow('temp', temp)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                result = cv2.matchTemplate(img2, temp, method) 
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                location = max_loc
                
                val_name[t] = max_val
                if max_val_rel < val_name[t]:
                    max_corr_index = t
                    max_val_rel = val_name[t]
                
        print("*****************************************")
        print("index:",max_corr_index)
        print("val_rel:", max_val_rel)

        info = max_corr_index.split("_")
        print(info)
        self.cl.append(int(info[0]))
        self.roll.append(np.round(float(info[3]),4))
        self.pitch.append(np.round(float(info[4]),4))
        info[5] = info[5][:len(info[5])-4] # per togliere il .jpg
        self.yaw.append(np.round(float(info[5]),4))

        rospy.loginfo('--- getting precise center of block ---')
        # calcolo più preciso del centro del blocco (attraverso la poincloud)
        uvs = []
        for i in range(int(xmin), int(xmax)):
            for j in range(int(ymin), int(ymax)):
                uvs.append([i, j])
        
        cloud_data = list(pc2.read_points(self.pointCloud, field_names=field_names, skip_nans=True, uvs=uvs))
        
        xyz = [np.array([x,y,z]) for x,y,z,_ in cloud_data ]
        
        xyz_worldframe = []
        for p in xyz:
            p_worldframe = np.array((np.dot(rot_z @ rot_x, p) + np.array([-0.4, 0.475, 1.45])).flat)
            if p_worldframe[2] > 0.816:
                xyz_worldframe.append(p_worldframe)
        
        open3d_cloud = o3d.geometry.PointCloud()
        if len(xyz_worldframe):
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz_worldframe))
            open3d_cloud.paint_uniform_color([0, 0, 1])
            aabb = open3d_cloud.get_axis_aligned_bounding_box()
            aabb.color = (1, 0, 0)
            c = aabb.get_center()
            print('centro bb: ', c)
            self.x_tavolo.append(np.round(c[0], 4))
            self.y_tavolo.append(np.round(c[1], 4))
            self.processed.append(False)
            # o3d.visualization.draw_geometries([open3d_cloud, aabb])
        else:
            print('errore nella lettura della pointcloud, nessun punto rilevato')

        # tot = np.array([0., 0., 0.])
        # for p in xyz_worldframe:
        #     tot += p
        # com = tot / len(xyz_worldframe)

        # stl_data = mesh.Mesh.from_file(os.path.join(path_stls, "X1-Y4-Z2", "X1-Y4-Z2.stl"))
        # points = stl_data.points.reshape([-1,3])
        # stl_points = np.unique(points, axis=0)

        # stl_cloud = o3d.geometry.PointCloud()
        # stl_cloud.points = o3d.utility.Vector3dVector(np.array(stl_points))
        # stl_cloud.paint_uniform_color([1, 0.706, 0])

        # diameter = np.linalg.norm(np.asarray(stl_cloud.get_max_bound()) - np.asarray(stl_cloud.get_min_bound()))
        # camera = [1., 0., diameter]
        # print('camera: ', camera)
        # radius = diameter * 1000

        # _, pt_map = stl_cloud.hidden_point_removal(camera, radius)

        # pcd = stl_cloud.select_by_index(pt_map)
        
        # pcd_moved = []
        # for p in np.asarray(pcd.points):
        #     if p[2] > 0.005:
        #         pcd_moved.append(p + np.array([c[0], c[1], 0.816]))
        
        # pcd.points = o3d.utility.Vector3dVector(np.array(pcd_moved))
        
        # downpcd = open3d_cloud.voxel_down_sample(voxel_size=0.004)
        # downpcd.paint_uniform_color([0, 1, 0])

        # # o3d.visualization.draw_geometries([pcd, downpcd])

        # trans_init = np.asarray([[1.,0.,0.,0.], [0.,1.,0.,0.], [0.,0.,1.,0.], [0.,0.,0.,1.]])

        # source = copy.deepcopy(pcd)
        # target = copy.deepcopy(downpcd)

        # source.paint_uniform_color([1, 0, 0])
        # target.paint_uniform_color([0, 1, 0])
        # source.transform(trans_init)
        # o3d.visualization.draw_geometries([source, target])

        # threshold = 0.02
        # evaluation = o3d.pipelines.registration.evaluate_registration(source, target, threshold, trans_init)
        # print(evaluation)

        # reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,o3d.pipelines.registration.TransformationEstimationPointToPoint())
        # print(reg_p2p)
        # print("Transformation is:")
        # print(reg_p2p.transformation)

        # source.transform(reg_p2p.transformation)
        # o3d.visualization.draw_geometries([source, target])

       
def presenceControl(arr, xmin, ymin):
    for xmin1, ymin1, xmax1, ymax1, prob1, cl1 in arr:
        if ((xmin < xmin1 + 3 and xmin > xmin1 - 3) and (ymin < ymin1 + 3 and ymin > ymin1 - 3)):
            return True
    return False


def dataProcessing():
    det_res = []

    with open(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "yolov5", "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    
    print(det_res)

    blocks_info = []
    to_ret = []
    if len(det_res):
        xmin, ymin, xmax, ymax, confidence, classe = det_res[0]
        blocks_info.append(det_res[0])

        xc_ = ((xmax + xmin) / 2)
        yc_ = ((ymin+ymax) / 2)

        l = Listener()

        if confidence > 0.94:
            l.call(xmin,xmax,ymin,ymax, int(classe))
        else:
            l.call(xmin,xmax,ymin,ymax)

        to_ret.append([classe, xc_, yc_])

        for xmin, ymin, xmax, ymax, confidence, classe in det_res:
            if (not presenceControl(blocks_info, xmin, ymin)):
                blocks_info.append([xmin, ymin, xmax, ymax, confidence, classe])

                xc_ = ((xmax + xmin) / 2)
                yc_ = ((ymax + ymin) / 2)

                if confidence > 0.94:
                    l.call(xmin,xmax,ymin,ymax, int(classe))
                else:
                    l.call(xmin,xmax,ymin,ymax)
                
                to_ret.append([int(classe), xc_, yc_])

        print("results in data: ", l.cl, l.x_tavolo, l.y_tavolo, l.roll, l.pitch, l.yaw)
        return visionResponse(len(l.cl), l.cl, l.x_tavolo, l.y_tavolo, l.roll, l.pitch, l.yaw, l.processed)


def detection():
    print("------------------------running yolo----------------------------")
    # os.system("python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + path_yolo + "/cv_img.jpg --data " + path_yolo + "/data.yaml")
    detect.run()
    print("post detection")


def retImage():
    print("------------------------taking image----------------------------")
    image = rospy.wait_for_message("/ur5/zed2/left/image_rect_color", Image, timeout=None)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "mono8")
    
    if not cv2.imwrite(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts","yolov5", "cv_img.jpg"), cv_image):
        raise Exception("Image not saved")
    print("image passed")


def imageProcessing(req):
    print("image processing begins")
    retImage()
    detection()
    return dataProcessing()


def visionServerFunc():
    rospy.init_node('vision_server', anonymous=True)
    service = rospy.Service('vision_service', vision, imageProcessing)
    print("Ready to take picture and send data")
    rospy.spin()


if __name__ == "__main__":
    visionServerFunc()
