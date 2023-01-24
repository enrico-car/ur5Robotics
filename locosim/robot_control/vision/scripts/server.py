#!/usr/bin/env python

from __future__ import print_function

from vision.srv import vision, visionResponse
import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import ast
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from roslib import message
import open3d as o3d
import math
import time
import glob

path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5")
path_template = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                            "template")


class Listener:

    def __init__(self):
        self.cl = []
        self.x_tavolo = []
        self.y_tavolo = []
        self.x_tavolo_def = []
        self.angle = []
        self.pointCloud = 0

    def call(self, cxmin, cxmax, cymin, cymax):
        print("-----------------------------getting distances----------------------------")
        self.pointCloud=rospy.wait_for_message("/ur5/zed2/point_cloud/cloud_registered", PointCloud2, timeout=None)
        data_out = pc2.read_points(self.pointCloud, field_names=("x", "y", "z"), skip_nans=False, uvs=[[int((cxmax+cxmin)/2), int(((cymax+cymin)/2))]])
        int_data = next(data_out)
        a, b, c = int_data

        # VERSIONE CON ROTAZIONI
        rot_z = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        rot_x = np.matrix([[1, 0, 0], [0, -0.5, 0.866], [0, -0.866, -0.5]])
        res = np.dot(np.matmul(rot_z, rot_x), np.array([a, b, c])) + np.array([-0.4, 0.475, 1.45])

        self.x_tavolo.append(round(res[0, 0], 4))
        self.y_tavolo.append(round(res[0, 1], 4))
        print(self.x_tavolo, self.y_tavolo)

        print("-----------------------------template matching----------------------------")
        img = cv2.imread(
            os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5", "cv_img.jpg"), 0)

        img = img[int(cymin):int(cymax), int(cxmin):int(cxmax)] #crop come box di yolo
        method = cv2.TM_CCORR_NORMED

        #scelgo la cartella relativa alla dimensione
        # if self.x_tavolo[len(self.x_tavolo) - 1] <= 0.34:
        #     template = os.path.join(path_template, "l", "*jpg")
        # elif self.x_tavolo[len(self.x_tavolo) - 1] <= 0.68:
        #     template = os.path.join(path_template, "m", "*jpg")
        # else:
        #     template = os.path.join(path_template, "h", "*jpg")
            
        template = os.path.join(path_template, "*jpg")   
        val_name = {}
        max_corr_index = ""
        max_val_rel = -1
        #almeno_uno=False
        for t in glob.glob(template):  # provo tutti i template a quella distanza
            temp = cv2.imread(t, 0)
            h, w = temp.shape
            if (h/w-0.2 <= ((cymax-cymin)/(cxmax-cxmin))<=h/w+0.2):
                #resize immagine
                img2 = img.copy()
                if(h>cymax-cymin):
                    temp=cv2.resize(temp, (int(cxmax-cxmin), int(cymax-cymin)-2))
                else:
                    img2=cv2.resize(img2, (w+2,h+2))
                
                # cv2.imshow("img",img2)
                # cv2.imshow("res",temp)
                # cv2.waitKey(0)
                # cv2.destroyAllWindows()

                result = cv2.matchTemplate(img2, temp, method) 
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                location = max_loc
                val_name[t] = max_val
                if max_val_rel < val_name[t]:
                    max_corr_index = t
                    max_val_rel = val_name[t]
                #bottom_right = (location[0] + w, location[1] + h)


        # print("index:",max_corr_index)
        # print("val_rel:",max_val_rel)
        info = max_corr_index.split("/")
        print(info)
        info = info[10]
        info = info.split("_")
        self.cl.append(int(info[0]))
        self.angle.append(float(info[3]))

        print("---------------------correzione posizione-----------------------------")
        if self.x_tavolo[len(self.x_tavolo) - 1] < 0.06:
            self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1])
        else:
            # forma quadrata
            if int(info[0]) == 0 or int(info[0]) == 9 or int(info[0]) == 10:
                if (int(info[0]) == 0):
                    lc = 0.01  # lato corto /2
                else:
                    lc = 0.02
                ang = float(info[3]) % 1.5708
                if ang < 0.7854:
                    self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1] + (lc / math.cos(ang)))
                else:
                    self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1] + (lc / math.cos(1.5708 - ang)))
            # forma rettangolare
            else:
                lc = 0.01
                ang = float(info[3]) % 3.1416
                if (int(info[0]) == 1 or int(info[0]) == 2 or int(info[0]) == 3 or int(info[0]) == 4):
                    ll = 0.02  # lato lungo /2
                elif (int(info[0]) == 5 or int(info[0]) == 6):
                    ll = 0.03
                else:
                    ll = 0.04
                if (ang < 1.5708):
                    self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1] + (lc / math.cos(ang)))
                elif (ang > 1.5708):
                    self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1] + (lc * math.cos(3.1416 - ang)))
                else:
                    self.x_tavolo_def.append(self.x_tavolo[len(self.x_tavolo) - 1] + ll)



def presenceControl(arr, xmin, ymin):
    for xmin1, ymin1, xmax1, ymax1, prob1, cl1 in arr:
        if ((xmin < xmin1 + 3 and xmin > xmin1 - 3) and (ymin < ymin1 + 3 and ymin > ymin1 - 3)):
            return True
    return False


def dataProcessing():
    det_res = []
    with open(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5", "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    print(det_res)
    blocks_info = []
    to_ret = []
    if len(det_res):
        xmin, ymin, xmax, ymax, p,c=det_res[0]
        blocks_info.append(det_res[0])
        xc_ = ((xmax + xmin) / 2)
        yc_ = ((ymin+ymax) / 2)
        l = Listener()
        l.call(xmin,xmax,ymin,ymax)
        to_ret.append([c, xc_, yc_])
        for xmin, ymin, xmax, ymax, prob, cl in det_res:
            if (not presenceControl(blocks_info, xmin, ymin)):
                blocks_info.append([xmin, ymin, xmax, ymax, prob, cl])
                xc_ = ((xmax + xmin) / 2)
                yc_ = ((ymax + ymin) / 2)
                l.call(xmin, xmax, ymin, ymax)
                to_ret.append([int(cl), xc_, yc_])

        print("results in data: ", l.cl, l.x_tavolo_def, l.y_tavolo)
        return visionResponse(len(l.cl), l.cl, l.x_tavolo_def, l.y_tavolo, l.angle)


def detection():
    print("------------------------running yolo----------------------------")
    os.system(
        "python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + path_yolo + "/cv_img.jpg --data " + path_yolo + "/data.yaml")
    print("post detection")


def retImage():
    print("------------------------taking image----------------------------")
    image = rospy.wait_for_message("/ur5/zed2/left/image_rect_color", Image, timeout=None)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "mono8")
    if not cv2.imwrite(
            os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5", "cv_img.jpg"), cv_image):
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
