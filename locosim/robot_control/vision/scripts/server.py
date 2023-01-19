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


# def retImageDepth(data, args):
#     print(data.is_bigendian)
#     print(data.encoding)
#     xc_, yc_ =args
#     bridge = CvBridge()
#     depth_image = bridge.imgmsg_to_cv2(data, "32FC1")
#     depth_image=cv2.resize(depth_image, [640,640],interpolation = cv2.INTER_CUBIC)
#     rows, cols=depth_image.shape
#     depth_image=depth_image[145:rows-310, 120:cols-100]
#     #cv2.imshow("data read", depth_image)
#     #cv_image_array = np.array(depth_image, dtype = np.dtype('f8'))
#     cv_image_norm = cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
#     #depth_image=cv2.resize(cv_image_norm, [640,640],interpolation = cv2.INTER_CUBIC)
#     # cv2.imwrite("depth.jpg",depth_image)
#     # cv2.imshow("after", depth_image)
#     # cv2.waitKey(0)
#     #cv2.imshow("Image window", depth_image)
#     #cv2.waitKey(1)
#     depth_sub.unregister()
#     #depth_image.data
#     print(depth_image[int(xc_),int(yc_)])
#     print(depth_image[int(xc_),int(yc_)])

# x_tavolo=-1
# y_tavolo=-1
# cl=-1


class Listener:

    def __init__(self):
        self.cl = []
        self.x_tavolo = []
        self.y_tavolo = []
        self.x_tavolo_def = []
        self.angle = []
        self.depth_sub_cloud = -1

    def callback(self, data, args):
        xc_, yc_, cl = args

        data_out = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[int(xc_), int((yc_ + 2))]])
        int_data = next(data_out)
        a, b, c = int_data

        # VERSIONE CON ROTAZIONI
        rot_z = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        rot_x = np.matrix([[1, 0, 0], [0, -0.5, 0.866], [0, -0.866, -0.5]])
        res = np.dot(np.matmul(rot_z, rot_x), np.array([a, b, c])) + np.array([-0.4, 0.475, 1.45])

        self.x_tavolo.append(round(res[0, 0], 4))
        self.y_tavolo.append(round(res[0, 1], 4))
        print(self.x_tavolo, self.y_tavolo)
        self.depth_sub_cloud.unregister()

    def call(self, cx, cy, cla):
        self.depth_sub_cloud = rospy.Subscriber("/ur5/zed2/point_cloud/cloud_registered", PointCloud2, self.callback, callback_args=(cx, cy, cla), queue_size=1)

    def tempMatch(self, xc, yc):
        print("template matching")
        img = cv2.imread(
            os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                         "yolov5", "cv_img.jpg"), 0)
        gap = 0
        # resize basato sulla distanza dalla camera
        if self.x_tavolo[len(self.x_tavolo) - 1] <= 0.34:
            template = os.path.join(path_template, "l", "*jpg")  # prendo tutta la cartella
            gap = 10
            img = img[int(yc) - 52:int(yc) + 52, int(xc) - 52:int(xc) + 52]
        elif self.x_tavolo[len(self.x_tavolo) - 1] <= 0.68:
            template = os.path.join(path_template, "m", "*jpg")  # prendo tutta la cartella
            gap = 6
            img = img[int(yc) - 29:int(yc) + 29, int(xc) - 35:int(xc) + 35]
        else:
            template = os.path.join(path_template, "h", "*jpg")  # prendo tutta la cartella
            gap = 4
            img = img[int(yc) - 21:int(yc) + 21, int(xc) - 27:int(xc) + 27]

        method = cv2.TM_CCORR_NORMED

        val_name = {}
        max_corr_index = ""
        max_val_rel = -1
        for t in glob.glob(template):  # provo tutti i template a quella distanza
            temp = cv2.imread(t, 0)
            h, w = temp.shape
            # img3=img[ int(yc)-gap-int(h/2):int(yc)+gap+int(h/2), int(xc)-gap-int(w/2):int(xc)+gap+int(w/2)]

            # cv2.imshow("cropped", img3)
            # cv2.imshow("template", temp)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            img2 = img.copy()
            result = cv2.matchTemplate(img2, temp, method)  # dim is (W-w+1, H-h+1), upper is value of base img
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            location = max_loc
            #print("val: ", max_val)
            val_name[t] = max_val
            if max_val_rel < val_name[t]:
                max_corr_index = t
                max_val_rel = val_name[t]
            #print(location)
            bottom_right = (location[0] + w, location[1] + h)
            # cv2.rectangle(img2, location, bottom_right, 255, 5)
            # cv2.imshow("Match", img2)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

        #print(val_name)
        #print("best_correnspondance name: ", max_corr_index, " with val: ", val_name[max_corr_index])

        info = max_corr_index.split("/")
        info = info[11]
        info = info.split("_")
        #print((info[0]))
        #print((info[3]))
        self.cl.append(int(info[0]))
        self.angle.append(float(info[3]))

        # correzione di tavolo_x basata sull'angolo
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


# def retImageCloud(data, args):
#     global x_tavolo
#     global y_tavolo
#     global cl
#     # width, height, point_step, row_step, data, isnan = data.width, data.height, data.point_step, data.row_step, data.data, False
#     # print(width, height, point_step, row_step)
#     xc_, yc_, cl= args

#     # lista=[]
#     # for i in range(int(xc_-50), int(xc_+50)):
#     #     for j in range(int(yc_-20), int(yc_+20)):
#     #         lista.append([int(i),int(j)])
#     # data_out = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True, uvs=lista) #, uvs=[[int(xc_+5), int(yc_)]])
#     # #print(len(data_out))
#     #print(sum(1 for _ in data_out)) #306847
#     # pcd=o3d.geometry.PointCloud()
#     # pcd.points=o3d.utility.Vector3dVector(data_out)
#     # o3d.visualization.draw_geometries([pcd])
#     data_out = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False, uvs=[[int(xc_),int(yc_)]])
#     int_data = next(data_out)
#     #for p in list(data_out):
#     a,b,c=int_data
#     depth=math.sqrt(a**2+b**2+c**2)
#     y_tavolo=0.525-a 
#     x_tavolo=math.sqrt(depth**2-0.6**2)-0.5
#     #print("x ", x_tavolo, "  y ",y_tavolo, cl)
#     depth_sub_cloud.unregister()   

#     # print(m)
#     # depth_sub_cal cloud.unregister()


# def getDepthCloud(cx, cy, cla):
#     global depth_sub_cloud
#     # global x_tavolo
#     # global y_tavolo
#     # global cl
#     l=Listener()
#     #depth_sub_cloud=rospy.Subscriber("/ur5/zed2/point_cloud/cloud_registered", PointCloud2, retImageCloud ,callback_args=(cx,cy, cla), queue_size=1)
#     depth_sub_cloud=rospy.Subscriber("/ur5/zed2/point_cloud/cloud_registered", PointCloud2, l.callback ,l, callback_args=(cx,cy, cla), queue_size=1)
#     #print("risultati di depth ", cl, x_tavolo, y_tavolo)retImageCloud


# def getDepth(cx,cy):
#     global depth_sub
#     depth_sub=rospy.Subscriber("/ur5/zed2/depth/depth_registered", Image, retImageDepth,callback_args=(cx,cy), queue_size=1)


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
    # print(det_res)
    # print(det_res[0][0])
    print(det_res)
    print("data converted")
    blocks_info = []
    to_ret = []
    if len(det_res):
        blocks_info.append(det_res[0])
        # global xc_resp,yc_resp
        xc_ = ((det_res[0][2] + det_res[0][0]) / 2) * 2  # //-145
        yc_ = ((det_res[0][3] + det_res[0][1]) / 2) * 1.88  # //-120
        l = Listener()
        l.call(xc_, yc_, int(det_res[0][5]))
        time.sleep(3.)
        l.tempMatch(xc_, yc_)
        time.sleep(1.)
        to_ret.append([int(det_res[0][5]), xc_, yc_])
        for xmin, ymin, xmax, ymax, prob, cl in det_res:
            if (not presenceControl(blocks_info, xmin, ymin)):
                blocks_info.append([xmin, ymin, xmax, ymax, prob, cl])
                xc_ = ((xmax + xmin) / 2) * 2  # -145
                yc_ = ((ymax + ymin) / 2) * 1.88  # -120
                l.call(xc_, yc_, int(cl))
                time.sleep(3.)
                l.tempMatch(xc_, yc_)
                time.sleep(1.)
                to_ret.append([int(cl), xc_, yc_])

        print("results in data: ", l.cl, l.x_tavolo_def, l.y_tavolo)
        return visionResponse(len(l.cl), l.cl, l.x_tavolo_def, l.y_tavolo, l.angle)
        # print(to_ret)        
        # return visionResponse(to_ret[0][0], to_ret[0][1], to_ret[0][2])


def detection():
    os.system(
        "python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + path_yolo + "/cv_img.jpg --data " + path_yolo + "/data.yaml")
    print("post detection")


def retImageCallback(data):
    print("callback begins")
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "mono8")
    # cv_image=cv2.resize(cv_image, [640,640])
    # cv2.imshow("wind",cv_image)
    # cv2.waitKey(0)
    # rows, cols=cv_image.shape
    # cv_image=cv_image[145:rows-310, 120:cols-100]
    # cv2.imshow("Image window", cv_image)
    if not cv2.imwrite(
            os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                         "yolov5", "cv_img.jpg"), cv_image):
        raise Exception("Image not saved")
    print("image passed")
    sub.unregister()


def retImage():
    global sub
    print("ret image begins")
    sub = rospy.Subscriber("/ur5/zed2/left/image_rect_color", Image, retImageCallback, queue_size=1)


def imageProcessing(req):
    print("image processing begins")
    retImage()
    detection()
    return dataProcessing()

    # print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))


def visionServerFunc():
    rospy.init_node('vision_server', anonymous=True)
    service = rospy.Service('vision_service', vision, imageProcessing)
    print("Ready to take picture and send data")
    rospy.spin()


if __name__ == "__main__":
    visionServerFunc()
