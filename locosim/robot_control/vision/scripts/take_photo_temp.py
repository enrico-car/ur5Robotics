#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import os
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
import sys
import ast
import time

path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5")
path_template = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                            "template")
nb="10"

def detection(ar):
    arg=ar
    print(arg)
    print("------------------------running yolo----------------------------")
    os.system(
        "python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + os.path.join(os.path.expanduser("~"), "yolo5_images", nb, arg)+ " --data " + path_yolo + "/data.yaml")
    print("post detection") #DA CAMBIARE
    with open(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5", "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    
    if len(det_res):
        xmin, ymin, xmax, ymax, p,c=det_res[0]
        img=cv2.imread(os.path.join(os.path.expanduser("~"), "yolo5_images",nb, arg)) #DA CAMBIARE
        img=img[ymin:ymax, xmin:xmax]
        cv2.imwrite(os.path.join(os.path.expanduser("~"), "template",nb, arg), img) #DA CAMBIARE


def retImageCallback(img, arg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "mono8")
    now = datetime.now()
    t = arg
    print(t)
    cv_image=cv_image[400:1000, 560:1350]
    h, w= cv_image.shape
    # for i in range(0, h):
    #     for j in range (0,w):
    #         r= cv_image[i][j]
    #         if (r==153):
    #             cv_image[i][j]=255
    cv2.imwrite(os.path.join(os.path.expanduser("~"), "yolo5_images",nb, t), cv_image) #DA CAMBIARE
    #sub.unregister()


def talker(arg):
    rospy.init_node('take_photo_node', anonymous=False)
    # print (arg) 
    # print (type(arg))
    img = rospy.wait_for_message('/ur5/zed2/left/image_rect_color', Image)
    retImageCallback(img, arg)
    # global sub
    # sub = rospy.Subscriber("/ur5/zed2/left/image_rect_color", Image, retImageCallback, callback_args=(arg),  queue_size=2)

    #rospy.spin()


if __name__ == '__main__':
    try:
        for arg in range (1,len(sys.argv)):
            print (sys.argv[arg]) 
            print (type(sys.argv[arg]))
            talker(sys.argv[arg])
            detection(sys.argv[arg])

    except rospy.ROSInterruptException:
        pass

