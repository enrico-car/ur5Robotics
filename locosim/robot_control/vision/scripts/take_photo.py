#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import os
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
import sys
import time


def retImageCallback(img):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "mono8")
    now = datetime.now()
    t = str(now.strftime("%m%d%Y:%H%M%S") + ".jpg")
    print(t)
    cv2.imwrite(os.path.join(os.path.expanduser("~"), "yolo5_images", t), cv_image)

    sub.unregister()


def talker():
    rospy.init_node('take_photo_node', anonymous=False)

    #img = rospy.wait_for_message('/ur5/zed2/left/image_rect_color', Image)
    global sub
    sub = rospy.Subscriber("/ur5/zed2/left/image_rect_color", Image, retImageCallback, queue_size=2)

    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

