#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from vision.srv import *

def serverCall():
    rospy.wait_for_service('vision_service')
    try:
        results = rospy.ServiceProxy('vision_service', vision)
        res=results()
        print("server called")
        print(res)
        #print(res.xcentre[0])
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print("Asking for info")
    serverCall()