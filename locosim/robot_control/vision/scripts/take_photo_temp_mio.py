#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim"))
from robot_control.vision.scripts.yolov5 import detect
import rospy
from cv_bridge import CvBridge
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
import ast
import time

path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control",
                         "vision", "scripts", "yolov5")
path_template = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", 
                             "vision", "scripts", "template")
path_yolo5_images = os.path.join(os.path.expanduser("~"), "yolo5_images")
block_class = ''


def checkTemplate(img_original, name, xmin, ymin, xmax, ymax):
    print('controllo template')
    img = img_original[ymin:ymax, xmin:xmax].copy()
    height, width = img.shape

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

def detection(name):
    print("------------------------running yolo----------------------------")
    
    # os.system(
    #     "python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + 
    #     os.path.join(os.path.expanduser("~"), "yolo5_images", block_class, name)+ " --data " + path_yolo + "/data.yaml")

    detect.run(source=os.path.join(path_yolo5_images, block_class, name))
    
    print("post detection") #DA CAMBIARE
    with open(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov5", "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    
    if len(det_res):
        xmin, ymin, xmax, ymax, p, c = det_res[0]
        
        img_original = cv2.imread(os.path.join(os.path.expanduser("~"), "yolo5_images", block_class, name), cv2.IMREAD_GRAYSCALE) #DA CAMBIARE
        
        # quando si ritaglia, si controlla che il bordo sia tutto dello stesso colore cosi si è sicuri che il blocco non viene tagliato
        ok = False
        while not ok:
            ok, xmin, ymin, xmax, ymax, img = checkTemplate(img_original, name, xmin, ymin, xmax, ymax)
        
        # img = img_original[ymin:ymax, xmin:xmax].copy()
        cv2.imwrite(os.path.join(os.path.expanduser("~"), "template", block_class, name), img) #DA CAMBIARE

def retImageCallback(img, name):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img, "mono8")

    cv_image = cv_image[400:1000, 560:1350]
    
    cv2.imwrite(os.path.join(path_yolo5_images, block_class, name), cv_image) #DA CAMBIARE

def talker(name):
    #rospy.init_node('take_photo_node', anonymous=False)

    img = rospy.wait_for_message('/ur5/zed2/left/image_rect_color', Image)
    time.sleep(1.)

    retImageCallback(img, name)


def main(name):
    talker(name)
    detection(name)

if __name__ == '__main__':
    print('**** running script take_photo ****')

    try:
        # for arg in range (1,len(sys.argv)):
        #     name = sys.argv[arg]
        #     print (name)
        #     block_class = name.split('_')[0]  

        #     talker(name)
        #     detection(name)
        import sys
        main(sys.argv[1:])

    except rospy.ROSInterruptException:
        pass

