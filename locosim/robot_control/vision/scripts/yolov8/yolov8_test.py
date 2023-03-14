from ultralytics import YOLO
import os
import sys
import rospy
import time
from cv_bridge import CvBridge
from datetime import datetime
import cv2
from sensor_msgs.msg import Image
import ast

path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                        "yolov8")
path_template = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts",
                            "template")
block_class=""

model = YOLO(os.path.join(path_yolo, 'best.pt'))


# for result in results:
#     print(result.boxes.xyxy)   # box with xyxy format, (N, 4)
#     #print(result.boxes.xywh)   # box with xywh format, (N, 4)
#     #print(result.boxes.xyxyn)  # box with xyxy format but normalized, (N, 4)
#     #print(result.boxes.xywhn)  # box with xywh format but normalized, (N, 4)
#     print(result.boxes.conf)   # confidence score, (N, 1)
#     print(result.boxes.cls)    # cls, (N, 1)

def detection(name, cv_image):
    print("------------------------running yolo----------------------------")
    
    results = model.predict(source=os.path.join(os.path.expanduser("~"), "yolo5_images", block_class, name), save=True)

    print("post detection") #DA CAMBIARE

    for result in results:
        print(result.boxes.xyxy)
        print(result.boxes.cls)
        print(result.boxes.conf)
    input('...')

    for result in results:
        xmin, ymin, (xmax, ymax) = 0, 0, cv_image.shape
        if len(result.boxes.xyxy.numpy()):
            xmin, ymin, xmax, ymax = result.boxes.xyxy.numpy()[0]
        img = cv_image[int(ymin):int(ymax), int(xmin):int(xmax)]

        cv2.imwrite(os.path.join(os.path.expanduser("~"), "template", block_class, name), img) #DA CAMBIARE

def talker(name):
    rospy.init_node('take_photo_node', anonymous=False)

    img_from_topic = rospy.wait_for_message('/ur5/zed2/left/image_rect_color', Image)
    time.sleep(1.)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_from_topic, "mono8")

    cv2.imshow('.', cv2.resize(cv_image, (640, 480)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cv_image = cv_image[400:1000, 560:1350]
    h, w = cv_image.shape
    # cambio colore background
    # for i in range(0, h):
    #     for j in range (0,w):
    #         r= cv_image[i][j]
    #         if (r==153):
    #             cv_image[i][j]=255
    cv2.imwrite(os.path.join(os.path.expanduser("~"), "yolo5_images", block_class, name), cv_image) #DA CAMBIARE
    
    detection(name, cv_image)
    

if __name__ == '__main__':
    print('esecuzione script yolov8_test')

    try:
        for arg in range (1,len(sys.argv)):
            name = sys.argv[arg]
            print(name)
            block_class = name.split('_')[0]
            print(block_class)
            talker(name)

    except rospy.ROSInterruptException:
        pass
