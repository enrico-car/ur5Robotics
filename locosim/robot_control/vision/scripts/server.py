#!/usr/bin/env python

import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim"))
from robot_control.vision.scripts.yolov5 import detect
from vision.srv import vision, visionResponse
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import ast
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import cmath
from math import pi, cos, atan2
import copy

locosim = os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim")
path_stls = os.path.join(locosim, "ros_impedance_controller", "worlds", "models")
path_yolo = os.path.join(locosim, "robot_control", "vision", "scripts", "yolov5")
path_vision = os.path.join(locosim, "robot_control", "vision")
path_template = os.path.join(locosim, "robot_control", "vision", "scripts", "template")
path_template_pcds = os.path.join(locosim, "robot_control", "vision", "scripts", "template", "pcds")
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
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.processed = []
        self.raw_pcd = None
        self.gazebo_pcd = None
        self.pcd_axis_aligned_bb = None
        self.pcd_center = None
    
    def savePointCloudTemplate(self):
        print('########## SAVING PCD ##########')
        pcd_center, pcd_dimensions = self.getPointcloudInfo()
        temp_x, temp_y = self.getTemplatePosition(pcd_center[0], pcd_center[1])
        temp_x = 0.5
        template_dir = str(temp_x)+'_'+str(temp_y)
        name = '0_'+template_dir+'_0_3.1415_0.3927.pcd'
        print(name)

        o3d.visualization.draw_geometries([self.gazebo_pcd])
        o3d.io.write_point_cloud(os.path.join(path_template_pcds, template_dir, name), self.gazebo_pcd)
        input('...')
    
    # given the image from yolo and the original bounding box, 
    # extend it if the block is cut by the borders
    def checkTemplate(self, img_original, xmin, ymin, xmax, ymax):
        h, w = img_original.shape

        img = img_original[ymin:ymax, xmin:xmax]
        height, width = img.shape
        
        if xmin <= 0 or xmax >= w-1 or ymin <= 0 or ymax >= h-1:
            return True, xmin, ymin, xmax, ymax

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
            
            # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
            if soglia1 == 0:
                xmin -= 1
                #print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax

            if soglia2 == 0:
                xmax += 1
                #print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax

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

            # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
            if soglia1 == 0:
                ymin -= 1
                #print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax

            if soglia2 == 0:
                ymax += 1
                #print('--- correzione template ---')
                return False, xmin, ymin, xmax, ymax
        
        return True, xmin, ymin, xmax, ymax

    # based on where the block is, decide in which main position it is (left, center, right)
    def getTemplatePosition(self, xpos, ypos):
        if 0. < xpos < 0.333:
            template_x = 0.1667
        elif 0.333 <= xpos <= 0.667:
            template_x = 0.5
        else:
            template_x = 0.8333
        
        if 0. < ypos < 0.367:
            template_y = 0.2584
        elif 0.367 <= ypos <= 0.5834:
            template_y = 0.475
        else:
            template_y = 0.6917
        
        return template_x, template_y

    # get the block's pointcloud extension and center
    def getPointcloudInfo(self):
        self.pcd_axis_aligned_bb = self.raw_pcd.get_axis_aligned_bounding_box()
        self.pcd_axis_aligned_bb.color = (1, 0, 0)
        pcd_center = self.pcd_axis_aligned_bb.get_center()
        self.pcd_center = pcd_center

        # if a block in the same position is already been read, do not consider it
        for xc, yc in zip(self.x_tavolo, self.y_tavolo):
            if xc-0.01 < pcd_center[0] < xc+0.01 and yc-0.01 < pcd_center[1] < yc+0.01:
                return None, None

        pcd_dimensions = self.pcd_axis_aligned_bb.get_extent()

        return pcd_center, pcd_dimensions
    
    # given the yolo bounding box (xmin-xmax, ymin-ymax), subscribe to the pointcloud topic and save one, 
    # then cut out the portion of interest, i.e. the block
    def prepareGazeboPointCloud(self, xmin, ymin, xmax, ymax):
        raw_pointcloud = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, timeout=None)
        field_names = [field.name for field in raw_pointcloud.fields]

        # retrive from the pc the center of the bounding box to check if it is a block or not quicker
        data_out = list(pc2.read_points(raw_pointcloud, field_names=field_names, skip_nans=False, uvs=[[int((xmax+xmin)/2), int(((ymax+ymin)/2))]]))
        xyz = np.array(np.array(data_out).flat)

        rot_z = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        rot_x = np.matrix([[1, 0, 0], [0, -0.5, 0.866], [0, -0.866, -0.5]])
        res = np.dot(rot_z @ rot_x, np.array([xyz[0], xyz[1], xyz[2]])) + np.array([-0.4, 0.475, 1.45])
        
        # check on the position of the pc (on the z-value)
        if res[0, 2] > 1.1:
            print('detection not correct - it is not a block (probably the arm or gripper)')
            return False

        cv_image = cv2.imread(os.path.join(locosim,"robot_control","vision","scripts","yolov5","cv_img.jpg"), 0)
        ok = False
        while not ok:
            ok, xmin, ymin, xmax, ymax = self.checkTemplate(cv_image, xmin, ymin, xmax, ymax)
        
        # now get the whole pc for the block
        uvs = []
        for i in range(int(xmin), int(xmax)):
            for j in range(int(ymin), int(ymax)):
                uvs.append([i, j])
        
        cloud_data = list(pc2.read_points(raw_pointcloud, field_names=field_names, skip_nans=True, uvs=uvs))
        
        # transform the coordinates in the world frame
        xyz = [np.array([x,y,z]) for x,y,z,_ in cloud_data ]
        xyz_worldframe = []
        for p in xyz:
            p_worldframe = np.array((np.dot(rot_z @ rot_x, p) + np.array([-0.4, 0.475, 1.45])).flat)
            if p_worldframe[2] > 0.866 and p_worldframe[1] > 0.155:
                xyz_worldframe.append(p_worldframe)
        
        raw_gazebo_pcd = o3d.geometry.PointCloud()
        if len(xyz_worldframe):
            raw_gazebo_pcd.points = o3d.utility.Vector3dVector(np.array(xyz_worldframe))
            raw_gazebo_pcd.paint_uniform_color([0, 0, 1])
            
            # o3d.visualization.draw_geometries([raw_gazebo_pcd])
            self.raw_pcd = raw_gazebo_pcd
            gazebo_pcd = raw_gazebo_pcd.voxel_down_sample(voxel_size=0.004)
            gazebo_pcd.paint_uniform_color([0, 0, 1])

            self.gazebo_pcd = gazebo_pcd
 
            # self.savePointCloudTemplate()

            return True
        else:
            print('error reading the pointcloud, no point registered')
            return False

    def bbDistance(self, template_pcd, transformation):
        template_pcd.transform(transformation)
        bb = template_pcd.get_axis_aligned_bounding_box()
        return np.linalg.norm(self.pcd_axis_aligned_bb.get_extent()-bb.get_extent())

    # execute ICP algorithm, choose the best fit and return the best transformation
    def getTransformation(self, gazebo_pcd_dimensions, template_dir):
        print("------------------------running ICP----------------------------")
        
        # import the names of the pc templates
        template_files = []
        for pc in os.listdir(os.path.join(path_template_pcds, template_dir)):
            template_files.append(pc)
        template_files.sort()
        
        # compute the translation error between the template and the pc just read
        x_offset = self.pcd_center[0] - 0.5
        y_offset = self.pcd_center[1] - float(template_dir.split('_')[1])
        trans_mat = np.eye(4)
        trans_mat[0:3, 3] = [x_offset, y_offset, 0]

        # parameters of evaluation of the ICP algorithm
        top_result = ''
        best_transformation = None
        best_fitness = 0.
        best_correspondence = 0
        best_rmse = 1.
        for pc in template_files:
            # pcd BLU -> reading from yolo (of which we have to determine class and orientation)
            # pcd RED -> pcd template (of known class and orientation)
            
            template_pcd = o3d.io.read_point_cloud(os.path.join(path_template_pcds, template_dir, pc))
            template_bb = template_pcd.get_axis_aligned_bounding_box()
            template_dimensions = template_bb.get_extent()
            
            ok = False
            if template_dimensions[2]-0.007 < gazebo_pcd_dimensions[2] < template_dimensions[2]+0.0025:
                if gazebo_pcd_dimensions[1]*0.9 < template_dimensions[1] < gazebo_pcd_dimensions[1]*1.1 or gazebo_pcd_dimensions[0]*0.9 < template_dimensions[0] < gazebo_pcd_dimensions[0]*1.1:
                    ok = True
            
            if not ok:
                continue
            
            # print(pc)
            
            template_pcd.paint_uniform_color([1, 0, 0])

            source = copy.deepcopy(template_pcd)
            source.transform(trans_mat)
            target = copy.deepcopy(self.gazebo_pcd)
            
            trans_init = np.eye(4)

            threshold = 0.009
            reg_p2p = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint())
            
            # compare the last results with the best till now and decide which is the best
            if reg_p2p.fitness >= best_fitness*0.95:
                dist = self.bbDistance(source, reg_p2p.transformation)
                if reg_p2p.fitness >= 1. and reg_p2p.inlier_rmse < best_rmse and dist < 0.0075 and len(reg_p2p.correspondence_set) > best_correspondence:
                    best_fitness = reg_p2p.fitness
                    best_rmse = reg_p2p.inlier_rmse
                    best_correspondence = len(reg_p2p.correspondence_set)
                    top_result = pc
                    best_transformation = reg_p2p.transformation
                
                elif reg_p2p.fitness >= 0.95 and dist < 0.01:
                    if len(reg_p2p.correspondence_set) >= best_correspondence and reg_p2p.inlier_rmse*0.8 <= best_rmse:
                        best_fitness = reg_p2p.fitness
                        best_rmse = reg_p2p.inlier_rmse
                        top_result = pc
                        best_correspondence = len(reg_p2p.correspondence_set)
                        best_transformation = reg_p2p.transformation

                else:
                    if reg_p2p.inlier_rmse <= best_rmse and dist < 0.01:
                        best_fitness = reg_p2p.fitness
                        best_rmse = reg_p2p.inlier_rmse
                        top_result = pc
                        best_transformation = reg_p2p.transformation

        if top_result != '':
            return top_result, best_transformation
        else:
            print('###### NO successful match ######')
            return None, None

    # get euler angles from the transformation
    def getAngles(self, transformation, result):
        rotm = transformation[0:3, 0:3]
        
        theta1 = -np.real(cmath.asin(rotm[2][0]))
        theta2 = pi - theta1

        phi1 = np.round(atan2(rotm[1][0]/cos(theta1), rotm[0][0]/cos(theta1)), 4)
        phi2 = np.round(atan2(rotm[1][0]/cos(theta2), rotm[0][0]/cos(theta2)), 4)

        info = result.split('_')
        roll = float(info[3])
        pitch = float(info[4])
        major_yaw = float(info[5][:len(info[5])-4]) # to take out .jpg
        
        yaw1 = np.round(phi1 + major_yaw, 4)
        yaw2 = np.round(phi2 + major_yaw, 4)

        if 0.0-0.01 < roll < 0.0+0.01 and 0.0-0.01 < pitch < 0.0+0.01 or pi-0.01 < pitch < pi+0.01:
            if 0.0 < yaw1 < pi:
                yaw = yaw1
            else:
                yaw = yaw2
        else:
            yaw = np.round((yaw1 + 2*pi) % (2*pi), 4)

        return roll, pitch, yaw

    # main function that processes the block detection
    def call(self, xmin, xmax, ymin, ymax, classe=None):
        ok = self.prepareGazeboPointCloud(xmin, ymin, xmax, ymax)

        if not ok:
            return False
        
        pcd_center, pcd_dimensions = self.getPointcloudInfo()
        if pcd_center is None and pcd_dimensions is None:
            return False

        temp_x, temp_y = self.getTemplatePosition(pcd_center[0], pcd_center[1])
        temp_x = 0.5
        template_dir = str(temp_x)+'_'+str(temp_y)

        top_result, best_transformation = self.getTransformation(pcd_dimensions, template_dir)
        
        if top_result is not None and best_transformation is not None:
            roll, pitch, yaw = self.getAngles(best_transformation, top_result)
            
            info = top_result.split('_')
            classe = info[0]
            self.cl.append(int(classe))
            self.roll.append(roll)
            self.pitch.append(pitch)
            self.yaw.append(yaw)
            self.x_tavolo.append(np.round(pcd_center[0], 4))
            self.y_tavolo.append(np.round(pcd_center[1], 4))
            
            return True
        
        return False
        
# check if the bounding box is overlapping with another one too much (means it is the same detection)
def presenceControl(arr, xmin, ymin, xmax, ymax):
    for xmin1, ymin1, xmax1, ymax1, _, _ in arr:
        if (xmin1-5 < xmin < xmin1+5 and ymin1-5 < ymin < ymin1+5) or (xmax1-5 < xmax < xmax1+5 and ymax1-5 < ymax < ymax1+5):
            return True
    return False

# process the results from yolo
def dataProcessing():
    det_res = []

    with open(os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "yolov5", "res_save.txt")) as file:
        det_res = [ast.literal_eval(line.rstrip("\n")) for line in file]
    
    blocks_info = []
    if len(det_res):
        l = Listener()

        for xmin, ymin, xmax, ymax, confidence, classe in det_res:
            if ymax < 420:
                continue
            
            block_presence = presenceControl(blocks_info, xmin, ymin, xmax, ymax)

            if not block_presence:
                if confidence > 0.94:
                    ok = l.call(xmin, xmax, ymin, ymax, int(classe))
                else:
                    ok = l.call(xmin, xmax, ymin, ymax)
                
                if not ok:
                    continue
                
                blocks_info.append([xmin, ymin, xmax, ymax, confidence, classe])

        print("results in data: ", l.cl, l.x_tavolo, l.y_tavolo, l.roll, l.pitch, l.yaw)
        return visionResponse(len(l.cl), l.cl, l.x_tavolo, l.y_tavolo, l.roll, l.pitch, l.yaw, l.processed)

# running yolo script
def detection():
    print("------------------------running yolo----------------------------")
    # os.system("python3 " + path_yolo + "/detect.py --weights " + path_yolo + "/best2.pt --source " + path_yolo + "/cv_img.jpg --data " + path_yolo + "/data.yaml")
    detect.run()

# take image from the ROS topic and convert it to opencv image
def retImage():
    print("------------------------taking image----------------------------")
    image = rospy.wait_for_message("/ur5/zed_node/left/image_rect_color", Image, timeout=None)
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
