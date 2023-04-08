import os
import sys
sys.path.insert(0, os.path.join(os.path.expanduser("~"),"ros_ws","src","locosim"))
from robot_control.vision.scripts.yolov5 import detect
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import cv2 as cv
import numpy as np
import open3d as o3d
from pprint import pprint
from math import pi as pi
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
import time
from gazebo_ros.gazebo_interface import SetModelState, ModelState, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

path_yolo = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "yolov5")
path_template_pcds = os.path.join(os.path.expanduser("~"), "ros_ws", "src", "locosim", "robot_control", "vision", "scripts", "template", "pcds")

block_names = ['X1-Y1-Z2','X1-Y2-Z1','X1-Y2-Z2','X1-Y2-Z2-CHAMFER','X1-Y2-Z2-TWINFILLET','X1-Y3-Z2','X1-Y3-Z2-FILLET','X1-Y4-Z1','X1-Y4-Z2','X2-Y2-Z2','X2-Y2-Z2-FILLET']
class2niter = {0: 12, 1: 24, 2: 24, 3: 40, 4: 20, 5: 24, 6: 40, 7: 24, 8: 24, 9: 12, 10: 40}
class2dimensions = {0: [0.031, 0.031, 0.057], 1: [0.031, 0.063, 0.039], 2: [0.031, 0.063, 0.057], 3: [0.031, 0.063, 0.057],
                    4: [0.031, 0.063, 0.057], 5: [0.031, 0.095, 0.057], 6: [0.031, 0.095, 0.057], 7: [0.031, 0.127, 0.039],
                    8: [0.031, 0.127, 0.057], 9: [0.063, 0.063, 0.057], 10: [0.063, 0.063, 0.057]}

block_class = 3
y_positions = [0.2584, 0.475, 0.6917]

altezza_tavolo = 1.8-0.935


def spawn_blocks():
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    for i in range(0, 3):
        model_name = 'brick'+str(i)+'_'+block_names[block_class]
        angles = [0, 0, 0]
        w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
        x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
        y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
        z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)
        orientation = Quaternion(x, y, z, w)
        
        spawn_model_client(
            model_name=model_name,
            model_xml=open('/home/carro/ros_ws/src/locosim/ros_impedance_controller/worlds/models/'+block_names[block_class]+'/model.sdf', 'r').read(),
            robot_namespace='',
            initial_pose=Pose(position=Point(0.5, y_positions[i], altezza_tavolo + class2dimensions[block_class][1]/2), orientation=orientation),
            reference_frame='world'
        )


def move_blocks(x, y, z, w, roll, pitch, yaw):
    print('moving blocks: ', x, y, z, w)

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    for i in range(0, 3):
        ms = ModelState()
        ms.model_name = 'brick'+str(i)+'_'+block_names[block_class]

        ms.pose.orientation.x = x
        ms.pose.orientation.y = y
        ms.pose.orientation.z = z
        ms.pose.orientation.w = w
        
        ms.pose.position.x = 0.5
        ms.pose.position.y = y_positions[i]
        
        ms.pose.position.z = altezza_tavolo + class2dimensions[block_class][1]/2 + 0.02
        
        print(set_state.call(ms))

    print('blocks moved')

    if pi-0.01 < pitch < pi+0.01:
        time.sleep(3.5)
    else:
        time.sleep(1.)


def namedef(posx, posy, r, p, y):
    if (-(pi/2)-0.01<=r<=-(pi/2)+0.01) :
        name = str(block_class)+"_"+str(posx)+"_"+str(posy)+"_"+str(abs(np.round(r,4)))+"_"+str(np.round(p,4))+"_"+str(np.round(y-pi,4))+".pcd"
    else:
        name = str(block_class)+"_"+str(posx)+"_"+str(posy)+"_"+str(abs(np.round(r,4)))+"_"+str(np.round(p,4))+"_"+str(np.round(y,4))+".pcd"
    print("name: " + name + "\n")

    return name


def get_template_position(xpos, ypos):
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


def create_quaternion(iter):
    if block_class == 4:
        if iter < 4:
            roll = 0
            pitch = 0
            yaw = iter * pi/4
        elif iter < 8:
            roll = 0
            pitch = pi/2
            yaw = (iter-4)*pi/4
        else:
            roll = 2.1356
            pitch = 0
            yaw = (iter-8)*pi/4
    
    #BLOCCHI RETTANGOLARI
    elif block_class in [1, 2, 5, 7, 8]:
        if iter < 4:
            roll = 0
            pitch = 0
            yaw = iter * pi/4
        elif iter < 8:
            roll = 0
            pitch = pi
            yaw = (iter-4)*pi/4
        elif iter < 16:
            roll = 0
            pitch = pi/2
            yaw = (iter-8)*pi/4
        else:
            roll = -pi/2
            pitch = 0
            yaw = (iter-16)*pi/4 + pi

    #BLOCCHI QUADRATI
    elif block_class in [0, 9]:
        if iter < 2:
            roll = 0
            pitch = 0
            yaw = iter*pi/4
        elif iter < 4:
            roll = 0
            pitch = pi
            yaw = (iter-2)*pi/4
        else:
            roll = -pi/2
            pitch = 0
            yaw = (iter-4)*pi/4 + pi

    #BLOCCO CHAMFER/FILLET
    else:
        if iter < 8: 
            roll = 0
            pitch = 0
            yaw = iter*pi/4
        elif iter < 16: 
            roll = 0
            pitch = pi
            yaw = (iter-8)*pi/4
        elif iter < 24:
            roll = 0
            pitch = pi/2
            yaw = (iter-16)*pi/4
        elif iter < 32:
            roll = 0
            pitch = 3*pi/2
            yaw = (iter-24)*pi/4
        else:
            pitch = 0
            roll = -pi/2
            yaw = (iter-32)*pi/4 + pi
    
    angles = [roll, pitch, yaw]
    print(angles)

    w = cos(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    x = sin(angles[0]/2)*cos(angles[1]/2)*cos(angles[2]/2) - cos(angles[0]/2)*sin(angles[1]/2)*sin(angles[2]/2)
    y = cos(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2) + sin(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2)
    z = cos(angles[0]/2)*cos(angles[1]/2)*sin(angles[2]/2) - sin(angles[0]/2)*sin(angles[1]/2)*cos(angles[2]/2)

    return x, y, z, w, roll, pitch, yaw


def presenceControl(arr, xmin, ymin, xmax, ymax):
    for xmin1, ymin1, xmax1, ymax1, _, _ in arr:
        if (xmin1-5 < xmin < xmin1+5  and ymin1-5 < ymin < ymin1 + 5) or (xmax1-5 < xmax < xmax1 + 5 and ymax1-5 < ymax < ymax1 + 5):
            return True
    return False


def checkTemplate(img_original, xmin, ymin, xmax, ymax):
    h, w = img_original.shape

    img = img_original[ymin:ymax, xmin:xmax]
    height, width = img.shape

    # cv.imshow('.', img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    
    if xmin <= 0 or xmax >= w-1 or ymin <= 0 or ymax >= h-1:
        return True, xmin, ymin, xmax, ymax

    n = 1
    expand = 5
    color = 220

    # controllo che i bordi siano tutti bianchi (max n pixel di fila)
    soglia1 = n
    soglia2 = n
    #p rint('sinistra - destra')
    for i in range(height):
        # prima colonna a sx
        if img[i, 0] < color:
            soglia1 -= 1
        else:
            soglia1 = n

        # ultima colonna a dx
        if img[i, width-1] < color:
            soglia2 -= 1
        else:
            soglia2 = n
        
        # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
        if soglia1 == 0:
            xmin -= expand
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax

        if soglia2 == 0:
            xmax += expand
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax

    soglia1 = n
    soglia2 = n
    # print('sopra - sotto')
    for j in range(width):
        # prima riga in alto
        if img[0, j] < color:
            soglia1 -= 1
        else:
            soglia1 = n

        #ultima riga in basso
        if img[height-1, j] < color:
            soglia2 -= 1
        else:
            soglia2 = n

        # se la soglia arriva a 0 -> sto tagliando male il template -> allargo il bounding box
        if soglia1 == 0:
            ymin -= expand
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax

        if soglia2 == 0:
            ymax += expand
            #print('--- correzione template ---')
            return False, xmin, ymin, xmax, ymax
    
    return True, xmin, ymin, xmax, ymax


def take_template(name):
    print('taking image')
    image = rospy.wait_for_message("/ur5/zed_node/left/image_rect_color", Image, timeout=None)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "mono8")

    cv.imwrite(os.path.join(path_yolo, 'cv_img.jpg'), cv_image)

    print('running yolo')
    detect.run()

    yolo_results = open(os.path.join(path_yolo, "res_save.txt"), 'r')
    results = yolo_results.readlines()

    processed_blocks = []
    for res in results:
        xmin, ymin, xmax, ymax, p, c = res.split(',')
        xmin, ymin, xmax, ymax, p, c = float(xmin), float(ymin), float(xmax), float(ymax), float(p), float(c)
        xmin, ymin, xmax, ymax, p, c = int(xmin)+3, int(ymin)+3, int(xmax)+3, int(ymax)+3, np.round(p,2), int(c)
        
        if presenceControl(processed_blocks, xmin, ymin, xmax, ymax):
            continue

        print('result: ', res)
        processed_blocks.append([xmin, ymin, xmax, ymax, p, c])

        ok = False
        while not ok:
            ok, xmin, ymin, xmax, ymax = checkTemplate(cv_image, xmin, ymin, xmax, ymax)

        print('reading pointcloud')
        gazebo_pcd = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, timeout=None)

        field_names = [field.name for field in gazebo_pcd.fields]

        uvs = []
        for i in range(xmin, xmax):
            for j in range(ymin, ymax):
                uvs.append([i, j])
        
        cloud_data = list(pc2.read_points(gazebo_pcd, field_names=field_names, skip_nans=True, uvs=uvs))
        
        xyz = [np.array([x,y,z]) for x,y,z,_ in cloud_data ]

        rot_z = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        rot_x = np.matrix([[1, 0, 0], [0, -0.5, 0.866], [0, -0.866, -0.5]])
        trasl = np.array([-0.4, 0.475, 1.45])
        
        xyz_worldframe = []
        for p in xyz:
            p_worldframe = np.array((np.dot(rot_z @ rot_x, p) + trasl).flat)
            if p_worldframe[2] > 0.866 and p_worldframe[1] > 0.155: # tolgo i punti del tavolo e del cassone a destra
                xyz_worldframe.append(p_worldframe)
        
        template_pcd = o3d.geometry.PointCloud()
        template_pcd.points = o3d.utility.Vector3dVector(np.array(xyz_worldframe))
        downpcd = template_pcd.voxel_down_sample(voxel_size=0.004)
        downpcd.paint_uniform_color([0, 1, 0])
        
        aabb = template_pcd.get_axis_aligned_bounding_box()
        aabb.color = (1, 0, 0)
        c = aabb.get_center()
        print('center: ', c)

        template_x, template_y = get_template_position(c[0], c[1])
        dir_name = str(template_x)+'_'+str(template_y)
        name = name.split('_')[3:]
        name = '_'.join([str(block_class), str(template_x), str(template_y)] + name)
        
        print('name')

        o3d.visualization.draw_geometries([downpcd])
        o3d.io.write_point_cloud(os.path.join(path_template_pcds, dir_name, name), downpcd)
        

def talker():
    rospy.init_node('icp_template', anonymous=True)
    rate = rospy.Rate(500)
    
    spawn_blocks()

    iter = 24
    end = class2niter[block_class]

    while not rospy.is_shutdown():
        x, y, z, w, roll, pitch, yaw = create_quaternion(iter)

        name = namedef(0.5, 0.475, roll, pitch, yaw)
        print('template: ', name)

        posx, posy = name.split('_')[1:3]
        dir_name = posx+'_'+posy

        file = os.path.join(path_template_pcds, dir_name, name)

        if not os.path.isfile(file):
            print('--- creazione template ---')
            
            move_blocks(x, y, z, w, roll, pitch, yaw)
            
            take_template(name)
        
        iter += 1
        print("iter: ", iter)

        if iter==end:
            exit(0)
        
        rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass