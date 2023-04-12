# ur5Robotics

## Kinematics

## Vision
We developed the vision part as the combination of two main techniques: Yolo and ICP.
We use Yolo in order to understand where the blocks are on the table, and then the ICP technique to truly obtain the center of the object and its orientation. 

#### Yolo - You Only Look Once
Yolo is a tool developed to perform real-time object detection on custom dataset of objects. 
We decided to use the fifth version and the largest model available in order to increase the precision, given that the blocks are similar.
To use Yolov5 we had to perform a training usign a custom dataset, that we realized using online tools. For each image in the dataset, there is a corresponding text file that describes the position and class of the box contining the object to detect. Clearly, the larger the dataset, the better the result, so we tried with a dataset of almost 600 images.
After the training, yolo elaborates the weights of the trained neural network that will be used for each detection. 

#### ICP - Iterativa Closest Point
In order to understand the center and the orientation of the block detected by yolo, we decided to use the ICP technique. The main difference from yolo is that ICP uses the point cloud of the area and not the plain image. ICP is an algorithm that aims to minimize the difference between two given point cloud. It is implemented by iterating a process of comparison between a point cloud that is fixed and another that is trasformed in order to best fit the KPI of the algorithm.
For this project we created point cloud templates for each object. Everytime an object is found by yolo, the corresponding area of the point cloud is compared with the templates and the one that best satisfies the chosen KPI gives both the center and the orientation od the block.
