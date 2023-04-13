# ur5Robotics

## Kinematics

## Vision
We developed the vision part as the combination of two main techniques: YOLO and ICP.
We use Yolo in order to understand where the blocks are on the table, and then the ICP technique to truly obtain its position and orientation. 

#### Yolo - You Only Look Once
Yolo is a tool developed to perform real-time object detection on custom dataset of objects. 
We decided to use the fifth version and the largest model available in order to increase the precision, given that the blocks are quite similar.
To use YOLOv5 we had to perform a training usign a custom dataset, that we realized using online tools. For each image in the dataset, there is a corresponding text file that describes position and class of the bounding box containing the object to detect. Clearly, the larger the dataset, the better the result, so we tried with a dataset of almost 600 images.
After the training, YOLO neural network is ready to be used for the detection of the blocks. 

#### ICP - Iterativa Closest Point
In order to understand class, position and orientation of the blocks detected by YOLO, we decided to use the ICP technique. The main difference from YOLO is that ICP uses pointclouds instead of plain images. ICP is an algorithm that aims to minimize the difference between two given pointclouds. It is implemented by iterating a process of comparison between a pointcloud that is fixed and another that is trasformed in order to best fit the KPI of the algorithm.
For this project we created pointcloud templates for each object. Everytime an object is found by YOLO, the corresponding area of the pointcloud is compared with the templates and the one that best satisfies the chosen KPI gives both class and orientation of the block.
The position of the block is obtained by computing the 3d bounding box of the block's pointcloud and calculating its center.
