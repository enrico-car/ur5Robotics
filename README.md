# ur5Robotics



## Vision
We developed the vision part as the combination of two main techniques: Yolo and ICP.
We use Yolo in order to understand where the blocks are on the table, and then the ICP technique to truly obtain the center of the object and its orientation. 

#### Yolo - You Only Look Once
Yolo is a tool developed to perform real-time object detection on custom dataset of objects. 
We decided to use the fifth version and the largest model available in order to increase the precision, given that the blocks are similar.
To use Yolov5 we had to perform a training.
#### ICP - Iterativa Closest Point
