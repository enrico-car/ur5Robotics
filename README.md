# ur5Robotics

#### introduzione al progetto: scopo, principali problemi da risolvere (cinematica, motion planning, vision)

## Kinematics
#### descrizione generale del concetto di cinematica per un braccio robotico
#### cinematica diretta, cinematica inversa (analitica, differenziale, differenze/pregi/difetti)
#### la nostra implementazione

## Vision
We developed the vision part as the combination of two main techniques: YOLO and ICP. We use Yolo in order to understand where the blocks are on the table, and then the ICP technique to truly obtain its position and orientation while also performing the classification. 

In the project the vision package is already configured to detect blocks. It automatically works if specified by the correspondent tag in the *[params.py](http://params.py)* file. It is suggested to use it, but it is possibile to disable it, for example to test the robot without considering the vision part. In that scenario, it is necessary to hard-code the position of the blocks directly in the *custom_publisher.cpp*.



## motion planning
#### path planning: come abbiamo fatto, miglioramenti possibili (per es. usando potenziale+discesa lungo il gradiente)
#### trajectory planning: come abbiamo fatto
