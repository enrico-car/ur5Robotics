# Ur5Robotics - Tino

> ~ Tino il robottino che sposta il blocchettino ~

## Setup Environment

This project is a fork of the repository https://github.com/mfocchi/locosim so before starting, follow the locosim installation rules.

After installing the requirements, copy all the files in the ros workspace folder and compile all the project with the following commands:

``` Bash
$ cd ros_ws
$ catkin_make install
```

Remove the `CATKIN_IGNORE` file inside the `ros_ws/src/locosim/robot_control/move_commands` folder and run the previous command again. Because some of the requirements needed by the move_commands scripts are created with the first compilation.

At the end, modify the absolute path inside `ros_ws/src/locosim/robot_control/move_commands/header/constants.h` with the path of the `output.json` inside the `castle_build_path` directory.

``` C++
const std::string Constants::jsonOutput = "path/to/output.json";
```

## Project Structure

### Move_commands

```bash
├── CMakeLists.txt
├── include
│   ├── algebra.h
│   ├── block.h
│   ├── constants.h
│   ├── joint_state_publisher.h
│   └── kinematic.h
├── package.xml
└── src
    ├── algebra.cpp
    ├── assignment_123.cpp
    ├── assignment_4.cpp
    ├── block.cpp
    ├── constants.cpp
    ├── joint_state_publisher.cpp
    └── kinematic.cpp

```

The `movement_commands` contain all the scripts for moving the robot and the scripts for communicating with the ros topic.

In the `constants.h` file are defined all the constant variables, such as ros topics and robot size.

The `ALgebra.h` file contains the rotation matrices and conversion functions to be used in the `kinematic.h` file, where functions for direct, inverse and differential kinematics are defined.

The `Block.h` file contain all the classes and methods to handle the block position and rotations during the 

The file `joint_state_publisher.h` groups functions for communicating with the topics and the procedures for the path planning.

`assignment_123.cpp` and  `assignment_4.cpp` are the procedures to execute the given assignment.

### Ur5_generic.py

The file `ur5_generic.py` is the script that prepares and launches the simulation environment, generating the manipulator robot and the work table. Depending on the parameters, this script generates the necessary blocks for the selected assignment.

### Vision

```bash
├── CMakeLists.txt
├── package.xml
├── scripts
│   ├── SpawnBlocks_temp.py
│   ├── template
│   └── yolov5
└── srv
    └── vision.srv
```

This is the folder containing the script for the object detection.

The `server.py` is initialize from the `ur5_generic`. This file when is running, expose a service callback for run the yolov5 and ICP detection. The service client is initialize in the  `joint_state_publisher.cpp` constructor

```
if (vision)
    {
        visionService = n.serviceClient<vision::vision>(Constants::visionService);
        visionService.waitForExistence();
        std::cout << "----- vision server started -----" << std::endl;
    }
```

and with the `visionService` the system requests vision data from the server

```c++
std::cout << "Vision Client" << std::endl;
    vision::vision visionResult;
    if (joint_state_publisher.visionService.call(visionResult))
    {
        std::cout << "Vision Client | SUCCESS" << std::endl;
        std::cout << "#n res: " << (int)visionResult.response.n_res << std::endl;
    }
    else
    {
        std::cout << "Vision Client | ERROR" << std::endl;
        return 1;
    }
```

### Castle_build_path

```
└── castle_build_path
    ├── block
    │   ├── Block.java
    │   ├── X1_Y1_Z2.java
    │   ├── X1_Y2_Z1.java
    │   ├── X1_Y2_Z2_CHAMFER.java
    │   ├── X1_Y2_Z2.java
    │   ├── X1_Y2_Z2_TWINFILLET.java
    │   ├── X1_Y3_Z2_FILLET.java
    │   ├── X1_Y3_Z2.java
    │   ├── X1_Y4_Z1.java
    │   ├── X1_Y4_Z2.java
    │   ├── X2_Y2_Z2_FILLET.java
    │   └── X2_Y2_Z2.java
    ├── CastleBuildPath.java
    ├── tolls
    │   ├── ReadJson.java
    │   └── WriteJson.java
    └── view
        ├── ButtonColumn.java
        ├── CubeBlock.java
        ├── GridBlock.java
        ├── PrettyButton.java
        ├── Square.java
        └── SquarePos.java


```

This simple javaFx project allow an easy way to write the `.json` file for the castle configuration use in the assignment 4.

## Block classes available

|     class name      |      block type      |
| :-----------------: | :------------------: |
|      X1-Y1-Z2       |  ![0](assets/0.jpg)  |
|      X1-Y2-Z1       |  ![1](assets/1.jpg)  |
|      X1-Y2-Z2       |  ![2](assets/2.jpg)  |
|  X1-Y2-Z2-CHAMFER   |  ![3](assets/3.jpg)  |
| X1-Y2-Z2-TWINFILLET |                      |
|      X1-Y3-Z2       |  ![5](assets/5.jpg)  |
|   X1-Y3-Z2-FILLET   |  ![6](assets/6.jpg)  |
|      X1-Y4-Z1       |  ![7](assets/7.jpg)  |
|      X1-Y4-Z2       |  ![8](assets/8.jpg)  |
|      X2-Y2-Z2       |  ![9](assets/9.jpg)  |
|   X2-Y2-Z2-FILLET   | ![10](assets/10.jpg) |



## Run assignment procedures

The project specifics are described in the following [file](assets/project.pdf)

### Assignment 1

> There is only one object in the initial stand, which is positioned with its base “naturally” in
> contact with the ground. The object can be of any of the classes specified by the project.
> Each class has an assigned position on the final stand, which is marked by a coloured shape
> representing the silhouette of the object.

First launch the environment with one of the block class `<block-class>` defined in the table above, by running:

```bash
$ ./launchWorld 1 <block-class>
```

When the block is spawned run:

```bash
$ rosrun move_commands assignment_123
```

### Assignment 2

> There are multiple objects on the initial stand, one for each class. There is no specific order
> in the initial configuration, except that the base of the object is “naturally” in contact with the
> ground. Each object has to be picked up and stored in the position prescribed for its class
> and marked by the object’s silhouette.

First launch the environment with one of the block class `<block-class-n>` defined in the table above, by running:

```bash
$ ./launchWorld 2 <block-class-1> <block-class1> ...
```

When the block is spawned run:

```bash
$ rosrun move_commands assignment_123
```

### Assignment 3

> There are multiple objects on the initial stand, and there can be more than one object for
> each class. The objects are positioned randomly on the stand but would not stand or lean on
> each other. An object could be lying on one of its lateral sides or on its top. Each object has
> to be stored in the position prescribed by its class. Objects of the same class have to be
> stacked up to form a tower.

First launch the environment with one of the block class `<block-class-n>` defined in the table above, by running:

```bash
$ ./launchWorld 3 <block-class-1> <block-class1> ...
```

When the block is spawned run:

```bash
$ rosrun move_commands assignment_123
```

### Assignment 4

> The objects on the initial stand are those needed to create a composite object with a known
> design (e.g., a castle). The objects are positioned randomly on the stand. An object could be
> lying on one of its lateral sides or on its top. The objects could also stand or lean on each
> other. The manipulator has to pick them up in sequence and create the desired composite
> object on the final stand.

First launch the `castle_build_path`  project and choose the configuration of the castle

First launch the environment with one of the block class `<block-class-n>` defined in the table above, by running:

```bash
$ ./launchWorld 4
```

When the block is spawned run:

```bash
$ rosrun move_commands assignment_4
```

## Acknowledgments

Students: [Daniele Carraro](https://github.com/d-aniele-carrar-o) - [Annachiara Fortuna](https://github.com/achf01) - [Enrico Carnelos](https://github.com/enrico-car)

Project of course **Fondamenti di robotica [145831]** - [PALOPOLI LUIGI](https://webapps.unitn.it/du/it/Persona/PER0002392/Curriculum) - [SEBE NICULAE](https://webapps.unitn.it/du/it/Persona/PER0051994/Curriculum) - [FOCCHI MICHELE](https://webapps.unitn.it/du/it/Persona/PER0221571/Didattica) - [FALQUETO PLACIDO](https://webapps.unitn.it/du/it/Persona/PER0230044/Curriculum)

[![img](assets/unitn_logo.png)](https://www.unitn.it/)
