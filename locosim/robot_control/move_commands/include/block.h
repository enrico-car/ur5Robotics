#ifndef __BLOCK_H__
#define __BLOCK_H__

#include <iostream>
#include <string>
#include <map>
#include "algebra.h"

/*
 * Orientation of the blocks
 */
enum BlockConfiguration
{
    UP,
    SIDE,
    DOWN,
    REGULAR
};
/*
 * Classification of the blocks
 */
enum BlockClass
{
    X1_Y1_Z2,
    X1_Y2_Z1,
    X1_Y2_Z2,
    X1_Y2_Z2_CHAMFER,
    X1_Y2_Z2_TWINFILLET,
    X1_Y3_Z2,
    X1_Y3_Z2_FILLET,
    X1_Y4_Z1,
    X1_Y4_Z2,
    X2_Y2_Z2,
    X2_Y2_Z2_FILLET
};

/*
 * Definitions of the block dimension for each class
 */
const std::map<int, Cartesian> BlockDimension = {
    {0, Cartesian(0.031, 0.031, 0.057)},
    {1, Cartesian(0.031, 0.063, 0.039)},
    {2, Cartesian(0.031, 0.063, 0.057)},
    {3, Cartesian(0.031, 0.063, 0.057)},
    {4, Cartesian(0.031, 0.063, 0.057)},
    {5, Cartesian(0.031, 0.095, 0.057)},
    {6, Cartesian(0.031, 0.095, 0.057)},
    {7, Cartesian(0.031, 0.127, 0.039)},
    {8, Cartesian(0.031, 0.127, 0.057)},
    {9, Cartesian(0.063, 0.063, 0.057)},
    {10, Cartesian(0.063, 0.063, 0.057)},
};

class Block
{
private:
    std::string name;
    BlockClass blockClass;
    BlockConfiguration configuration;
    Cartesian position;
    RPY rpy;
    double height;
    Cartesian minPosition;
    Cartesian maxPosition;
    double zPos;
    double top;
    /*
    * Position on the desk
    */
    int quadrant;

    Matrix3 approachRotm;
    Cartesian approachPos;
    Matrix3 landRotm;
    Cartesian landPos;

    void update();
    void computeApproachAndLandPose(double xLandPose, double yLandPose, RPY finalRpy, double zOffset=0.0, int yApproachAngle=90);

public:
    Block(){};
    Block(std::string name, BlockClass blockClass, Cartesian position, RPY rpy);
    Block(std::string name, BlockClass blockClass, Cartesian position, Quaternion orientation);
    
};

#endif
