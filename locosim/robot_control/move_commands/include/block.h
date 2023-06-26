#ifndef __BLOCK_H__
#define __BLOCK_H__

#include <iostream>
#include <string>
#include <map>
#include "algebra.h"

/// @brief Orientation of the blocks
enum BlockConfiguration
{
    UP,
    SIDE,
    DOWN,
    REGULAR
};

/// @brief Classes of the blocks
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

/// @brief Definitions of the block dimension for each class
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

BlockClass stringToBlockClass(std::string c)
{
    if (c.compare("X1-Y1-Z2") == 0)
    {
        return BlockClass::X1_Y1_Z2;
    }
    else if (c.compare("X1-Y2-Z1") == 0)
    {
        return BlockClass::X1_Y2_Z1;
    }
    else if (c.compare("X1-Y2-Z2") == 0)
    {
        return BlockClass::X1_Y2_Z2;
    }
    else if (c.compare("X1-Y2-Z2-CHAMFER") == 0)
    {
        return BlockClass::X1_Y2_Z2_CHAMFER;
    }
    else if (c.compare("X1-Y2-Z2-TWINFILLET") == 0)
    {
        return BlockClass::X1_Y2_Z2_TWINFILLET;
    }
    else if (c.compare("X1-Y3-Z2") == 0)
    {
        return BlockClass::X1_Y3_Z2;
    }
    else if (c.compare("X1-Y3-Z2-FILLET") == 0)
    {
        return BlockClass::X1_Y3_Z2_FILLET;
    }
    else if (c.compare("X1-Y4-Z1") == 0)
    {
        return BlockClass::X1_Y4_Z1;
    }
    else if (c.compare("X1-Y4-Z2") == 0)
    {
        return BlockClass::X1_Y4_Z2;
    }
    else if (c.compare("X2-Y2-Z2") == 0)
    {
        return BlockClass::X2_Y2_Z2;
    }
    else
    {
        return BlockClass::X2_Y2_Z2_FILLET;
    }
};

/// @brief Final position of blocks in their coloured rectangle on the table
Cartesian finalPosFromClass(BlockClass c)
{
    switch (c)
    {
    case BlockClass::X1_Y1_Z2:
        return Cartesian(0.9655, 0.774, 0.0);
        break;
    case BlockClass::X1_Y2_Z1:
        return Cartesian(0.885, 0.774, 0.0);
        break;
    case BlockClass::X1_Y2_Z2:
        return Cartesian(0.79, 0.774, 0.0);
        break;
    case BlockClass::X1_Y2_Z2_CHAMFER:
        return Cartesian(0.69, 0.774, 0.0);
        break;
    case BlockClass::X1_Y2_Z2_TWINFILLET:
        return Cartesian(0.947, 0.708, 0.0);
        break;
    case BlockClass::X1_Y3_Z2:
        return Cartesian(0.83, 0.708, 0.0);
        break;
    case BlockClass::X1_Y3_Z2_FILLET:
        return Cartesian(0.69, 0.708, 0.0);
        break;
    case BlockClass::X1_Y4_Z1:
        return Cartesian(0.91, 0.65, 0.0);
        break;
    case BlockClass::X1_Y4_Z2:
        return Cartesian(0.72, 0.65, 0.0);
        break;
    case BlockClass::X2_Y2_Z2:
        return Cartesian(0.947, 0.575, 0.0);
        break;
    case BlockClass::X2_Y2_Z2_FILLET:
        return Cartesian(0.85, 0.575, 0.0);
        break;
    default:
        return Cartesian(0.75, 0.58, 0.0);
        break;
    }
}

/// @brief Class to manage the block positions and orientation during the procedures
class Block
{
private:
    std::string name;
    BlockClass blockClass;
    BlockConfiguration configuration;
    Cartesian position;
    Cartesian finalPos;
    RPY rpy;
    RPY fRpy;
    bool processed;
    double height;
    double radius;
    double distanceFromShoulder;
    Cartesian minPosition;
    Cartesian maxPosition;
    double zPos;
    double top;
    int quadrant;
    
    Matrix3 approachRotm90;
    Matrix3 approachRotm;
    Cartesian approachPos;
    Matrix3 landRotm;
    Cartesian landPos;

public:
    Block(){};
    Block(std::string name, BlockClass blockClass, Cartesian position, RPY rpy);
    Block(std::string name, BlockClass blockClass, Cartesian position, Quaternion orientation);

    /// @brief Compute the approach and land pose for the end effector
    /// @param xLandPose x coordinate of final position
    /// @param yLandPose y coordinate of final position
    /// @param finalRpy final orientation
    /// @param zOffset offset of the z axe
    /// @param yApproachAngle angle of end effector during the approach pose
    void computeApproachAndLandPose(double xLandPose, double yLandPose, RPY finalRpy, double zOffset = 0.0, int yApproachAngle = 90);

    /// @brief Update the block params
    void update();
    /// @brief set position and rpy before calling [update] function
    void autoUpdate();
    

    BlockConfiguration getConfiguration();
    BlockClass getClass();
    std::string getName();
    Cartesian getPosition();
    Vector3 getApproachPos();
    Matrix3 getApproachRotm90();
    Matrix3 getApproachRotm();
    Vector3 getLandPos();
    Matrix3 getLandRotm();
    RPY getFinalRpy();
    int getQuadrant();
    double getRpyY();
    RPY getRpy();
    bool getProcessed();
    double getDistanceFromShoulder();
    double getRadius();

    void setProcessed(bool f);
    void setRpyY(double y);
    void setRpy(RPY newRpy);
    void setName(std::string n);

    void print();
};

#endif
