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

struct Detected
{
    Detected()
    {
        n = 0;
        xCenter.clear();
        yCenter.clear();
        zCenter.clear();
        roll.clear();
        pitch.clear();
        yaw.clear();
    };
    int n;
    std::vector<BlockClass> blockClass;
    std::vector<double> xCenter;
    std::vector<double> yCenter;
    std::vector<double> zCenter;
    std::vector<double> roll;
    std::vector<double> pitch;
    std::vector<double> yaw;

    int findFirst(BlockClass c)
    {
        for (int i = 0; i < n; i++)
        {
            if (blockClass[i] == c)
                return i;
        }
        std::cout << "block not found" << std::endl;
        return -1;
    }

    void remove(int i)
    {
        n--;
        blockClass.erase(blockClass.begin() + i - 1);
        xCenter.erase(xCenter.begin() + i - 1);
        yCenter.erase(yCenter.begin() + i - 1);
        zCenter.erase(zCenter.begin() + i - 1);
        roll.erase(roll.begin() + i - 1);
        pitch.erase(pitch.begin() + i - 1);
        yaw.erase(yaw.begin() + i - 1);
    }
};

BlockClass stringToBlockClass(std::string c)
{
    if (c == "X1_Y1_Z2")
    {
        return BlockClass::X1_Y1_Z2;
    }
    else if (c == "X1_Y2_Z1")
    {
        return BlockClass::X1_Y2_Z1;
    }
    else if (c == "X1_Y2_Z2")
    {
        return BlockClass::X1_Y2_Z2;
    }
    else if (c == "X1_Y2_Z2_CHAMFER")
    {
        return BlockClass::X1_Y2_Z2_CHAMFER;
    }
    else if (c == "X1_Y2_Z2_TWINFILLET")
    {
        return BlockClass::X1_Y2_Z2_TWINFILLET;
    }
    else if (c == "X1_Y3_Z2")
    {
        return BlockClass::X1_Y3_Z2;
    }
    else if (c == "X1_Y3_Z2_FILLET")
    {
        return BlockClass::X1_Y3_Z2_FILLET;
    }
    else if (c == "X1_Y4_Z1")
    {
        return BlockClass::X1_Y4_Z1;
    }
    else if (c == "X1_Y4_Z2")
    {
        return BlockClass::X1_Y4_Z2;
    }
    else if (c == "X2_Y2_Z2")
    {
        return BlockClass::X2_Y2_Z2;
    }
    else
    {
        return BlockClass::X2_Y2_Z2_FILLET;
    }
};

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

public:
    Block(){};
    Block(std::string name, BlockClass blockClass, Cartesian position, RPY rpy);
    Block(std::string name, BlockClass blockClass, Cartesian position, Quaternion orientation);

    void computeApproachAndLandPose(double xLandPose, double yLandPose, RPY finalRpy, double zOffset = 0.0, int yApproachAngle = 90);

    void update();
    void autoUpdate();
    RPY getFinalRpy();

    BlockConfiguration getConfiguration();
    BlockClass getClass();
    std::string getName();
    Cartesian getPosition();
    Vector3 getApproachPos();
    Matrix3 getApproachRotm();
    Vector3 getLandPos();
    Matrix3 getLandRotm();
    int getQuadrant();
    double getRpyY();
    RPY getRpy();
    bool getProcessed();
    void setProcessed(bool f);

    void setRpyY(double y);
    void setRpy(RPY newRpy);
    void setName(std::string n);
};

#endif
