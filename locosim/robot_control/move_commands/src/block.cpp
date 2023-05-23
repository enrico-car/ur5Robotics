#include "block.h"

Block::Block(std::string name, BlockClass blockClass, Cartesian position, RPY rpy)
{
    this->name = name;
    this->blockClass = blockClass;
    this->position = position;
    this->rpy = rpy;
    this->processed = false;
    update();
}

Block::Block(std::string name, BlockClass blockClass, Cartesian position, Quaternion orientation)
{
    this->name = name;
    this->blockClass = blockClass;
    this->position = position;
    this->rpy = orientation.toRpy();
    this->processed = false;
    update();
}

BlockConfiguration Block::getConfiguration()
{
    return configuration;
}

BlockClass Block::getClass()
{
    return blockClass;
}

std::string Block::getName()
{
    return name;
}

Cartesian Block::getPosition()
{
    return position;
}

Vector3 Block::getApproachPos()
{
    return approachPos.toVector();
}

Matrix3 Block::getApproachRotm()
{
    return approachRotm;
}

Vector3 Block::getLandPos()
{
    return landPos.toVector();
}

Matrix3 Block::getLandRotm()
{
    return landRotm;
}

int Block::getQuadrant()
{
    return quadrant;
}

double Block::getRpyY()
{
    return rpy.y;
}

RPY Block::getRpy()
{
    return rpy;
}

bool Block::getProcessed()
{
    return processed;
}

void Block::setRpyY(double y)
{
    rpy.y = y;
}

void Block::setRpy(RPY newRpy)
{
    rpy = newRpy;
}

void Block::setName(std::string n)
{
    name = n;
}

void Block::setProcessed(bool flag)
{
    processed = flag;
}

void Block::update()
{
    Cartesian dim = BlockDimension.at(blockClass);
    double s1, s2;
    // Block up configuration
    if ((M_PI / 2 - 0.01 < rpy.r && rpy.r < M_PI / 2 + 0.01))
    {
        s1 = abs(dim.x * cos(rpy.y)) + abs(dim.z * sin(rpy.y));
        s2 = abs(dim.x * sin(rpy.y)) + abs(dim.z * cos(rpy.y));
        position.z = DESK_HEIGHT + dim.y / 2;
        height = dim.y;
        configuration = BlockConfiguration::UP;
    }
    // Block side configuration
    else if (M_PI / 2 - 0.01 < rpy.p && rpy.p < M_PI / 2 + 0.01)
    {
        s1 = abs(dim.y * sin(rpy.y)) + abs(dim.z * cos(rpy.y));
        s2 = abs(dim.y * cos(rpy.y)) + abs(dim.z * sin(rpy.y));
        position.z = DESK_HEIGHT + dim.x / 2;
        height = dim.y;
        configuration = BlockConfiguration::SIDE;
    }
    else
    {
        s1 = abs(dim.y * sin(rpy.y)) + abs(dim.x * cos(rpy.y));
        s2 = abs(dim.y * cos(rpy.y)) + abs(dim.x * sin(rpy.y));
        position.z = DESK_HEIGHT + dim.z / 2;
        height = dim.z;
        // Block down configuration
        if (M_PI - 0.01 < rpy.y && rpy.y < M_PI + 0.01)
        {
            configuration = BlockConfiguration::DOWN;
        }
        // Block regular configuration
        else
        {
            configuration = BlockConfiguration::REGULAR;
        }
    }

    if (blockClass == BlockClass::X2_Y2_Z2 || blockClass == BlockClass::X2_Y2_Z2_FILLET)
    {
        position.z += 0.03;
    }

    // Cartesian s(s1 / 2, s2 / 2, 0);
    // minPosition = position - s;
    // maxPosition = position + s;

    zPos = DESK_HEIGHT + dim.x / 2;
    top = DESK_HEIGHT + height;

    if (rpy.y < 0)
    {
        rpy.y = 2 * M_PI - rpy.y;
    }

    // desk position
    if (position.x > 0.5 && position.y <= 0.475)
    {
        quadrant = 1;
    }
    else if (position.x > 0.5 && position.y > 0.475)
    {
        quadrant = 2;
    }
    else if (position.x <= 0.5 && position.y > 0.475)
    {
        quadrant = 3;
    }
    else
    {
        quadrant = 4;
    }
    std::cout << "Block updated" << std::endl;
}

void Block::autoUpdate()
{
    position = finalPos;
    rpy = fRpy;
    update();
}

RPY Block::getFinalRpy()
{
    if (!(-0.01 < rpy.p && rpy.p < 0.01))
    {
        double roll = M_PI / 2;
        double yaw = M_PI / 2;

        if (M_PI / 2 - 0.01 < rpy.p && rpy.p < M_PI / 2 + 0.01)
        {
            if (blockClass > 0 && blockClass < 5)
            {
                roll = 0;
            }
            if (quadrant == 1 || quadrant == 4)
            {
                yaw = M_PI;
            }
        }
        else
        {
            if (position.x < 0.5)
            {
                yaw = M_PI / 2;
            }
            else
            {
                yaw = 3 * M_PI / 2;
            }
        }
        return RPY(roll, 0, yaw);
    }
    // TODO there was else if not else
    else
    {
        return RPY(0, 0, M_PI / 2);
    }
}

void Block::computeApproachAndLandPose(double xLandPose, double yLandPose, RPY finalRpy, double zOffset, int yApproachAngle)
{
    RPY approachRPY(M_PI, 0, rpy.y + M_PI / 2);
    RPY landRPY(M_PI, 0, finalRpy.y + M_PI / 2);

    Cartesian landPosNoCorrection;

    if (configuration == BlockConfiguration::REGULAR || yApproachAngle == 90)
    {
        approachRotm = Algebra::eul2RotM(approachRPY.toVector());
        approachPos = position;

        if (configuration == BlockConfiguration::REGULAR && blockClass > 0 && blockClass < 8)
        {
            approachPos = approachPos + Cartesian(0, 0, -0.008);
        }

        if (yApproachAngle == 90 && configuration == BlockConfiguration::UP)
        {
            approachPos.z = top - 0.015;
        }

        landRotm = Algebra::eul2RotM(landRPY.toVector());
        landPos = Cartesian(xLandPose, yLandPose, approachPos.z);

        landPosNoCorrection = landPos;
    }
    else if (yApproachAngle == 45)
    {
        Cartesian correctionApproach45, correctionLand45;
        double landZPos;
        double theta = M_PI / 4;

        Matrix3 apprachRotm90 = Algebra::eul2RotM(approachRPY.toVector());
        approachRotm = apprachRotm90 * Algebra::rotY(theta);

        if (configuration == BlockConfiguration::UP)
        {
            landRPY = landRPY + RPY(0, 0, -M_PI);

            double d = 0.00;
            if (blockClass == BlockClass::X2_Y2_Z2)
            {
                d = 0.016;
            }
            else if (blockClass == BlockClass::X2_Y2_Z2_FILLET)
            {
                d = 0.01;
            }
            else if (blockClass == BlockClass::X1_Y2_Z2 || blockClass == BlockClass::X1_Y2_Z2_CHAMFER || blockClass == BlockClass::X1_Y3_Z2)
            {
                d = 0.01;
            }
            correctionApproach45 = Cartesian(sin(rpy.y) * d, -cos(rpy.y) * d, 0);
            correctionLand45 = Cartesian(0, 0, d);
            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).z / 2 + 0.0015;
        }
        else if (configuration == BlockConfiguration::SIDE)
        {
            if (blockClass == BlockClass::X1_Y2_Z1 || blockClass == BlockClass::X1_Y2_Z2 || blockClass == BlockClass::X1_Y2_Z2_CHAMFER || blockClass == BlockClass::X1_Y2_Z2_TWINFILLET)
            {
                approachRPY = approachRPY + RPY(0, 0, M_PI);
            }

            landRPY = landRPY + RPY(0, 0, -M_PI / 2);
            double d = 0.01;
            correctionApproach45 = Cartesian(0, 0, d);
            correctionLand45 = Cartesian(cos(finalRpy.y) * d, sin(finalRpy.y) * d, 0.002);
            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).y / 2;
        }
        else
        {
            correctionApproach45 = Cartesian(0, 0, 0);
            if (blockClass == BlockClass::X2_Y2_Z2 || blockClass == BlockClass::X2_Y2_Z2_FILLET)
            {
                correctionApproach45 = Cartesian(0, 0, 0.025);
            }
            correctionLand45 = Cartesian(0, 0, 0.002);
            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).y / 2;
        }

        approachPos = position + correctionApproach45;

        apprachRotm90 = Algebra::eul2RotM(approachRPY.toVector());
        theta = M_PI / 4;
        approachRotm = apprachRotm90 * Algebra::rotY(theta);

        landPosNoCorrection = Cartesian(xLandPose, yLandPose, landZPos);
        landPos = landPosNoCorrection + correctionLand45;

        landRotm = Algebra::eul2RotM(landRPY.toVector()) * Algebra::rotY(-theta);
    }

    landPos = landPos + Cartesian(0, 0, zOffset);
    finalPos = landPosNoCorrection;
    fRpy = finalRpy;
}