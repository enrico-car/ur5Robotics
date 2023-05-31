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

double Block::getDistanceFromShoulder()
{
    return distanceFromShoulder;
}

double Block::getRadius()
{
    return radius;
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
    // // double s1, s2;
    // Block up configuration
    if ((M_PI / 2 - 0.01 < rpy.r && rpy.r < M_PI / 2 + 0.01))
    {
        // // s1 = abs(dim.x * cos(rpy.y)) + abs(dim.z * sin(rpy.y));
        // // s2 = abs(dim.x * sin(rpy.y)) + abs(dim.z * cos(rpy.y));
        position.z = DESK_HEIGHT + dim.y / 2;
        height = dim.y;
        configuration = BlockConfiguration::UP;
        radius = std::max(dim.y, dim.z) / 2.0;
    }
    // Block side configuration
    else if (M_PI / 2 - 0.01 < rpy.p && rpy.p < M_PI / 2 + 0.01)
    {
        // // s1 = abs(dim.y * sin(rpy.y)) + abs(dim.z * cos(rpy.y));
        // // s2 = abs(dim.y * cos(rpy.y)) + abs(dim.z * sin(rpy.y));
        position.z = DESK_HEIGHT + dim.x / 2;
        height = dim.x;
        configuration = BlockConfiguration::SIDE;
        radius = std::max(dim.y, dim.z) / 2.0;
    }
    else
    {
        // // s1 = abs(dim.y * sin(rpy.y)) + abs(dim.x * cos(rpy.y));
        // // s2 = abs(dim.y * cos(rpy.y)) + abs(dim.x * sin(rpy.y));
        position.z = DESK_HEIGHT + dim.z / 2;
        height = dim.z;
        radius = std::max(dim.y, dim.x) / 2.0;
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

    // // Cartesian s(s1 / 2, s2 / 2, 0);
    // // minPosition = position - s;
    // // maxPosition = position + s;
    // // zPos = DESK_HEIGHT + dim.x / 2;

    top = DESK_HEIGHT + height;
    distanceFromShoulder = sqrt((position.x - 0.5) * (position.x - 0.5) + (position.y - 0.35) * (position.y - 0.35));

    // if (rpy.y < 0)
    // {
    //     rpy.y = 2 * M_PI - rpy.y;
    // }

    // desk position
    if (position.x > 0.5 && position.y <= 0.475)
    {
        quadrant = 1;
        if (blockClass >= 0 && blockClass <= 9 && configuration == BlockConfiguration::DOWN)
        {
            rpy.y += M_PI / 2;
        }
    }
    else if (position.x > 0.5 && position.y > 0.475)
    {
        quadrant = 2;
        if (blockClass >= 0 && blockClass <= 9 && configuration == BlockConfiguration::DOWN)
        {
            rpy.y += M_PI;
        }
    }
    else if (position.x <= 0.5 && position.y > 0.475)
    {
        quadrant = 3;
        if (blockClass >= 0 && blockClass <= 9 && configuration == BlockConfiguration::DOWN)
        {
            rpy.y += M_PI;
        }
    }
    else
    {
        quadrant = 4;
        if (blockClass >= 0 && blockClass <= 9 && configuration == BlockConfiguration::DOWN)
        {
            rpy.y += M_PI;
        }
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
    RPY finalRpy;
    if (!(-0.01 < rpy.p && rpy.p < 0.01))
    {
        double roll = M_PI / 2;
        double yaw = M_PI;

        if (M_PI / 2 - 0.01 < rpy.p && rpy.p < M_PI / 2 + 0.01)
        {
            if (blockClass > 0 && blockClass < 5)
            {
                roll = 0;
                if (quadrant == 1)
                {
                    yaw = M_PI;
                }
                else if (quadrant == 4)
                {
                    yaw = 0.0;
                }
                else
                {
                    yaw = M_PI / 2;
                }
            }
        }
        else
        {
            if (quadrant == 1 || quadrant == 2)
            {
                yaw = 3 * M_PI / 2;
            }
            else
            {
                yaw = M_PI / 2;
            }
        }
        finalRpy = RPY(roll, 0.0, yaw);
    }
    else
    {
        finalRpy = RPY(0.0, 0.0, M_PI / 2);
    }

    return finalRpy;
}

void Block::computeApproachAndLandPose(double xLandPose, double yLandPose, RPY finalRpy, double zOffset, int yApproachAngle)
{
    RPY approachRPY(M_PI, 0.0, rpy.y + M_PI / 2);
    RPY landRPY(M_PI, 0.0, finalRpy.y + M_PI / 2);

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
            approachPos.z = top - 0.03;
            if (blockClass == BlockClass::X1_Y1_Z2)
            {
                approachPos.z = top - 0.015;
            }
        }

        if (configuration == BlockConfiguration::SIDE && blockClass > 0 && blockClass < 5)
        {
            approachRPY.y += -M_PI / 2;
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
            if (quadrant == 1 || quadrant == 2)
            {
                landRPY.y += -M_PI;
            }

            double d = 0.00;
            if (blockClass == BlockClass::X2_Y2_Z2)
            {
                d = 0.012;
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
            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).z / 2 + 0.004;
        }
        else if (configuration == BlockConfiguration::SIDE)
        {
            double d = 0.01;
            correctionApproach45 = Cartesian(0, 0, d);
            correctionLand45 = Cartesian(cos(finalRpy.y) * d, sin(finalRpy.y) * d, 0.004);

            if (blockClass > 0 && blockClass < 5)
            {
                approachRPY.y += M_PI / 2;
                landRPY.y += M_PI / 2;
                correctionLand45 = Cartesian(-cos(finalRpy.y) * d, sin(finalRpy.y) * d, 0.004);
            }
            else if (blockClass != 3 && blockClass != 4 && blockClass != 6 && blockClass != 10)
            {
                if (rpy.y < M_PI / 2 || rpy.y > 3 / 2 * M_PI)
                {
                    theta = -M_PI / 4;
                }
                landRPY.y += -M_PI / 2;
                correctionLand45 = Cartesian(Algebra::sgn(theta) * cos(finalRpy.y) * d, Algebra::sgn(theta) * sin(finalRpy.y) * d, 0.004);
            }
            else
            {
                landRPY.y += -M_PI / 2;
            }

            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).y / 2;
        }
        else
        {
            correctionApproach45 = Cartesian(0.0, 0.0, 0.0);
            double d = 0;
            if (blockClass == BlockClass::X2_Y2_Z2 || blockClass == BlockClass::X2_Y2_Z2_FILLET)
            {
                d = 0.025;
                correctionApproach45 = Cartesian(0, 0, d);
            }
            correctionLand45 = Cartesian(-d * sin(finalRpy.y), cos(finalRpy.y) * d, 0.004);
            landZPos = DESK_HEIGHT + BlockDimension.at(blockClass).y / 2;
        }

        approachPos = position + correctionApproach45;

        apprachRotm90 = Algebra::eul2RotM(approachRPY.toVector());
        approachRotm = apprachRotm90 * Algebra::rotY(theta);

        landPosNoCorrection = Cartesian(xLandPose, yLandPose, landZPos);
        landPos = landPosNoCorrection + correctionLand45;

        landRotm = Algebra::eul2RotM(landRPY.toVector()) * Algebra::rotY(-theta);
    }

    landPos = landPos + Cartesian(0.0, 0.0, zOffset);
    finalPos = landPosNoCorrection;
    fRpy = finalRpy;
}

void Block::print()
{
    std::cout << "x " << position.x << " y " << position.y << " Configuraiton " << configuration
              << " roll " << rpy.r << " pitch " << rpy.p << " yaw " << rpy.y << std::endl;
}