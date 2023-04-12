#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include "kinematic.h"
#include "block.h"

int main(int argc, char **argv)
{
    std::cout << BlockDimension.at(4) << std::endl;
    // Vector6 initialJ;
    // initialJ << -0.32, -0.2, -2.8, -1.63, 0, -1.0;
    // Vector3 finalJ;
    // finalJ << 0.0, 0.6, -0.7;
    // Matrix3 m4;
    // m4 << 1, 0, 0,
    //     0, -1, 0,
    //     0, 0, -1;

    // Trajectory res = Kinematic::differentialKinematic(initialJ, finalJ, m4);

    // for (auto x : res.positions)
    // {
    //     for (auto y : x)
    //     {
    //         std::cout << y << " ";
    //     }
    //     std::cout << std::endl;
    // }

    return 0;
}