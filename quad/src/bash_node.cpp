#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <cmath>


class bashNode
{
private:
    ros::NodeHandle nh;

public:
    bashNode()
        : nh("~")
    {

    }

    void run()
    {
        system("~/catkin_ws/src/quad/src/serialBash.sh");
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "bash");
    bashNode bash_node;
    bash_node.run();
}

