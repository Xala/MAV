#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#define _USE_MATH_DEFINES
#include <cmath>


class anglesNode
{
private:
    ros::NodeHandle nh;



    ros::Publisher pubLaser;
    ros::Subscriber subLaser;

public:
    anglesNode()
        : nh("~")
    {
        subLaser = nh.subscribe("/scan", 1, &anglesNode::laserCallback, this);
        pubLaser = nh.advertise<sensor_msgs::LaserScan>("/fixScan",1);
    }

    void laserCallback(const sensor_msgs::LaserScan &msg)
        {
            fixAngle(msg);
        }


    void fixAngle(sensor_msgs::LaserScan scan)
        {
            scan.angle_max = -scan.angle_min;
            pubLaser.publish(scan);
        }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "angles");
    anglesNode angles_node;
    angles_node.run();
}

