#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>


class laserScanAngleCompensation
{
private:
    ros::NodeHandle nh;

    ros::Subscriber subImu;
    ros::Subscriber subLaser;

    ros::Publisher pubLaser;


    tf::Quaternion tfQuat;

public:
    laserScanAngleCompensation()
        : nh("~")
    {
        subImu = nh.subscribe("/mavros/imu/data", 1, &laserScanAngleCompensation::imuCallback, this);
        subLaser = nh.subscribe("/fixScan",1,  &laserScanAngleCompensation::laserCallback, this);
        pubLaser = nh.advertise<sensor_msgs::LaserScan>("/scanAngleCompensated",1);
    }

    ~laserScanAngleCompensation()
    {
    }

    void imuCallback(const sensor_msgs::Imu &msg)
    {
        geometry_msgs::Quaternion quat= msg.orientation;

        tf::quaternionMsgToTF(quat,tfQuat);
    }

    void laserCallback(const sensor_msgs::LaserScan &msg)
    {
        attitudeCompensation(msg);
    }

    void attitudeCompensation(sensor_msgs::LaserScan scan)
    {
        tf::Matrix3x3 m(tfQuat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        //pitch = -pitch;
        //std::cout << roll*180/3.1415 << std::endl;
        //std::cout << pitch*180/3.1415 << std::endl;
        //std::cout << yaw*180/3.1415 << std::endl;

        //std::cout << scan.angle_increment << std::endl;
        double n = scan.angle_max*2/scan.angle_increment;
        sensor_msgs::LaserScan compensatedScan = scan;
        for (int i = 0; i <= n; i++)
        {
            double currLaserYawAngle = scan.angle_min + n*scan.angle_increment;
            double currRange = scan.ranges[i];
            //Compensates laser range based on IMU angles.
            double rollCompensation = sin(currLaserYawAngle)*(currRange - currRange*cos(roll));
            double pitchCompensation = cos(currLaserYawAngle)*(currRange - currRange*cos(pitch));
            //std::cout << "roll: " << roll << " pitch: " << pitchCompensation << std::endl;
            double newRange = currRange + abs(rollCompensation) + abs(pitchCompensation);
            compensatedScan.ranges[i] = newRange;
        }
        pubLaser.publish(compensatedScan);
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "laserScanAngleCompensation");
    laserScanAngleCompensation laserScanAngleCompensation_node;
    laserScanAngleCompensation_node.run();
}

