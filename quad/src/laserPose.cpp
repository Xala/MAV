#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>


class laserPose
{
private:
    ros::NodeHandle nh;

    ros::Subscriber subImu;
    ros::Subscriber subPose2D;
    ros::Subscriber subSonarRange;

    ros::Publisher pubPose;


    tf::Quaternion tfQuat;
    sensor_msgs::Range sonarRange;
    geometry_msgs::Pose2D pose2D;

public:
    laserPose()
        : nh("~")
    {
        subImu = nh.subscribe("/mavros/imu/data", 1, &laserPose::imuCallback, this);
        subPose2D = nh.subscribe("/pose2D",1,  &laserPose::pose2DCallback, this);
        subSonarRange = nh.subscribe("/mavros/px4flow/ground_distance",1, &laserPose::sonarRangeCallback, this);

        pubPose = nh.advertise<geometry_msgs::PoseStamped>("/pose",1);
    }

    void imuCallback(const sensor_msgs::Imu &msg)
    {
        geometry_msgs::Quaternion quat= msg.orientation;

        tf::quaternionMsgToTF(quat,tfQuat);

    }

    void pose2DCallback(const geometry_msgs::Pose2D &msg)
    {
        poseConstruction(msg);
    }

    void sonarRangeCallback(const sensor_msgs::Range &msg)
    {
        sonarRange = msg;
    }

    void poseConstruction(geometry_msgs::Pose2D pose2D)
    {
        geometry_msgs::PoseStamped fusedPose;
    }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "laserPose");
    laserPose laserPose_node;
    laserPose_node.run();
}
