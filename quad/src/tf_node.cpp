#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#define _USE_MATH_DEFINES
#include <cmath>


class tfNode
{
private:
    ros::NodeHandle nh;



    ros::Publisher pub_pose;
    ros::Subscriber sub_pos;
    ros::Subscriber sub_pose_correction;
    tf::TransformBroadcaster pub_transform;

public:
    tfNode()
        : nh("~")
    {
        sub_pos = nh.subscribe("/mavros/local_position/local", 1, &tfNode::poseCallback, this);
    }

    void poseCallback(const geometry_msgs::PoseStamped &msg)
        {
            publishOdometry(msg);
        }

   
    void publishOdometry(geometry_msgs::PoseStamped pose)
        {
            ros::Time now = ros::Time::now();
            //publish "robot" transform
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(pose.pose.position.x,
                                            pose.pose.position.y,
                                            0));//pose.pose.position.z));
            transform.setRotation(tf::Quaternion(pose.pose.orientation.x,
                                                 pose.pose.orientation.y,
                                                 pose.pose.orientation.z,
                                                 pose.pose.orientation.w));
            pub_transform.sendTransform(tf::StampedTransform(transform,now, "map", "MAV"));
        }

    void run()
    {
        ros::spin();
    }
};

int main (int argc, char** argv)
{
    ros::init(argc, argv, "tf_node");
    tfNode tf_node;
    tf_node.run();
}

