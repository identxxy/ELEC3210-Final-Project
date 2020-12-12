#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
using namespace std;
// #include<std_msgs/Float64.h>
// #include<nav_msgs/MapMetaData.h>
// #include<nav_msgs/OccupancyGrid.h>

void pos_callback(const geometry_msgs::PoseStampedConstPtr &pos_msg)
{

    double pos_x = pos_msg->pose.position.x;
    double pos_y = pos_msg->pose.position.y;
    cout << pos_x << " " << pos_y;
    if (pos_x > 5.5)
    {
        cout << "D" << endl;
    }
    else
    {
        if (pos_y > -3)
        {
            cout << "A" << endl;
        }
        else if (pos_y < -3 && pos_y > -6.5)
        {
            cout << "B" << endl;
        }
        else
        {
            cout << "C" << endl;
        }
    }
}

// void map_callback(const nav_msgs::OccupancyGridConstPtr &map_msg)
// {

// }

// void entropy_callback(const std_msgs::Float64ConstPtr &entropy_msg)
// {

// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "location");

    ros::NodeHandle n;

    ros::Subscriber mapmeta_sub = n.subscribe("/slam_out_pose", 1000, pos_callback);
    // ros::Subscriber map_sub = n.subscribe("/gmapping/map", 1000, map_callback);
    // ros::Subscriber entropy_sub = n.subscribe("/gmapping/~entropy", 1000, entropy_callback);
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);

    // ros::Rate rate(10.0);
    // while (n.ok())
    // {
    //     geometry_msgs::TransformStamped transformStamped;
    //     try
    //     {
    //         transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0));
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         ROS_WARN("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //         continue;
    //     }

    //     cout << transformStamped.transform.translation.x << "," << transformStamped.transform.translation.y << endl;
    //     if (transformStamped.transform.translation.y > 5)
    //     {
    //         ROS_INFO("D");
    //     }
    //     else
    //     {
    //         if (transformStamped.transform.translation.x > -3)
    //         {
    //             ROS_INFO("A");
    //         }
    //         else if (transformStamped.transform.translation.x > -6 && transformStamped.transform.translation.x < -3)
    //         {
    //             ROS_INFO("B");
    //         }
    //         else
    //         {
    //             ROS_INFO("C");
    //         }
    //     }

    //     rate.sleep();
    //}
    ros::spin();
    return 0;
}
