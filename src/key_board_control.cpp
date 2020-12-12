#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
#include "signal.h"
#include "termios.h"

#define RIGHT 0x43
#define LEFT 0x44
#define UP 0x41
#define DOWN 0x42
#define QUIT 0x71
#define TRACK 0x74      //T
#define STOP_TRACK 0x73 //S
double linear_v = 0;
double angular_v = 0;

int main(int argc, char **argv)
{
    int kfd = 0; //used for capturing keyboard input
    struct termios cooked, raw;
    std_msgs::Bool run;
    run.data = false;

    ros::init(argc, argv, "key_board_control");

    ros::NodeHandle n;

    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);
    ros::Publisher pub2 = n.advertise<std_msgs::Bool>("run_tracking", 1000);

    ros::Rate loop_rate(10);

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    while (ros::ok())
    {
        // keyboard
        char in; //get input from key_board
        if (read(kfd, &in, 1) < 0)
        {
            ROS_ERROR("cannot read input");
            exit(-1);
        }

        geometry_msgs::Twist tw;

        switch (in)
        {
        case UP:
            ROS_INFO("up");
            linear_v = linear_v + 0.5;
            tw.linear.x = linear_v;
            break;
        case DOWN:
            ROS_INFO("down");
            linear_v = linear_v - 0.5;
            tw.linear.x = linear_v;
            break;
        case LEFT:
            ROS_INFO("left");
            angular_v = angular_v + 0.5;
            tw.angular.z = angular_v;
            break;
        case RIGHT:
            ROS_INFO("right");
            angular_v = angular_v - 0.5;
            tw.angular.z = angular_v;
            break;
        case TRACK:
            ROS_INFO("start_track");
            run.data = true;
            break;
        case STOP_TRACK:
            ROS_INFO("stop_track");
            run.data = false;
            angular_v = 0.0;
            linear_v = 0.0;
            break;
        case QUIT:
            ROS_DEBUG("quit");
            return 0;
        }

        //tw.linear.x = 1;
        //tw.angular.x = 0;
        if (run.data == false)
            cmd_pub.publish(tw);
        pub2.publish(run);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

    //   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    //   ros::Rate loop_rate(10);

    //   int count = 0;
    //   while (ros::ok())
    //   {

    //     std_msgs::String msg;

    //     std::stringstream ss;
    //     ss << "hello world " << count;
    //     msg.data = ss.str();

    //     ROS_INFO("%s", msg.data.c_str());

    //     chatter_pub.publish(msg);

    //     ros::spinOnce();

    //     loop_rate.sleep();
    //     ++count;
    //   }

    return 0;
}