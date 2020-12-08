#include <ros/ros.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;

ros::Publisher track_pub;
ros::Subscriber track_sub;
ros::Subscriber sub_run;
bool run;
double remA = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("image rec");
    // geometry_msgs::Twist tw;
    // tw.linear.x = 1;
    // track_pub.publish(tw);
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    //ROS_INFO_STREAM("New Image from " << frame_id);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat image_src, image_hsv, image_binary;
    image_src = cv_ptr->image;
    cvtColor(image_src, image_hsv, COLOR_BGR2HSV);
    inRange(image_hsv, Scalar(20, 95, 95), Scalar(30, 255, 255), image_binary);
    // imshow("original", image_src);
    // imshow("hsv", image_hsv);
    // imshow("binary", image_binary);

    //cvtColor(cv_ptr->image, gray, COLOR_BGR2GRAY);
    //medianBlur(gray, gray, 5);
    vector<Vec3f> circles;
    HoughCircles(image_binary, circles, HOUGH_GRADIENT, 1,
                 image_binary.rows / 16, // change this value to detect circles with different distances to each other
                 85, 15, 1, 0            // change the last two parameters
                                         // (min_radius & max_radius) to detect larger circles
    );
    cout << circles.size() << endl;

    Point image_center = Point(image_binary.rows / 2, image_binary.cols / 2);
    //cout << image_center;

    circle(image_binary, image_center, 1, Scalar(0, 50, 100), 3, LINE_AA);
    for (size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point c_center = Point(c[0], c[1]);
        // circle center
        circle(image_binary, c_center, 1, Scalar(0, 100, 100), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(image_binary, c_center, radius, Scalar(255, 0, 255), 3, LINE_AA);
        // cout << c_center << radius << endl;
    }

    //imshow("detected circles", image_binary);
    waitKey(3);
    double L_velocity = 0;
    double A_velocity = 0;

    if (circles.size())
    {

        Vec3i ball = circles[0];
        cout << ball[2] << endl;
        if (ball[2] < 100)
        {
            L_velocity = 1.75;
        }
        else
        {
            L_velocity = 0;
        }
        if (ball[0] < image_binary.rows / 2)
        {
            A_velocity = -0.5;
        }
        else if (ball[0] > image_binary.rows / 2)
        {
            A_velocity = 0.5;
        }
        else
        {
            A_velocity = 0;
        }
        remA = A_velocity;
    }
    else
    {
        A_velocity = remA;
    }
    //cout<<"send cmd"<<endl;
    //cout << L_velocity << A_velocity << endl;
    geometry_msgs::Twist tw;
    if (run)
    {
        tw.linear.x = L_velocity;
        tw.angular.z = A_velocity;
        track_pub.publish(tw);
    }
}

void run_tracker_callback(const std_msgs::Bool &msg)
{
    run = msg.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track");
    ros::NodeHandle n;
    sub_run = n.subscribe("run_tracking", 1000, run_tracker_callback);
    track_sub = n.subscribe("/vrep/image", 1000, imageCallback);

    track_pub = n.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1000);

    ros::spin();
    return 0;
}