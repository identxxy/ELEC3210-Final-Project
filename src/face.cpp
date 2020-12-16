#include "opencv2/objdetect.hpp"
#include "opencv2/face.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>
#include <cmath>
using namespace std;
using namespace cv;
using namespace cv::face;
Rect detectAndResize(Mat &frame, Mat &resizedROI, Size resize_scale);
CascadeClassifier face_cascade;
Ptr<EigenFaceRecognizer> model;

ros::Publisher arrow_pub;
ros::Publisher text_pub;
ros::Subscriber img_sub;
ros::Subscriber laser_sub;
sensor_msgs::LaserScan laser_msg;
tf2_ros::Buffer tf_buffer;

void imageCallback(const sensor_msgs::ImageConstPtr &msg);
void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
void pubMarker(geometry_msgs::Vector3 p, tf2::Quaternion q, double l, string tx);
void clearMarker();

Rect detectAndResize(Mat &frame, Mat &resizedROI, Size resize_scale = Size(50, 50))
{
    static int num_saved;
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    // equalizeHist(frame_gray, frame_gray);
    //-- Detect faces
    std::vector<Rect> faces;
    face_cascade.detectMultiScale(frame, faces, 1.1, 4, 0, cv::Size(50, 50));
    int max_area = 0;
    int max_i = -1;
    for (size_t i = 0; i < faces.size(); i++)
    {
        Point start(faces[i].x, faces[i].y);
        Point end(faces[i].x + faces[i].width, faces[i].y + faces[i].height);
        rectangle(frame, start, end, Scalar(255, 0, 255), 4);
        if (faces[i].width * faces[i].height > max_area)
        {
            max_area = faces[i].width * faces[i].height;
            max_i = i;
        }
    }
    if (faces.size() >= 1)
    {
        Mat faceROI = frame_gray(faces[max_i]);
        resize(faceROI, resizedROI, resize_scale);
        // imshow("ROI", resizedROI);
        // if (waitKey(1) == 's')
        // {
        //     ROS_INFO_STREAM("saving img " << ++num_saved);
        //     imwrite(format("%d.png", num_saved), resizedROI);
        // }
        return faces[max_i];
    }
    return Rect();
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
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

    Mat frame, resizedROI;
    flip(cv_ptr->image, frame, 1);
    Rect rect = detectAndResize(frame, resizedROI);
    if (rect.width > 0)
    {
        Point center(rect.x + rect.width / 2 + 1, rect.y + rect.height / 2 + 1);
        double x_frame_angle = center.x * M_PI / 4.0f / 512.0f;
        double y_frame_angle = center.y * M_PI / 4.0f / 512.0f;
        int range_i = (M_PI / 4 + x_frame_angle) / laser_msg.angle_increment;
        string recog_name;
        switch (model->predict(resizedROI))
        {
        case 0:
            recog_name = "Obama";
            break;
        case 1:
            recog_name = "Avril";
            break;
        case 2:
            recog_name = "Cartoon";
            break;
        case 3:
            recog_name = "Zhang";
            break;
        case 4:
            recog_name = "Elf";
            break;
        default:
            break;
        }
        ROS_INFO_STREAM("Predict result : ==> " << recog_name);
        putText(frame, recog_name, Point(rect.x, rect.y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,255,0), 1);
        // if received laser msg
        if (range_i > 0)
        {
            double dist = laser_msg.ranges[range_i];
            ROS_INFO_STREAM("distance : ==> " << dist);
            double yaw = x_frame_angle - M_PI / 8;
            double pitch = y_frame_angle - M_PI / 8;
            tf2::Quaternion q_n, q_local, q;
            q_n.setRPY(M_PI * 100 / 180, 0, 0);
            q_local.setRPY(0, pitch, -yaw);
            double length = dist / cos(pitch);

            while (!tf_buffer.canTransform("map", "camera_link", ros::Time(0)))
                ;
            geometry_msgs::TransformStamped tf_msg = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0));
            tf2::Quaternion q_global;
            tf2::convert(tf_msg.transform.rotation, q_global);

            q = q_global * q_n * q_local;
            q_n.setRPY(-M_PI * 100 / 180, 0, 0);
            q = q * q_n;
            q.normalize();
            pubMarker(tf_msg.transform.translation, q, length, recog_name);
        }
    }
    else clearMarker();
    //-- Show what you got
    imshow("Capture - Face detection", frame);
    waitKey(1);
}

void laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
    // original angle_increment is negative, ranges is from left to right;
    laser_msg.angle_increment = -msg->angle_increment;
    laser_msg.ranges = msg->ranges;
}

void clearMarker()
{
    visualization_msgs::Marker clr;
    clr.id = 0;
    clr.action = visualization_msgs::Marker::DELETE;
    text_pub.publish(clr);
    arrow_pub.publish(clr);
}

void pubMarker(geometry_msgs::Vector3 p, tf2::Quaternion q, double l, string tx)
{
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.header.stamp = ros::Time();
    arrow.id = 0;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = p.x;
    arrow.pose.position.y = p.y;
    arrow.pose.position.z = p.z;
    arrow.color.a = 1.0; // Don't forget to set the alpha!
    arrow.color.g = 1.0;
    visualization_msgs::Marker text(arrow);
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose.orientation.x = q.x();
    arrow.pose.orientation.y = q.y();
    arrow.pose.orientation.z = q.z();
    arrow.pose.orientation.w = q.w();
    arrow.scale.x = l;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;

    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.color.r = 1.0;
    text.text = tx;
    text.scale.x = 1.0;
    text.scale.y = 1.0;
    text.scale.z = 1.0;

    arrow_pub.publish(arrow);
    text_pub.publish(text);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face");
    ros::NodeHandle n;
    img_sub = n.subscribe("/vrep/image", 1000, imageCallback);
    laser_sub = n.subscribe("/vrep/scan", 1000, laserCallback);
    arrow_pub = n.advertise<visualization_msgs::Marker>("visualization_arrow", 50);
    text_pub = n.advertise<visualization_msgs::Marker>("visualization_text", 50);

    //-- 1. Load the cascades
    String face_cascade_name;
    n.getParam("face_detection_dataset", face_cascade_name);
    if (!face_cascade.load(face_cascade_name))
    {
        cerr << "Error loading face cascade" << endl;
        return -1;
    };

    //-- 2. Load the eigenfaces
    String face_recog_name[5];
    vector<Mat> images;
    vector<int> labels;
    int training_num = 0;
    if (!n.getParam("face_recognition_dataset_num", training_num))
    {
        ROS_ERROR("Error loading face recognition dataset num");
        return -1;
    }
    ROS_INFO_STREAM("face_recognition_dataset_num is " << training_num);
    for (int i = 0; i < 5; ++i)
    {
        if (!n.getParam("face_recognition_dataset" + to_string(i + 1), face_recog_name[i]))
            ROS_ERROR_STREAM("Error loading face recognition dataset" << i);
        ROS_INFO_STREAM("Reading dir " << face_recog_name[i] << " with label " << i);
        for (int k = 0; k < training_num; ++k)
        {
            string img_name = face_recog_name[i] + "/" + to_string(k + 1) + ".png";
            images.push_back(imread(img_name, 0));
            labels.push_back(i);
        }
    }
    model = EigenFaceRecognizer::create();
    ROS_INFO("Start model training");
    ROS_INFO_STREAM("model dim " << images[0].size);
    model->train(images, labels);
    ROS_INFO("Finish model training");

    //-- 3. Create TF listener
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::spin();
}