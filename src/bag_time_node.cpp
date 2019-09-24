//
// Created by ethan on 19-7-1.
//

//
// Created by ethan on 19-7-1.
//

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
//#include <rslidar_msgs/rslidarScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Accel.h>
#define foreach BOOST_FOREACH

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "bag_time_node");
    ros::NodeHandle nh_;
    rosbag::Bag bag1;//t265
    rosbag::Bag bag_result;

    bag1.open("/home/ethan/Documents/lifelong/bag_factory/vins-t265-1-mono-format.bag", rosbag::bagmode::Read);
    cout<<"vins-t265-1-mono-format.bag opened"<<endl;
    bag_result.open("/home/ethan/Documents/lifelong/bag_factory/vins-t265-1-mono-format-cameraInfo.bag", rosbag::bagmode::Write);

    //t265
    std::vector<std::string> topics;
    topics.push_back(std::string("/device_0/sensor_0/Fisheye_1/info/camera_info"));int d435depthcnt=0;
    topics.push_back(std::string("/device_0/sensor_0/Fisheye_1/image/data"));
    cout<<"reading and writing...."<<endl;
    sensor_msgs::CameraInfo cameraInfo;
    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::CameraInfoConstPtr scameraInfo = m.instantiate<sensor_msgs::CameraInfo>();
        if(scameraInfo != NULL)
        {
            cameraInfo = *scameraInfo;
        }

        sensor_msgs::Image::ConstPtr sdimage = m.instantiate<sensor_msgs::Image>();
        ros::Time bag_time=m.getTime();
        if (sdimage != NULL) {
            d435depthcnt++;
            cameraInfo.header.stamp=sdimage->header.stamp;
            bag_result.write("/device_0/sensor_0/Fisheye_1/info/camera_info",bag_time,cameraInfo);
            bag_result.write("/device_0/sensor_0/Fisheye_2/info/camera_info",bag_time,cameraInfo);

        }
    }

    bag1.close();
    bag_result.close();
    cout<<"cameraInfo cnt "<<d435depthcnt<<endl;
    cout<<"==========cameraInfo writing done!============="<<endl;

    return 0;
}