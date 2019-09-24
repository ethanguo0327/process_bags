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
    std::ofstream f2;
    f2.open("/home/ethan/Documents/lifelong/bag_factory/vins-t265-1-timestamp.csv",std::ios_base::out|std::ios_base::trunc);
    bag1.open("/home/ethan/Documents/lifelong/bag_factory/vins-t265-1-mono-format.bag", rosbag::bagmode::Read);
    cout<<"vins-t265-1-mono-format.bag opened"<<endl;

    //t265
    std::vector<std::string> topics;
    topics.push_back(std::string("/device_0/sensor_0/Fisheye_1/image/data"));int d435depthcnt=0;
    cout<<"reading and writing...."<<endl;
    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr sdimage = m.instantiate<sensor_msgs::Image>();
        ros::Time bag_time=m.getTime();
        if (sdimage != NULL) {
            d435depthcnt++;
            f2<<sdimage->header.stamp<<std::endl;
        }
    }

    bag1.close();
    cout<<"timestamp cnt "<<d435depthcnt<<endl;
    cout<<"==========timestamp writing done!============="<<endl;

    return 0;
}