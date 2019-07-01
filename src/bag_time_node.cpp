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
#define foreach BOOST_FOREACH

using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "bag_time_node");
    ros::NodeHandle nh_;

    rosbag::Bag bag;
    rosbag::Bag bag2;
    bag.open("/home/ethan/bag1/2019-06-29-13-24-51.bag", rosbag::bagmode::Read);
    bag2.open("/home/ethan/bag1/2019-06-29-13-24-51_2.bag", rosbag::bagmode::Write);
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/color/camera_info"));
    topics.push_back(std::string("/camera/color/image_raw"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    cout<<"reading and writing...."<<endl;


    int i=0;int j=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr s = m.instantiate<sensor_msgs::Image>();
        if (s != NULL)
        {
            cout<<"image: "<<i++<<endl;
            ros::Time enhanced_stamp(82,0);
            enhanced_stamp.sec+=s->header.stamp.sec;
            enhanced_stamp.nsec=s->header.stamp.nsec;
            sensor_msgs::Image image=*s;
            image.header.stamp=enhanced_stamp;
            bag2.write("/camera/color/image_raw",enhanced_stamp,image);
        }

        sensor_msgs::CameraInfo::ConstPtr a= m.instantiate<sensor_msgs::CameraInfo>();
        if (a != NULL)
        {
            cout<<"camera info: "<<j++<<endl;
            ros::Time enhanced_stamp(82,0);
            enhanced_stamp.sec+=a->header.stamp.sec;
            enhanced_stamp.nsec=a->header.stamp.nsec;
            sensor_msgs::CameraInfo cameraInfo=*a;
            cameraInfo.header.stamp=enhanced_stamp;
            bag2.write("/camera/color/camera_info",enhanced_stamp,cameraInfo);
        }

    }

    bag.close();
    bag2.close();
    cout<<"writing done!"<<endl;

    return 0;
}