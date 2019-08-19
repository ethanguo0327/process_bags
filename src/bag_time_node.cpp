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
    rosbag::Bag bag1;//d435 depth
    rosbag::Bag bag_result;

    bag1.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713_3/3d/record-d400-filtered_depth.bag", rosbag::bagmode::Read);
    cout<<"d400_depth.bag opened"<<endl;
    bag_result.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713_3/3d/record-d400_depth_processed.bag", rosbag::bagmode::Write);

    //d435
    std::vector<std::string> topics;
    topics.push_back(std::string("/device_0/sensor_0/Depth_0/image/data"));int d435depthcnt=0;
    cout<<"reading and writing...."<<endl;

    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    foreach(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::Image::ConstPtr sdimage = m.instantiate<sensor_msgs::Image>();
        ros::Time bag_time=m.getTime();
        if (sdimage != NULL) {
            d435depthcnt++;
            sensor_msgs::Image depthimg = *sdimage;
            for(int i=sdimage->height*5/6*sdimage->step;i<sdimage->height*sdimage->step;i++)
            {
                depthimg.data[i]=0;
            }
            bag_result.write("/device_0/sensor_0/Depth_0/image/data",bag_time,depthimg);

        }
    }

    bag1.close();
    bag_result.close();
    cout<<"d435depthcnt "<<d435depthcnt<<endl;
    cout<<"==========depth writing done!============="<<endl;

    return 0;
}