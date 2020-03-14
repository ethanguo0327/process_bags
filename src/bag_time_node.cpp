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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
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

    bag1.open("/media/ethan/gyq7/test_bag/ouster/0313/lidar_odom/2020-03-13-16-54-01.bag", rosbag::bagmode::Read);
    cout<<"bag opened"<<endl;
    bag_result.open("/media/ethan/gyq7/test_bag/ouster/0313/lidar_odom/2020-03-13-16-54-01_processed.bag", rosbag::bagmode::Write);

    //d435
    std::vector<std::string> topics;
    topics.push_back(std::string("/odom"));int odomcnt=0;
    topics.push_back(std::string("/tf"));int tfcnt=0;
    topics.push_back(std::string("/os1_cloud_node/points"));int ptscnt=0;
    cout<<"reading and writing...."<<endl;

    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    int i=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        if(m.getTopic()=="/os1_cloud_node/points"){
            sensor_msgs::PointCloud2ConstPtr ptsptr = m.instantiate<sensor_msgs::PointCloud2>();
            ros::Time bag_time=m.getTime();
            if (ptsptr != NULL) {
                ptscnt++;
                sensor_msgs::PointCloud2 pcld = *ptsptr;
                bag_result.write("/os1_cloud_node/points",bag_time,pcld);
            }
        }
        if(m.getTopic()=="/odom"){
            nav_msgs::OdometryConstPtr odomptr = m.instantiate<nav_msgs::Odometry>();
            ros::Time bag_time=m.getTime();
            if (odomptr != NULL) {
                odomcnt++;
                nav_msgs::Odometry odom = *odomptr;
                double new_timestamp = odom.header.stamp.toSec() - 62.68;
                odom.header.stamp.sec=new_timestamp;
                bag_result.write("/odom",bag_time,odom);

            }
        }
        if(m.getTopic()=="/tf"){
            tf2_msgs::TFMessageConstPtr tfptr = m.instantiate<tf2_msgs::TFMessage>();
            if (tfptr != NULL) {
                if (tfptr->transforms[0].header.frame_id == "base_odom") {
                    tfcnt++;
                    ros::Time bag_time = m.getTime();
                    tf2_msgs::TFMessage tf2msg = *tfptr;
                    double new_timestamp = tfptr->transforms[0].header.stamp.toSec() - 62.68;
                    tf2msg.transforms[0].header.stamp.sec = new_timestamp;
                    geometry_msgs::TransformStamped transformStamped;
                    transformStamped.header.stamp.sec = new_timestamp;
                    transformStamped.header.frame_id = "base_link";
                    transformStamped.header.seq = tfcnt;
                    transformStamped.child_frame_id = "os1_lidar";
                    transformStamped.transform.translation.x = 0;
                    transformStamped.transform.translation.y = 0;
                    transformStamped.transform.translation.z = 0;
                    transformStamped.transform.rotation.x = 0;
                    transformStamped.transform.rotation.y = 0;
                    transformStamped.transform.rotation.z = 0;
                    transformStamped.transform.rotation.w = 1;
                    tf2msg.transforms.push_back(transformStamped);
                    bag_result.write("/tf", bag_time, tf2msg);

                }
            }
        }
    }

    bag1.close();
    bag_result.close();
    cout<<"ptscnt "<<odomcnt<<endl;
    cout<<"odomcnt "<<odomcnt<<endl;
    cout<<"tfcnt "<<tfcnt<<endl;
    cout<<"==========depth writing done!============="<<endl;

    return 0;
}