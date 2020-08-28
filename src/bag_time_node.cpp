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
#include <sensor_msgs/LaserScan.h>
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

    bag1.open("/media/ethan/TOSHIBA/issues/mapping_v5/lost/PGJ-LOST/2020-8-20-20-12-24_0.bag_filtered.bag", rosbag::bagmode::Read);
    cout<<"bag opened"<<endl;
    bag_result.open("/media/ethan/TOSHIBA/issues/mapping_v5/lost/PGJ-LOST/2020-8-20-20-12-24_0.filtered_processed.bag", rosbag::bagmode::Write);

    //d435
    std::vector<std::string> topics;
    topics.push_back(std::string("/scan"));int scancnt=0;
    topics.push_back(std::string("/odom"));int odomcnt=0;
    topics.push_back(std::string("/tf"));int tfcnt=0;

  cout<<"reading and writing...."<<endl;

    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    int i=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        if(m.getTopic()=="/scan"){
            sensor_msgs::LaserScanConstPtr scanptr = m.instantiate<sensor_msgs::LaserScan>();
            ros::Time bag_time = m.getTime();
            if (bag_time.toSec() > 1597925606.6 && bag_time.toSec() < 1597925608.6){
              std::cout<<"ignore 2"<<std::endl;
            }else if(bag_time.toSec() > 1597925587.6 && bag_time.toSec() < 1597925589.6){
              std::cout<<"ignore 1"<<std::endl;
            }else{
              if (scanptr != NULL) {
                scancnt++;
                sensor_msgs::LaserScan scan = *scanptr;
//                double new_timestamp = odom.header.stamp.toSec();
//                odom.header.stamp.sec=new_timestamp;
                bag_result.write("/scan",bag_time,scan);
              }
            }
        }
        if(m.getTopic()=="/odom") {
          nav_msgs::OdometryConstPtr odomptr = m.instantiate<nav_msgs::Odometry>();
          ros::Time bag_time = m.getTime();
          if (odomptr != NULL) {
            odomcnt++;
            nav_msgs::Odometry odom = *odomptr;
            bag_result.write("/odom", bag_time, odom);
          }
        }
          if (m.getTopic() == "/tf") {
            tf2_msgs::TFMessageConstPtr tfptr = m.instantiate<tf2_msgs::TFMessage>();
            if (tfptr != NULL) {
              if (tfptr->transforms[0].header.frame_id == "base_odom") {
                tfcnt++;
                ros::Time bag_time = m.getTime();
                tf2_msgs::TFMessage tf2msg = *tfptr;
                double new_timestamp = tfptr->transforms[0].header.stamp.toSec() - 62.68;
                tf2msg.transforms[0].header.stamp.sec = new_timestamp;
                bag_result.write("/tf", bag_time, tf2msg);

              }
            }
          }
    }
    bag1.close();
    bag_result.close();
    cout<<"scancnt "<<scancnt<<endl;
    cout<<"==========depth writing done!============="<<endl;

    return 0;
}