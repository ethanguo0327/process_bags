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

    ros::init(argc, argv, "bag_processor_node");
    ros::NodeHandle nh_;
    rosbag::Bag bag1;//d435 depth
    rosbag::Bag bag_result;

    bag1.open("/home/guo/Documents/毕业论文/traj3.bag", rosbag::bagmode::Read);
    cout<<"bag opened"<<endl;
    bag_result.open("/home/guo/Documents/毕业论文/traj3_processed.bag", rosbag::bagmode::Write);

    //d435
    std::vector<std::string> topics;
    topics.push_back(std::string("/odom"));int posecnt=0;


    cout<<"reading and writing...."<<endl;

    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    int i=0;
    foreach(rosbag::MessageInstance const m, view)
    {
        if(m.getTopic()=="/odom"){
            nav_msgs::OdometryConstPtr poseptr = m.instantiate<nav_msgs::Odometry>();
            ros::Time bag_time = m.getTime();
            if (98937 < poseptr->header.seq && poseptr->header.seq < 99100){
                posecnt++;
                nav_msgs::Odometry pose = *poseptr;
                nav_msgs::Odometry pose_origin = *poseptr;
                pose.pose.pose.position.x += 40 + 0.1;//0.002*(poseptr->header.seq-34899) + 
                // pose.pose.pose.position.y -= 0.2;//*(poseptr->header.seq-34899);
                pose_origin.pose.pose.position.x += 40;
                bag_result.write("map6",bag_time,pose_origin);
                bag_result.write("map5",bag_time,pose);
            }else{
              if (poseptr != NULL) {
                posecnt++;
                nav_msgs::Odometry pose = *poseptr;
                pose.pose.pose.position.x += 40;
                bag_result.write("map6",bag_time,pose);
                bag_result.write("map5",bag_time,pose);
              }
            }
        }
    }
    bag1.close();
    bag_result.close();
    cout<<"posecnt "<< posecnt <<endl;
    cout<<"==========depth writing done!============="<<endl;

    return 0;
}