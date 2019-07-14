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

    rosbag::Bag bag;//robot
    rosbag::Bag bag2;//d435
    rosbag::Bag bag3;//t265
    rosbag::Bag bag_result;
    bag.open("/home/ethan/Documents/Gaussian_src/intel_d435/record/data-collection-config/recorder/build/713bagrobot/2019-7-13-15-40-48_0.bag", rosbag::bagmode::Read);
    bag2.open("/home/ethan/Documents/Gaussian_src/intel_d435/record/data-collection-config/recorder/build/713bagcamera/record-d400_1-filtered.bag", rosbag::bagmode::Read);
    bag3.open("/home/ethan/Documents/Gaussian_src/intel_d435/record/data-collection-config/recorder/build/713bagcamera/record-t265-1-filtered.bag", rosbag::bagmode::Read);
    bag_result.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/result/713result.bag", rosbag::bagmode::Write);
    //robot
    std::vector<std::string> topics;
    topics.push_back(std::string("/v5_current_pose"));int posecnt=0;
    topics.push_back(std::string("/tf"));int tfcnt=0;
    topics.push_back(std::string("/odom"));int odomcnt=0;
    topics.push_back(std::string("/rslidar_packets"));int rslidarcnt=0;
    //d435 1
    std::vector<std::string> topics2;
    //topics2.push_back(std::string("/camera/color/camera_info"));
    topics2.push_back(std::string("/device_0/sensor_1/Color_0/image/data"));int d435imgcnt=0;
    topics2.push_back(std::string("/device_0/sensor_2/Accel_0/imu/data"));int d435acccnt=0;
    //d435 2
    std::vector<std::string> topics3;
    topics3.push_back(std::string("/device_0/sensor_0/Depth_0/image/data"));int d435depthcnt=0;
    topics3.push_back(std::string("/device_0/sensor_2/Gyro_0/imu/data"));int d435gyrcnt=0;
    //t265 1
    std::vector<std::string> topics4;
    topics4.push_back(std::string("/device_0/sensor_0/Fisheye_1/image/data"));int fish1imgcnt=0;
    topics4.push_back(std::string("/device_0/sensor_0/Fisheye_1/info/camera_info")); int fish1cmicnt=0;
    topics4.push_back(std::string("/device_0/sensor_0/Accel_0/imu/data"));int t265acccnt=0;
    //topics4.push_back(std::string("/device_0/sensor_0/Fisheye_1/tf/0"));
    //t265 2
    std::vector<std::string> topics5;
    topics5.push_back(std::string("/device_0/sensor_0/Fisheye_2/image/data"));int fish2imgcnt=0;
    topics5.push_back(std::string("/device_0/sensor_0/Fisheye_2/info/camera_info"));int fish2cmicnt=0;
    topics5.push_back(std::string("/device_0/sensor_0/Gyro_0/imu/data"));int t265gyrcnt=0;
    //topics5.push_back(std::string("/device_0/sensor_0/Fisheye_2/tf/0"));
    topics5.push_back(std::string("/device_0/sensor_0/Pose_0/pose/accel/data"));
    topics5.push_back(std::string("/device_0/sensor_0/Pose_0/pose/twist/data"));
    topics5.push_back(std::string("/device_0/sensor_0/Pose_0/pose/transform/data"));
//    //t265 3
//    std::vector<std::string> topics6;
//    topics6.push_back(std::string("/device_0/sensor_0/Accel_0/tf/0"));
//    //t265 4
//    std::vector<std::string> topics7;
//    topics7.push_back(std::string("/device_0/sensor_0/Gyro_0/tf/0"));
    //t265 5
//    //t265 6
//    std::vector<std::string> topics9;
//    topics9.push_back(std::string("/device_0/sensor_0/Pose_0/tf/0"));

    cout<<"reading and writing...."<<endl;
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    view.addQuery(bag2,rosbag::TopicQuery(topics2));
    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr spose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (spose != NULL)
        {
            posecnt++;
            geometry_msgs::PoseWithCovarianceStamped pose=*spose;
            bag_result.write("/v5_current_pose",pose.header.stamp,pose);
        }
        tf2_msgs::TFMessage::ConstPtr stf = m.instantiate<tf2_msgs::TFMessage>();
        if (stf != NULL)
        {
            tfcnt++;
            tf2_msgs::TFMessage tf=*stf;
            bag_result.write("/tf",tf.transforms[0].header.stamp,tf);
        }
        nav_msgs::Odometry::ConstPtr sodom = m.instantiate<nav_msgs::Odometry>();
        if (sodom != NULL)
        {
            odomcnt++;
            nav_msgs::Odometry odom=*sodom;
            bag_result.write("/odom",odom.header.stamp,odom);
        }
        sensor_msgs::Image::ConstPtr simage = m.instantiate<sensor_msgs::Image>();
        if (simage != NULL)
        {
            d435imgcnt++;
            sensor_msgs::Image image=*simage;
            ros::Time enhanced_stamp(98,25462);
            enhanced_stamp.sec+=simage->header.stamp.sec;
            enhanced_stamp.nsec=simage->header.stamp.nsec;
            image.header.stamp=enhanced_stamp;
            bag_result.write("/device_0/sensor_1/Color_0/image/data",image.header.stamp,image);
        }
        sensor_msgs::Imu::ConstPtr simu = m.instantiate<sensor_msgs::Imu>();
        if (simu != NULL)
        {
            d435acccnt++;
            sensor_msgs::Imu imu=*simu;
            ros::Time enhanced_stamp(98,25462);
            enhanced_stamp.sec+=simu->header.stamp.sec;
            enhanced_stamp.nsec=simu->header.stamp.nsec;
            imu.header.stamp=enhanced_stamp;
            bag_result.write("/device_0/sensor_2/Accel_0/imu/data",imu.header.stamp,imu);
        }
    }

    rosbag::View view2(bag2, rosbag::TopicQuery(topics3));
    foreach(rosbag::MessageInstance const m, view2)
    {
        sensor_msgs::Image::ConstPtr sdimage = m.instantiate<sensor_msgs::Image>();
        if (sdimage != NULL) {
            d435depthcnt++;
            sensor_msgs::Image depthimg = *sdimage;
            ros::Time enhanced_stamp(98,25462);
            enhanced_stamp.sec+=sdimage->header.stamp.sec;
            enhanced_stamp.nsec=sdimage->header.stamp.nsec;
            depthimg.header.stamp=enhanced_stamp;
            bag_result.write("/device_0/sensor_0/Depth_0/image/data",depthimg.header.stamp,depthimg);
        }
        sensor_msgs::Imu::ConstPtr sgyr = m.instantiate<sensor_msgs::Imu>();
        if (sgyr != NULL) {
            d435gyrcnt++;
            sensor_msgs::Imu gyro = *sgyr;
            ros::Time enhanced_stamp(98,25462);
            enhanced_stamp.sec+=sgyr->header.stamp.sec;
            enhanced_stamp.nsec=sgyr->header.stamp.nsec;
            gyro.header.stamp=enhanced_stamp;
            bag_result.write("/device_0/sensor_2/Gyro_0/imu/data",gyro.header.stamp,gyro);
        }
    }

    rosbag::View view3(bag3, rosbag::TopicQuery(topics4));
    foreach(rosbag::MessageInstance const m, view3)
        {
            sensor_msgs::Image::ConstPtr sf1image = m.instantiate<sensor_msgs::Image>();
            if (sf1image != NULL) {
                fish1imgcnt++;
                sensor_msgs::Image f1img = *sf1image;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf1image->header.stamp.sec;
                enhanced_stamp.nsec=sf1image->header.stamp.nsec;
                f1img.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Fisheye_1/image/data",f1img.header.stamp,f1img);
            }
            sensor_msgs::CameraInfo::ConstPtr sf1cmi = m.instantiate<sensor_msgs::CameraInfo>();
            if (sf1cmi != NULL) {
                fish1cmicnt++;
                sensor_msgs::CameraInfo f1cmi = *sf1cmi;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf1cmi->header.stamp.sec;
                enhanced_stamp.nsec=sf1cmi->header.stamp.nsec;
                f1cmi.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Fisheye_1/info/camera_info",f1cmi.header.stamp,f1cmi);
            }
            sensor_msgs::Imu::ConstPtr sf1acc = m.instantiate<sensor_msgs::Imu>();
            if (sf1acc != NULL) {
                t265acccnt++;
                sensor_msgs::Imu f1acc = *sf1acc;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf1acc->header.stamp.sec;
                enhanced_stamp.nsec=sf1acc->header.stamp.nsec;
                f1acc.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Accel_0/imu/data",f1acc.header.stamp,f1acc);
            }
        }

    rosbag::View view4(bag3, rosbag::TopicQuery(topics5));
    foreach(rosbag::MessageInstance const m, view4)
        {
            sensor_msgs::Image::ConstPtr sf2image = m.instantiate<sensor_msgs::Image>();
            if (sf2image != NULL) {
                fish2imgcnt++;
                sensor_msgs::Image f2img = *sf2image;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf2image->header.stamp.sec;
                enhanced_stamp.nsec=sf2image->header.stamp.nsec;
                f2img.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Fisheye_2/image/data",f2img.header.stamp,f2img);
            }
            sensor_msgs::CameraInfo::ConstPtr sf2cmi = m.instantiate<sensor_msgs::CameraInfo>();
            if (sf2cmi != NULL) {
                fish2cmicnt++;
                sensor_msgs::CameraInfo f2cmi = *sf2cmi;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf2cmi->header.stamp.sec;
                enhanced_stamp.nsec=sf2cmi->header.stamp.nsec;
                f2cmi.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Fisheye_2/info/camera_info",f2cmi.header.stamp,f2cmi);
            }
            sensor_msgs::Imu::ConstPtr sf2gyr = m.instantiate<sensor_msgs::Imu>();
            if (sf2gyr != NULL) {
                t265gyrcnt++;
                sensor_msgs::Imu f2gyr = *sf2gyr;
                ros::Time enhanced_stamp(98,25462);
                enhanced_stamp.sec+=sf2gyr->header.stamp.sec;
                enhanced_stamp.nsec=sf2gyr->header.stamp.nsec;
                f2gyr.header.stamp=enhanced_stamp;
                bag_result.write("/device_0/sensor_0/Gyro_0/imu/data",f2gyr.header.stamp,f2gyr);
            }


        }

    bag.close();
    bag2.close();
    bag_result.close();
    cout<<"posecnt"<<posecnt<<endl;
    cout<<"tfcnt"<<tfcnt<<endl;
    cout<<"odomcnt"<<odomcnt<<endl;
    cout<<"d435imgcnt"<<d435imgcnt<<endl;
    cout<<"d435acccnt"<<d435acccnt<<endl;
    cout<<"d435depthcnt"<<d435depthcnt<<endl;
    cout<<"d435gyrcnt"<<d435gyrcnt<<endl;
    cout<<"fish1imgcnt"<<fish1imgcnt<<endl;
    cout<<"fish1cmicnt"<<fish1cmicnt<<endl;
    cout<<"t265acccnt"<<t265acccnt<<endl;
    cout<<"fish2imgcnt"<<fish2imgcnt<<endl;
    cout<<"fish2cmicnt"<<fish2cmicnt<<endl;
    cout<<"t265gyrcnt"<<t265gyrcnt<<endl;
    cout<<"writing done!"<<endl;

    return 0;
}