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

    rosbag::Bag bag1;//robot
    rosbag::Bag bag2;//robot
    rosbag::Bag bag3;//d435 depth
    rosbag::Bag bag4;//t265
    rosbag::Bag bag5;//d435 blur
    rosbag::Bag bag6;//t265 blur
    rosbag::Bag bag7;//d435 imu
    rosbag::Bag bag_result;
    bag1.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagrobot/2019-7-13-16-51-27_0.bag", rosbag::bagmode::Read);
    cout<<"bag1 opened"<<endl;
    bag2.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagrobot/2019-7-13-16-51-27_1.bag", rosbag::bagmode::Read);
    cout<<"bag2 opened"<<endl;
    bag3.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagcamera/3d/record-d400_3-filtered_depth.bag", rosbag::bagmode::Read);
    cout<<"d400_depth.bag opened"<<endl;
    bag4.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagcamera/3t/record-t265-3-filtered_exceptimg.bag", rosbag::bagmode::Read);
    cout<<"t265_exceptimg.bag opened"<<endl;
    bag5.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagcamera/3d/record-d400_3-filtered-face-blur.bag", rosbag::bagmode::Read);
    cout<<"d400 faceblur opened"<<endl;
    bag6.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/713bagcamera/3t/record-t265-3-face-blur.bag", rosbag::bagmode::Read);
    cout<<"t265 faceblur opened"<<endl;
    bag7.open("/home/ethan/Documents/Gaussian_src/intel_d435/record/data-collection-config/2019-07-30-11-19-21.bag",rosbag::bagmode::Read);
    cout<<"imu_bag opened"<<endl;
    bag_result.open("/media/ethan/LENOVO_USB_HDD/Dataset_drf/result/713result.bag", rosbag::bagmode::Write);
    //robot
    std::vector<std::string> topics;
    topics.push_back(std::string("/v5_current_pose"));int posecnt=0;
    topics.push_back(std::string("/tf"));int tfcnt=0;
    topics.push_back(std::string("/odom"));int odomcnt=0;
    topics.push_back(std::string("/rslidar_packets"));int rslidarcnt=0;
    //d435 1
    std::vector<std::string> topics2;
    topics2.push_back(std::string("/d400/imu"));int d435imucnt=0;
    //d435 2
    std::vector<std::string> topics3;
    topics3.push_back(std::string("/device_0/sensor_0/Depth_0/image/data"));int d435depthcnt=0;
    //t265 1
    std::vector<std::string> topics4;
    topics4.push_back(std::string("/device_0/sensor_0/Fisheye_1/info/camera_info")); int fish1cmicnt=0;
    topics4.push_back(std::string("/device_0/sensor_0/Accel_0/imu/data"));int t265acccnt=0;
    //t265 2
    std::vector<std::string> topics5;
    topics5.push_back(std::string("/device_0/sensor_0/Fisheye_2/info/camera_info"));int fish2cmicnt=0;
    topics5.push_back(std::string("/device_0/sensor_0/Gyro_0/imu/data"));int t265gyrcnt=0;
    //d435 3
    std::vector<std::string> topics6;
    topics6.push_back(std::string("/device_0/sensor_1/Color_0/image/data"));int d435imgcnt=0;
    //t265 3
    std::vector<std::string> topics7;
    topics7.push_back(std::string("/device_0/sensor_0/Fisheye_1/image/data"));int fish1imgcnt=0;
    //t265 4
    std::vector<std::string> topics8;
    topics8.push_back(std::string("/device_0/sensor_0/Fisheye_2/image/data"));int fish2imgcnt=0;
    cout<<"reading and writing...."<<endl;
    //robot
    rosbag::View view(bag1, rosbag::TopicQuery(topics));
    view.addQuery(bag2,rosbag::TopicQuery(topics));
    view.addQuery(bag7,rosbag::TopicQuery(topics2));
    foreach(rosbag::MessageInstance const m, view)
    {
        geometry_msgs::PoseWithCovarianceStamped::ConstPtr spose = m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();
        if (spose != NULL)
        {
            if(spose->header.stamp.toSec()>1563007905.953579&&spose->header.stamp.toSec()<1563008212.244068)
            {
                posecnt++;
                geometry_msgs::PoseWithCovarianceStamped pose=*spose;
                bag_result.write("/v5_current_pose",pose.header.stamp,pose);
            }
        }
        tf2_msgs::TFMessage::ConstPtr stf = m.instantiate<tf2_msgs::TFMessage>();
        if (stf != NULL)
        {
            if(stf->transforms[0].header.stamp.toSec()>1563007905.953579&&stf->transforms[0].header.stamp.toSec()<1563008212.244068)
            {
                tfcnt++;
                tf2_msgs::TFMessage tf=*stf;
                bag_result.write("/tf",tf.transforms[0].header.stamp,tf);
            }
        }
        nav_msgs::Odometry::ConstPtr sodom = m.instantiate<nav_msgs::Odometry>();
        if (sodom != NULL)
        {
            if (sodom->header.stamp.toSec() > 1563007905.953579 && sodom->header.stamp.toSec() < 1563008212.244068) {
                odomcnt++;
                nav_msgs::Odometry odom = *sodom;
                bag_result.write("/odom", odom.header.stamp, odom);
            }
        }
        sensor_msgs::Imu::ConstPtr simu = m.instantiate<sensor_msgs::Imu>();
        if (simu != NULL)
        {
                sensor_msgs::Imu imu = *simu;
                ros::Time enhanced_stamp(simu->header.stamp.toSec() - 1449167.661);
                imu.header.stamp = enhanced_stamp;
            if (imu.header.stamp.toSec() > 1563007905.953579 && imu.header.stamp.toSec() < 1563008212.244068) {
                d435imucnt++;
                bag_result.write("/d400/imu", imu.header.stamp, imu);
            }
        }
    }

    rosbag::View view2(bag3, rosbag::TopicQuery(topics3));
    foreach(rosbag::MessageInstance const m, view2)
    {
        sensor_msgs::Image::ConstPtr sdimage = m.instantiate<sensor_msgs::Image>();
        if (sdimage != NULL) {
            d435depthcnt++;
            sensor_msgs::Image depthimg = *sdimage;
            ros::Time enhanced_stamp(sdimage->header.stamp.toSec()+97.9769);
            if(enhanced_stamp.toSec() > 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068)
            {
                depthimg.header.stamp=enhanced_stamp;
                for(int i=sdimage->height*5/6*sdimage->step;i<sdimage->height*sdimage->step;i++)
                {
                    depthimg.data[i]=0;
                }
                bag_result.write("/device_0/sensor_0/Depth_0/image/data",depthimg.header.stamp,depthimg);
            }
        }
    }

    rosbag::View view3(bag4, rosbag::TopicQuery(topics4));
    foreach(rosbag::MessageInstance const m, view3)
        {
            sensor_msgs::CameraInfo::ConstPtr sf1cmi = m.instantiate<sensor_msgs::CameraInfo>();
            if (sf1cmi != NULL) {
                fish1cmicnt++;
                sensor_msgs::CameraInfo f1cmi = *sf1cmi;
                ros::Time enhanced_stamp(sf1cmi->header.stamp.toSec()+95.1983);
                f1cmi.header.stamp=enhanced_stamp;
                //bag_result.write("/device_0/sensor_0/Fisheye_1/info/camera_info",f1cmi.header.stamp,f1cmi);
            }
            sensor_msgs::Imu::ConstPtr sf1acc = m.instantiate<sensor_msgs::Imu>();
            if (sf1acc != NULL) {
                t265acccnt++;
                sensor_msgs::Imu f1acc = *sf1acc;
                ros::Time enhanced_stamp(sf1acc->header.stamp.toSec()+95.1983);
                if(enhanced_stamp.toSec()> 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068) {
                    f1acc.header.stamp = enhanced_stamp;
                    bag_result.write("/device_0/sensor_0/Accel_0/imu/data", f1acc.header.stamp, f1acc);
                }
            }
        }

    rosbag::View view4(bag4, rosbag::TopicQuery(topics5));
    foreach(rosbag::MessageInstance const m, view4)
        {

            sensor_msgs::CameraInfo::ConstPtr sf2cmi = m.instantiate<sensor_msgs::CameraInfo>();
            if (sf2cmi != NULL) {
                fish2cmicnt++;
                sensor_msgs::CameraInfo f2cmi = *sf2cmi;
                ros::Time enhanced_stamp(sf2cmi->header.stamp.toSec()+95.1983);
                f2cmi.header.stamp=enhanced_stamp;
               // bag_result.write("/device_0/sensor_0/Fisheye_2/info/camera_info",f2cmi.header.stamp,f2cmi);
            }
            sensor_msgs::Imu::ConstPtr sf2gyr = m.instantiate<sensor_msgs::Imu>();
            if (sf2gyr != NULL) {
                t265gyrcnt++;
                sensor_msgs::Imu f2gyr = *sf2gyr;
                ros::Time enhanced_stamp(sf2gyr->header.stamp.toSec()+95.1983);
                if(enhanced_stamp.toSec()> 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068) {
                    f2gyr.header.stamp = enhanced_stamp;
                    bag_result.write("/device_0/sensor_0/Gyro_0/imu/data", f2gyr.header.stamp, f2gyr);
                }
            }
        }

    rosbag::View view5(bag5, rosbag::TopicQuery(topics6));
    foreach(rosbag::MessageInstance const m, view5)
        {
            sensor_msgs::Image::ConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if (simage != NULL)
            {
                d435imgcnt++;
                sensor_msgs::Image image=*simage;
                ros::Time enhanced_stamp(simage->header.stamp.toSec()+97.9769);
                image.header.stamp=enhanced_stamp;
                if(enhanced_stamp.toSec()> 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068) {
                    bag_result.write("/device_0/sensor_1/Color_0/image/data", image.header.stamp, image);
                }
            }
        }

    rosbag::View view6(bag6, rosbag::TopicQuery(topics7));
        foreach(rosbag::MessageInstance const m, view6)
        {
            sensor_msgs::Image::ConstPtr sf1image = m.instantiate<sensor_msgs::Image>();
            if (sf1image != NULL) {
                fish1imgcnt++;
                sensor_msgs::Image f1img = *sf1image;

                ros::Time enhanced_stamp(sf1image->header.stamp.toSec()+95.1983);
                f1img.header.stamp=enhanced_stamp;
                if(enhanced_stamp.toSec()> 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068) {
                    bag_result.write("/device_0/sensor_0/Fisheye_1/image/data", f1img.header.stamp, f1img);
                }
            }
        }
    rosbag::View view7(bag6, rosbag::TopicQuery(topics8));
        foreach(rosbag::MessageInstance const m, view7)
        {
            sensor_msgs::Image::ConstPtr sf2image = m.instantiate<sensor_msgs::Image>();
            if (sf2image != NULL) {
                fish2imgcnt++;
                sensor_msgs::Image f2img = *sf2image;
                ros::Time enhanced_stamp(sf2image->header.stamp.toSec()+95.1983);
                f2img.header.stamp=enhanced_stamp;
                if(enhanced_stamp.toSec()> 1563007905.953579 && enhanced_stamp.toSec() < 1563008212.244068) {
                    bag_result.write("/device_0/sensor_0/Fisheye_2/image/data", f2img.header.stamp, f2img);
                }
            }
        }

    bag1.close();
    bag2.close();
    bag3.close();
    bag4.close();
    bag5.close();
    bag6.close();
    bag_result.close();
    cout<<"posecnt      "<<posecnt<<endl;
    cout<<"tfcnt        "<<tfcnt<<endl;
    cout<<"odomcnt      "<<odomcnt<<endl;
    cout<<"d435imgcnt   "<<d435imgcnt<<endl;
    cout<<"d435acccnt   "<<d435imucnt<<endl;
    cout<<"d435depthcnt "<<d435depthcnt<<endl;
    //cout<<"d435gyrcnt   "<<d435gyrcnt<<endl;
    cout<<"fish1imgcnt  "<<fish1imgcnt<<endl;
    cout<<"fish1cmicnt  "<<fish1cmicnt<<endl;
    cout<<"t265acccnt   "<<t265acccnt<<endl;
    cout<<"fish2imgcnt  "<<fish2imgcnt<<endl;
    cout<<"fish2cmicnt  "<<fish2cmicnt<<endl;
    cout<<"t265gyrcnt   "<<t265gyrcnt<<endl;
    cout<<"==========writing done!============="<<endl;

    return 0;
}