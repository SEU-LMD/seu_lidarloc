//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_ROS_PUBSUB_H
#define SEU_LIDARLOC_ROS_PUBSUB_H
#include <boost/bind.hpp>

#include <ros/ros.h>

#include "pubsub/pubusb.h"
#include "./ivsensorgps.h"

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

class ROSPubSub:public PubSubInterface{
public:
    ros::NodeHandle* pNH;
    std::vector<ros::Subscriber> subscribers;
    std::map<std::string, ros::Publisher> publishers;

    std::map<std::string, CallBackT> cloud_callbacks;
    std::map<std::string, CallBackT> imu_callbacks;
    std::map<std::string, CallBackT> gnss_ins_callbacks;
    std::map<std::string, CallBackT> wheel_callbacks;


    void initPubSub(int argc, char** argv, const std::string& node_name){
        ros::init(argc,argv, node_name);
        this->pNH = new ros::NodeHandle();
    }

    void run(){
        ros::spin();
    }
    //********************************************************************************************************

    void addPublisher(const std::string& topic_name, const DataType& type, int queue_size){

        if(type==DataType::LIDAR){
            publishers[topic_name] = this->pNH->advertise<sensor_msgs::PointCloud2>(topic_name, queue_size);
        }
        else if(type==DataType::ODOMETRY){
            publishers[topic_name] = this->pNH->advertise<nav_msgs::Odometry>(topic_name, queue_size);
        }
//        else if(type==DataType::PATH){
//
//        }
    }

    void PublishCloud(const std::string& topic_name, const CloudTypeXYZIRT& data ){
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(data.cloud, ros_cloud);
        ros_cloud.header.stamp = ros::Time().fromSec(data.timestamp);
        ros_cloud.header.frame_id = data.frame;
        publishers[topic_name].publish(ros_cloud);
    }

    void PublishCloud(const std::string& topic_name, const CloudTypeXYZI& data ){
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(data.cloud, ros_cloud);
        ros_cloud.header.stamp = ros::Time().fromSec(data.timestamp);
        ros_cloud.header.frame_id = data.frame;
        publishers[topic_name].publish(ros_cloud);
    }

    virtual void PublishCloud(const std::string &topic_name, const CloudTypeXYZICOLRANGE &data){
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(data.cloud, ros_cloud);
        ros_cloud.header.stamp = ros::Time().fromSec(data.timestamp);
        ros_cloud.header.frame_id = data.frame;
        publishers[topic_name].publish(ros_cloud);
    }


    void PublishOdometry(const std::string& topic_name, const OdometryType& data){
        nav_msgs::Odometry ros_odom;
        ros_odom.header.stamp = ros::Time().fromSec(data.timestamp);
        ros_odom.header.frame_id = data.frame;

        Eigen::Quaterniond q = data.pose.GetQ();
        Eigen::Vector3d t = data.pose.GetXYZ();
        ros_odom.pose.pose.orientation.x = q.x();
        ros_odom.pose.pose.orientation.y = q.y();
        ros_odom.pose.pose.orientation.z = q.z();
        ros_odom.pose.pose.orientation.w = q.w();
        ros_odom.pose.pose.position.x = t[0];
        ros_odom.pose.pose.position.y = t[1];
        ros_odom.pose.pose.position.z = t[2];
        publishers[topic_name].publish(ros_odom);
    }

    //********************************************************************************************************

    //点云对应的转换函数
    void convertROSCloudMsgToCloud(const sensor_msgs::PointCloud2ConstPtr cloud_in, CloudTypeXYZIRT& cloud_out ){
        cloud_out.frame = cloud_in->header.frame_id;
        cloud_out.timestamp = cloud_in->header.stamp.toSec();

        sensor_msgs::PointCloud2 currentCloudMsg;
        currentCloudMsg = std::move(*cloud_in);
        pcl::moveFromROSMsg(currentCloudMsg, cloud_out.cloud);
    }

    void CloudROSCallback(const sensor_msgs::PointCloud2ConstPtr& data, const std::string& topic_name){
        CloudTypeXYZIRT data_out;
        EZLOG(INFO)<<"CloudROSCallback*************"<<std::endl;
        convertROSCloudMsgToCloud(data, data_out);
        (this->cloud_callbacks[topic_name])(data_out);
    }

    //组合导航设备转换函数
    void convertROSGNSSINSMsgToGNSSINS(const gps_imu::ivsensorgpsConstPtr& gnss_ins_in, GNSSINSType& gnss_ins_out ){
        gnss_ins_out.lla[0] = gnss_ins_in->lon;
        gnss_ins_out.lla[1] = gnss_ins_in->lat;
        gnss_ins_out.lla[2] = gnss_ins_in->height;

        gnss_ins_out.pitch= gnss_ins_in->pitch;
        gnss_ins_out.roll = gnss_ins_in->roll;
        gnss_ins_out.yaw = gnss_ins_in->heading;

        gnss_ins_out.imu_angular_v[0] = gnss_ins_in->angx;
        gnss_ins_out.imu_angular_v[1] = gnss_ins_in->angy;
        gnss_ins_out.imu_angular_v[2] = gnss_ins_in->yaw;

        gnss_ins_out.imu_linear_v[0] = gnss_ins_in->accx;
        gnss_ins_out.imu_linear_v[1] = gnss_ins_in->accy;
        gnss_ins_out.imu_linear_v[2] = gnss_ins_in->accz;

        gnss_ins_out.timestamp = gnss_ins_in->header.stamp.toSec();
        gnss_ins_out.frame = gnss_ins_in->header.frame_id;

        gnss_ins_out.gps_status = gnss_ins_in->status;

    }
    void GNSSINSROSCallback(const gps_imu::ivsensorgpsConstPtr& data, const std::string& topic_name){
        GNSSINSType data_out;
        convertROSGNSSINSMsgToGNSSINS(data, data_out);
        (this->gnss_ins_callbacks[topic_name])(data_out);
    }

    void addSubscriber(const std::string& topic_name, const DataType& type, CallBackT callBack){

        if(type==DataType::LIDAR){
            this->subscribers.push_back(this->pNH->subscribe<sensor_msgs::PointCloud2>(topic_name,10,boost::bind(&ROSPubSub::CloudROSCallback,this,_1,topic_name)));
            this->cloud_callbacks[topic_name] = callBack;
        }
        else if(type==DataType::GNSS_INS){
            this->subscribers.push_back(this->pNH->subscribe< gps_imu::ivsensorgps>(topic_name,10,boost::bind(&ROSPubSub::GNSSINSROSCallback,this,_1,topic_name)));
            this->gnss_ins_callbacks[topic_name] = callBack;
        }
//        else if(type==DataType::IMU){
//
//        }
//        else if(type==DataType::WHEEL){
//
//        }
    }//end function addSubscriber

};
#endif //SEU_LIDARLOC_ROS_PUBSUB_H
