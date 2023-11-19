//
// Created by slam on 23-10-21.
//
#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "udp_client.h"
#include "udp_seralize.h"

ros::Publisher publish_gnss_odom;
ros::Publisher publish_lidar_odom;
ros::Publisher publish_dr_odom;
ros::Publisher publish_fu_odom;


void PubilshOdometry(const UDP_CLENT& client);

int main(int argc, char** argv){
    ros::init(argc, argv, "rviz_udp");
    ros::NodeHandle nh;

     publish_gnss_odom=nh.advertise<nav_msgs::Odometry>("/gnss",100);
     publish_lidar_odom=nh.advertise<nav_msgs::Odometry>("/lidar",100);
     publish_dr_odom=nh.advertise<nav_msgs::Odometry>("/dr",100);
     publish_fu_odom=nh.advertise<nav_msgs::Odometry>("/fu",100);

    //udp_intial

    UDP_CLENT udp_client;
    int client_port = 8000;
    udp_client.init(client_port);  //只用来接受只绑定端口

    ros::Rate rate(100);
    while(ros::ok){
        udp_client.recvProcess();
        std::cout<<udp_client.rcv_msg<<std::endl;
        PubilshOdometry(udp_client);
        ros::spinOnce(); // 处理订阅和发布的消息
        rate.sleep();
    }

}

void PubilshOdometry(const UDP_CLENT& client){
    Vis_Odometry vis_Odometry;
    Vis_Odometry::fromString(client.rcv_msg,vis_Odometry);
    //"gn" means = gnss
    if(vis_Odometry.type=="gn"){
        nav_msgs::Odometry gnss_msg;
        gnss_msg.header.stamp =ros::Time::now();
        gnss_msg.header.frame_id = "/map";
        gnss_msg.pose.pose.position.x = vis_Odometry.t[0];
        gnss_msg.pose.pose.position.y = vis_Odometry.t[1];
        gnss_msg.pose.pose.position.z = vis_Odometry.t[2];
        gnss_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        gnss_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        gnss_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        gnss_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_gnss_odom.publish(gnss_msg);
    }
    //"li" means lio
    else if(vis_Odometry.type=="li"){
        nav_msgs::Odometry lida_msg;
        lida_msg.header.stamp =ros::Time::now();
        lida_msg.header.frame_id = "/map";
        lida_msg.pose.pose.position.x = vis_Odometry.t[0];
        lida_msg.pose.pose.position.y = vis_Odometry.t[1];
        lida_msg.pose.pose.position.z = vis_Odometry.t[2];
        lida_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        lida_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        lida_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        lida_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_lidar_odom.publish(lida_msg);

    }
    else if(vis_Odometry.type=="dr"){
        nav_msgs::Odometry dr_msg;
        dr_msg.header.stamp =ros::Time::now();
        dr_msg.header.frame_id = "/map";
        dr_msg.pose.pose.position.x = vis_Odometry.t[0];
        dr_msg.pose.pose.position.y = vis_Odometry.t[1];
        dr_msg.pose.pose.position.z = vis_Odometry.t[2];
        dr_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        dr_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        dr_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        dr_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_dr_odom.publish(dr_msg);
    }
    else if(vis_Odometry.type=="fu"){
        nav_msgs::Odometry dr_msg;
        dr_msg.header.stamp =ros::Time::now();
        dr_msg.header.frame_id = "/map";
        dr_msg.pose.pose.position.x = vis_Odometry.t[0];
        dr_msg.pose.pose.position.y = vis_Odometry.t[1];
        dr_msg.pose.pose.position.z = vis_Odometry.t[2];
        dr_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        dr_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        dr_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        dr_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_fu_odom.publish(dr_msg);
    }

}
