//
// Created by slam on 23-10-21.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include  <pcl_conversions/pcl_conversions.h>
#include "udp_client.h"
#include "udp_seralize.h"
#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/point_types.h>     //PCL中支持的点类型的头文件
#include <pcl/point_cloud.h>



struct MyPointType{
    PCL_ADD_POINT4D;

    PCL_ADD_INTENSITY;

    int down_grid_index;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(int, down_grid_index, down_grid_index)
)

typedef MyPointType PointType;



ros::Publisher publish_corner_map;
ros::Publisher publish_surf_map;
ros::Publisher publish_gnss_odom;
ros::Publisher publish_gnss_test_odom;
ros::Publisher publish_lidar_odom;
ros::Publisher publish_dr_odom;
ros::Publisher publish_fu_odom;
ros::Publisher publish_fu_test_odom;
std::string pcd_path;

void PubilshOdometry(const UDP_CLENT& client);
void PubilshCloud();
void Pubfixedframe();

int main(int argc, char** argv){
    ros::init(argc, argv, "rviz_udp");
    ros::NodeHandle nh;

    if(argc < 2){
        std::cout<<"please input path!!!"<<std::endl;
        exit(1);
    }
    pcd_path = argv[1];

     publish_corner_map = nh.advertise<sensor_msgs::PointCloud2>("/corner_cloud",100);
     publish_surf_map = nh.advertise<sensor_msgs::PointCloud2>("/surf_cloud",100);
     publish_gnss_odom=nh.advertise<nav_msgs::Odometry>("/gnss",100);
     publish_gnss_test_odom=nh.advertise<nav_msgs::Odometry>("/ground_truth",100);
     publish_lidar_odom=nh.advertise<nav_msgs::Odometry>("/lidar",100);
     publish_dr_odom=nh.advertise<nav_msgs::Odometry>("/dr",100);
     publish_fu_odom=nh.advertise<nav_msgs::Odometry>("/fu",100);
     publish_fu_test_odom=nh.advertise<nav_msgs::Odometry>("/loc_result",100);

    //udp_intial

    UDP_CLENT udp_client;
    int client_port = 8000;
    udp_client.init(client_port);  //只用来接受只绑定端口
    PubilshCloud();

    while(1){
        udp_client.recvProcess();
        std::cout<<udp_client.rcv_msg<<std::endl;
        Pubfixedframe();
        PubilshOdometry(udp_client);
    }

}

void Pubfixedframe(){
    static tf::TransformBroadcaster br;
    static tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/world"));
}

void PubilshCloud(){
    pcl::PointCloud<PointType>::Ptr in_cloud(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (pcd_path + "global_surf.pcd", *in_cloud) == -1){
        std::cout <<"cann“t find pori map"<<std::endl;
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*in_cloud, ros_cloud);
    ros_cloud.header.frame_id = "/map";
    publish_surf_map.publish(ros_cloud);

    
    pcl::PointCloud<PointType>::Ptr in_cloud2(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType> (pcd_path + "global_corner.pcd", *in_cloud2) == -1){
        std::cout <<"cann“t find pori map"<<std::endl;
    }
    pcl::toROSMsg(*in_cloud2, ros_cloud);
    ros_cloud.header.frame_id = "/map";
    publish_corner_map.publish(ros_cloud);
    std::cout<< "地图已发布！！！！！！！！！！！！！！！！" << std::endl;
}

void PubilshOdometry(const UDP_CLENT& client){
    Vis_Odometry vis_Odometry;
    Vis_Odometry::fromString(client.rcv_msg,vis_Odometry);
    //"gn" means = gnss
    if(vis_Odometry.type=="gn"){
        nav_msgs::Odometry gnss_msg;
        gnss_msg.header.stamp =ros::Time::now();
        gnss_msg.header.frame_id = "/world";
        gnss_msg.pose.pose.position.x = vis_Odometry.t[0];
        gnss_msg.pose.pose.position.y = vis_Odometry.t[1];
        gnss_msg.pose.pose.position.z = vis_Odometry.t[2];
        gnss_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        gnss_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        gnss_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        gnss_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_gnss_odom.publish(gnss_msg);

	nav_msgs::Odometry gnss_test_msg;
        gnss_test_msg.header.stamp =ros::Time::now();
        gnss_test_msg.header.frame_id = "/world";
        gnss_test_msg.pose.pose.position.x = vis_Odometry.t[0];
        gnss_test_msg.pose.pose.position.y = vis_Odometry.t[1];
        gnss_test_msg.pose.pose.position.z = 0;
        gnss_test_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        gnss_test_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        gnss_test_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        gnss_test_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_gnss_test_odom.publish(gnss_test_msg);

    }
    //"li" means lio
    else if(vis_Odometry.type=="li"){
        nav_msgs::Odometry lida_msg;
        lida_msg.header.stamp =ros::Time::now();
        lida_msg.header.frame_id = "/world";
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
        dr_msg.header.frame_id = "/world";
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
        dr_msg.header.frame_id = "/world";
        dr_msg.pose.pose.position.x = vis_Odometry.t[0];
        dr_msg.pose.pose.position.y = vis_Odometry.t[1];
        dr_msg.pose.pose.position.z = vis_Odometry.t[2];
        dr_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        dr_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        dr_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        dr_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_fu_odom.publish(dr_msg);

	nav_msgs::Odometry dr_test_msg;
        dr_test_msg.header.stamp =ros::Time::now();
        dr_test_msg.header.frame_id = "/world";
        dr_test_msg.pose.pose.position.x = vis_Odometry.t[0];
        dr_test_msg.pose.pose.position.y = vis_Odometry.t[1];
        dr_test_msg.pose.pose.position.z = 0;
        dr_test_msg.pose.pose.orientation.x = vis_Odometry.q.x();
        dr_test_msg.pose.pose.orientation.y = vis_Odometry.q.y();
        dr_test_msg.pose.pose.orientation.z = vis_Odometry.q.z();
        dr_test_msg.pose.pose.orientation.w = vis_Odometry.q.w();

        publish_fu_test_odom.publish(dr_test_msg);

	
        static tf::TransformBroadcaster bf;
        static tf::Transform transformbf;

        transformbf.setOrigin(tf::Vector3(dr_msg.pose.pose.position.x, dr_msg.pose.pose.position.y, dr_msg.pose.pose.position.z));
        transformbf.setRotation(tf::Quaternion(dr_msg.pose.pose.orientation.x,dr_msg.pose.pose.orientation.y ,dr_msg.pose.pose.orientation.z, dr_msg.pose.pose.orientation.w));
        bf.sendTransform(tf::StampedTransform(transformbf, ros::Time::now(), "/map", "/fu"));
    }

}
