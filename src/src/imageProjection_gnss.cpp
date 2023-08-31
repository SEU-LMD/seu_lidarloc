#include "utility.h"
#include "lio_sam_6axis/cloud_info.h"

#include "ivsensorgps.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

#include <pcl/io/pcd_io.h>       //PCD读写类相关的头文件
#include <pcl/io/ply_io.h>

#include "config_helper.h"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP


Eigen::Vector3d ConvertToUTM(const Eigen::Vector3d& lla){

}

struct PoseTime{
    double timestamp;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
};

//定义点云类型：VelodynePointXYZIRT,PandarPointXYZIRT,OusterPointXYZIRT
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    uint8_t intensity;
    uint16_t ring;
    double latency;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT ( VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)
                                           (uint16_t, ring, ring)(double, latency, latency)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

static GeographicLib::LocalCartesian geoConverter;

class ImageProjection  {

private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pub_origin_cloud, pub_origin_cloud_world;
    ros::Publisher pub_deskew_cloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pub_gnss_odom;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> T_w_b_odomQueue;
    std::deque<PoseTime> gnssQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;


    pcl::PointCloud<PointXYZIRT>::Ptr cur_scan_cloud_body;//传入的原始点云
    pcl::PointCloud<PointType>::Ptr deskewCloud;//去畸变之后的全部点云
    pcl::PointCloud<PointType>::Ptr extractedCloud;

//    int deskewFlag;
    cv::Mat rangeMat;

    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam_6axis::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;
    Eigen::Matrix3d extrinc_rotation;//  rotation from gnss to lidar
    Eigen::Vector3d extrinc_translation;//translation from gnss to lidar
    Eigen::Matrix3d rotationinlidar;//  rotation from gnss to lidar
    Eigen::Vector3d translationinlidar;//translation from gnss to lidar
    Eigen::Isometry3d T_b_n;//translation matrix from gnss to lidar
    Eigen::Matrix4d T_L_G;
//    std::string RESULT_PATH;

public:
    ros::NodeHandle nh;
    ImageProjection() {

        subImu = nh.subscribe<gps_imu::ivsensorgps>(SensorConfig::gpsTopic,
                                                    2000,
                                                    &ImageProjection::gnssHandler,
                                                    this,
                                                    ros::TransportHints().tcpNoDelay());

        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(SensorConfig::pointCloudTopic,
                                                               5,
                                                               &ImageProjection::cloudHandler,
                                                               this,
                                                               ros::TransportHints().tcpNoDelay());

        //for debug use topic
        pub_origin_cloud = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud", 1);
        pub_origin_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/origin_cloud_world", 1);
        pub_deskew_cloud = nh.advertise<sensor_msgs::PointCloud2>("/deskew_cloud", 1);
        pub_gnss_odom = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 2000);

        //publish to next node
        pubLaserCloudInfo = nh.advertise<lio_sam_6axis::cloud_info>("lio_sam_6axis/deskew/cloud_info", 1);


        allocateMemory();//内存分配
//        resetParameters();//变量重置 comment fyy

//        pcl::console::setVerbosityLevel(pcl::console::L_ERROR); //设置PCL的打印日志级别为错误 fyy comment
//comment fyy
//        extrinc_rotation <<     0, 1, 0,
//                                -1, 0, 0,
//                                0, 0, 1;

//        extrinc_rotation <<   -0.000646759294, 0.999921769,    0.0124914668,
//                              -0.999989044,    -0.000588790089, -0.00464382452,
//                              -0.00463610638,  -0.0124943334,   0.999911195;

        //Eigen::Vector3d extrinc_translation(-0.33555608, -0.22477218, 0.38088802);//fyy comment

//comment by fyy
//        T_L_G.setIdentity();
//        T_L_G.block<3,3>(0,0) = extrinc_rotation;
//        T_L_G.topRightCorner(3,1) = extrinc_translation;

//        std::cout<<"*********************************************"<<T_L_G<<std::endl;

//        T_b_n.translation() = extrinc_translation;//translate matrix
//        T_b_n.linear() = extrinc_rotation;

//        T_b_n.translate(extrinc_translation) ;
//        T_b_n.rotate(extrinc_rotation) ;

//        RESULT_PATH = "/home/lsy/point/result.txt";
    }//end function ImageProjection

    void allocateMemory() {
        //
        cur_scan_cloud_body.reset(new pcl::PointCloud<PointXYZIRT>());
        deskewCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        deskewCloud->points.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);//将fullCloud的点集大小初始化为N_SCAN * Horizon_SCAN

        //
        cloudInfo.startRingIndex.assign(SensorConfig::N_SCAN, 0);//为cloudInfo中的startRingIndex、endRingIndex向量分配大小N_SCAN,并初始化为0
        cloudInfo.endRingIndex.assign(SensorConfig::N_SCAN, 0);

        cloudInfo.pointColInd.assign(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN, 0);//为pointColInd和pointRange向量分配N_SCAN * Horizon_SCAN大小,并初始化为0
        cloudInfo.pointRange.assign(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters() {  //重置变量，每次处理完一帧点云后调用,目的是为下一次点云的处理重新初始化相关变量,避免使用到上一帧数据的状态
        cur_scan_cloud_body->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(SensorConfig::N_SCAN, SensorConfig::Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;

        for (int i = 0; i < queueLength; ++i) {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(SensorConfig::N_SCAN, 0);
    }

    ~ImageProjection() {}

    int init = false;

    void gnssHandler(const gps_imu::ivsensorgpsConstPtr &gnss_msg) { //接收GNSS数据，存入队列

        if(!init)
        {
            geoConverter.Reset(gnss_msg->lat, gnss_msg->lon, gnss_msg->height);
            init = true;
            return;
        }

        double t_enu[3];
        geoConverter.Forward(gnss_msg->lat, gnss_msg->lon, gnss_msg->height,
                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate

        Eigen::Matrix3d z_matrix;//calculate Quaternion
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = -gnss_msg->heading * 3.1415926535 / 180.0;
        double pitch_Y = gnss_msg->pitch * 3.1415926535 / 180.0;
        double roll_X = gnss_msg->roll * 3.1415926535 / 180.0;
        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                    sin(heading_Z), cos(heading_Z),  0,
                    0,                 0,            1;

        x_matrix << 1,                 0,              0,
                    0,            cos(pitch_Y),     -sin(pitch_Y),
                    0,            sin(pitch_Y),    cos(pitch_Y);

        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                    0,                 1,               0 ,
                    -sin(roll_X),      0,         cos(roll_X);

        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);
        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);
        Eigen::Quaterniond q_w_b(R_w_b);//获得局部坐标系的四元数

        Eigen::Matrix4d T_w_b = Eigen::Matrix4d::Identity();
        T_w_b.block<3,3>(0,0) = R_w_b;
        T_w_b.block<3,1>(0,3) = t_w_b;
//        Eigen::Matrix4d T_w_l =  T_w_b*SensorConfig::T_L_B.inverse();
//        Eigen::Vector3d t_w_l = T_w_l.block<3,1>(0,3);
//        Eigen::Quaterniond q_w_l(T_w_l.block<3,3>(0,0));//获得局部坐标系的四元数

//        std::cout<<"befeore!!!!!!!!!!!!!!!!!!!!!1"<<std::endl;
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "map";
        odom_msg.pose.pose.position.x = t_w_b[0];
        odom_msg.pose.pose.position.y = t_w_b[1];
        odom_msg.pose.pose.position.z = t_w_b[2];
        odom_msg.pose.pose.orientation.w = q_w_b.w();
        odom_msg.pose.pose.orientation.x = q_w_b.x();
        odom_msg.pose.pose.orientation.y = q_w_b.y();
        odom_msg.pose.pose.orientation.z = q_w_b.z();
        pub_gnss_odom.publish(odom_msg);
        EZLOG(INFO)<<"publish odom"<<std::endl;



//delete
        nav_msgs::Odometry::Ptr thisgnss(new nav_msgs::Odometry());

        thisgnss->header.stamp = gnss_msg->header.stamp;
        thisgnss->pose.pose.position.x = t_w_b[0];
        thisgnss->pose.pose.position.y = t_w_b[1];
        thisgnss->pose.pose.position.z = t_w_b[2];
        thisgnss->pose.pose.orientation.w = q_w_b.w();
        thisgnss->pose.pose.orientation.x = q_w_b.x();
        thisgnss->pose.pose.orientation.y = q_w_b.y();
        thisgnss->pose.pose.orientation.z = q_w_b.z();

        std::lock_guard<std::mutex> lock2(odoLock);
        T_w_b_odomQueue.push_back(*thisgnss);//delete
        //backup for T_w_b_odomQueue
        PoseTime posetime;
        posetime.timestamp = gnss_msg->header.stamp.toSec();
        posetime.q = q_w_b;
        posetime.t = t_w_b;
        gnssQueue.push_back(posetime);

    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {

        double delta_max_time, delta_min_time;
        //转换格式, 获取每个点的delta time
        bool cahce_state = cachePointCloud(laserCloudMsg, delta_min_time ,delta_max_time );
        if(cahce_state==false){
            return;
        }
        /* pcl::PointCloud<PointXYZIRT>::Ptr origin_cloud_transformed(new pcl::PointCloud<PointXYZIRT>);
        Eigen::Matrix4f T_w_l = Eigen::Matrix4f::Identity();

        pcl::transformPointCloud(*cur_scan_cloud_body, *origin_cloud_transformed, T_w_l);*/
        Eigen::Matrix4d T_w_lidar_start;
        if (!deskewInfo(T_w_lidar_start))  //获取陀螺仪和里程计数据{}
        {
            return;
        }

//        static long long int idx = 0;
        //for deubug use
//        {
//            pcl::PointCloud<pcl::PointXYZ> temp ;
//            for(int i = 0; i < cur_scan_cloud_body -> points.size(); ++i){
//                pcl::PointXYZ p;
//                p.x = cur_scan_cloud_body -> points[i].x;
//                p.y = cur_scan_cloud_body -> points[i].y;
//                p.z = cur_scan_cloud_body -> points[i].z;
//                temp.push_back(p);
//            }
//            std::string filename = "/home/lsy/point/before"+ to_string( idx )+".ply";
//            pcl::io::savePLYFile(filename, temp);
////            ++idx;
//        }

//        std::cout<<"******************288"<<std::endl;
//        ++idx;
//        std::cout<<"***************************"<<idx<<std::endl;


        //very important function!!!!!!!!!!!!!!!!!!1
        //projectPointCloud(delta_min_time, delta_max_time, T_w_lidar_start);  //投影点云

//        {
//            pcl::PointCloud<pcl::PointXYZ> temp ;
//            for(int i = 0; i < deskewCloud -> points.size(); ++i){
//                pcl::PointXYZ p;
//                p.x = deskewCloud -> points[i].x;
//                p.y = deskewCloud -> points[i].y;
//                p.z = deskewCloud -> points[i].z;
//                temp.push_back(p);
//            }
//            std::string filename2 = "/home/lsy/point/after"+ to_string( idx )+".ply";
//            pcl::io::savePLYFile(filename2, temp);
//            ++idx;
//        }

        //cloudExtraction();  //提取分割点云

        //publishClouds();  //发布去畸变点云

        //resetParameters();
    }

    //
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg, double& min_timestamp, double& max_timestamp) {
        // cache point cloud
        //1.convert msg to pcl cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        sensor_msgs::PointCloud2 currentCloudMsg;
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        pcl::moveFromROSMsg(currentCloudMsg, *cur_scan_cloud_body);
        EZLOG(INFO) << "cur_scan_cloud_body pts size = " << cur_scan_cloud_body->points.size() << std::endl;

        Eigen::Matrix4f T_B_L = (SensorConfig::T_L_B.inverse()).cast<float>();
        pcl::PointCloud<PointXYZIRT> tf_cloud;
        pcl::transformPointCloud(*cur_scan_cloud_body, tf_cloud, T_B_L);
        *cur_scan_cloud_body = tf_cloud;


        //2.find the min and max timestamp in one scan and get the delta time stamp
        min_timestamp = 10000000;
        max_timestamp = -100000000;
        double t_0 = cur_scan_cloud_body->points[0].latency;
        for(int i=0; i < cur_scan_cloud_body->points.size(); ++i)
        {
            cur_scan_cloud_body->points[i].latency = cur_scan_cloud_body->points[i].latency - t_0;
//            if( cur_scan_cloud_body->points[i].latency <= std::numeric_limits<double>::epsilon()) {
//                cur_scan_cloud_body->points[i].latency = 0.0;
//            }


            if(cur_scan_cloud_body->points[i].latency > max_timestamp){
                max_timestamp = cur_scan_cloud_body->points[i].latency;

            }
            if(cur_scan_cloud_body->points[i].latency < min_timestamp){
                min_timestamp = cur_scan_cloud_body->points[i].latency;
            }
        }

        EZLOG(INFO)<<"min_timestamp = "<<min_timestamp<<std::endl;
        EZLOG(INFO)<<"max_timestamp = "<<max_timestamp<<std::endl;
        EZLOG(INFO)<<"one scan cloud delta time(second) = "<< max_timestamp - min_timestamp<<std::endl;

        //3.获取在ros系统下面的启动和结束时间戳
        // get timestamp  获得时间戳信息，找到每帧开始和结束的时间
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();
        timeScanEnd = timeScanCur + max_timestamp - min_timestamp;

//      if (deskewFlag == -1)
//                ROS_WARN(
//                        "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
//        }
        sensor_msgs::PointCloud2 cur_scan_cloud_body_ros;
        pcl::toROSMsg(*cur_scan_cloud_body, cur_scan_cloud_body_ros);
        cur_scan_cloud_body_ros.header.frame_id = "map";
        pub_origin_cloud.publish(cur_scan_cloud_body_ros);

        return true;
    }//end function cachePointCloud

    //找到激光起始时刻的位姿
    bool deskewInfo( Eigen::Matrix4d& T_w_b_lidar_start ) {

       odoLock.lock();

        // make sure GNSS data available for the scan
        if (T_w_b_odomQueue.empty() || T_w_b_odomQueue.front().header.stamp.toSec() > timeScanCur  //检查gnssQueue队列是否有数据,以及时间戳是否包含当前帧的开始和结束时间
            || T_w_b_odomQueue.back().header.stamp.toSec() < timeScanEnd) {
            EZLOG(INFO) << T_w_b_odomQueue.size() << std::endl;
            EZLOG(INFO) << T_w_b_odomQueue.front().header.stamp.toSec() - timeScanCur << std::endl;
            EZLOG(INFO) << T_w_b_odomQueue.back().header.stamp.toSec() - timeScanEnd << std::endl;
            EZLOG(INFO)<<timeScanCur - timeScanEnd<<std::endl;

            EZLOG(INFO)<<"Waiting for GNSS data ..."<<std::endl;
            odoLock.unlock();
            return false;
        }


        //将里程计中时间早于当前帧前0.01秒的数据取出
        while (!T_w_b_odomQueue.empty()) {
            if (T_w_b_odomQueue.front().header.stamp.toSec() < timeScanCur - 0.015)
                T_w_b_odomQueue.pop_front();
            else
                break;
        }



        for (int i = 0; i < (int) T_w_b_odomQueue.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

            if (T_w_b_odomQueue[i].header.stamp.toSec() > timeScanCur){
                double ktime = (timeScanCur - T_w_b_odomQueue[i - 1].header.stamp.toSec()) / (T_w_b_odomQueue[i].header.stamp.toSec() - T_w_b_odomQueue[i - 1].header.stamp.toSec());
                //平移插值
                Eigen::Vector3d t_w_b_lidar_start;
                t_w_b_lidar_start[0] = T_w_b_odomQueue[i - 1].pose.pose.position.x +
                                                    ktime * (T_w_b_odomQueue[i].pose.pose.position.x - T_w_b_odomQueue[i - 1].pose.pose.position.x);
                t_w_b_lidar_start[1] = T_w_b_odomQueue[i - 1].pose.pose.position.y +
                                                    ktime * (T_w_b_odomQueue[i].pose.pose.position.y - T_w_b_odomQueue[i - 1].pose.pose.position.y);
                t_w_b_lidar_start[2] = T_w_b_odomQueue[i - 1].pose.pose.position.z +
                                                    ktime * (T_w_b_odomQueue[i].pose.pose.position.z - T_w_b_odomQueue[i - 1].pose.pose.position.z);
                //旋转插值
                Eigen::Quaterniond q_w_b_lidar_start;//旋转
                Eigen::Quaterniond frontQ;
                Eigen::Quaterniond rearQ;
                frontQ.w() = T_w_b_odomQueue[i - 1].pose.pose.orientation.w;
                frontQ.x() = T_w_b_odomQueue[i - 1].pose.pose.orientation.x;
                frontQ.y() = T_w_b_odomQueue[i - 1].pose.pose.orientation.y;
                frontQ.z() = T_w_b_odomQueue[i - 1].pose.pose.orientation.z;

                rearQ.w() = T_w_b_odomQueue[i].pose.pose.orientation.w;
                rearQ.x() = T_w_b_odomQueue[i].pose.pose.orientation.x;
                rearQ.y() = T_w_b_odomQueue[i].pose.pose.orientation.y;
                rearQ.z() = T_w_b_odomQueue[i].pose.pose.orientation.z;
                q_w_b_lidar_start = frontQ.slerp(ktime,rearQ);

                //插值位姿态矩阵
                T_w_b_lidar_start = ConstructPoseT(t_w_b_lidar_start , q_w_b_lidar_start);

                //pusblish world origin cloud
                pcl::PointCloud<PointXYZIRT> cur_scan_cloud_w;
                pcl::transformPointCloud(*cur_scan_cloud_body, cur_scan_cloud_w, (T_w_b_lidar_start).cast<float>());
                sensor_msgs::PointCloud2 cur_scan_cloud_w_ros;
                pcl::toROSMsg(cur_scan_cloud_w, cur_scan_cloud_w_ros);
                cur_scan_cloud_w_ros.header.frame_id = "map";
                pub_origin_cloud_world.publish(cur_scan_cloud_w_ros);
            }
            continue;
        }

//        cloudInfo.odomAvailable = true;


        if (T_w_b_odomQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            odoLock.unlock();
            return  false;

        }

//        nav_msgs::Odometry endgnssMsg;
//
//        for (int i = 0; i < (int) T_w_b_odomQueue.size(); ++i) {  //遍历里程计队列，得到帧结束时间的里程计数据
//            endgnssMsg = T_w_b_odomQueue[i];
//
//            if (ROS_TIME(&endgnssMsg) < timeScanEnd)
//                continue;
//            else
//                break;
//        }
        odoLock.unlock();
        return true;
    }



    //find the start lidar gnss pose  startT startQ
    //
    void odomDeskewInfo() {

//        cloudInfo.odomAvailable = false;

    }

    //find x y z in lidar coordinate,Q in gnss coordinate at the pointtime and calculate translation matrix
    void findRotation(double pointTime, Eigen::Matrix4d *transformation_matrix) {//IMU数据中找到与当前点对应时刻的变换矩阵
        //lock mutex????????????????????????
        std::lock_guard<std::mutex> lock2(odoLock);
        for(int i = 0;i < (int)T_w_b_odomQueue.size(); i++){

            double currentgnsstime = T_w_b_odomQueue[i].header.stamp.toSec();

            if (currentgnsstime > pointTime)
            {

                nav_msgs::Odometry frontgnss = T_w_b_odomQueue[i - 1];
                nav_msgs::Odometry reargnss = T_w_b_odomQueue[i];

                double k_time = (pointTime - frontgnss.header.stamp.toSec())
                                /(reargnss.header.stamp.toSec() - frontgnss.header.stamp.toSec());

                //平移
                Eigen::Vector3d t_interpolate;
                t_interpolate[0] = frontgnss.pose.pose.position.x +
                                                   k_time * (reargnss.pose.pose.position.x-frontgnss.pose.pose.position.x);
                t_interpolate[1] = frontgnss.pose.pose.position.y +
                                                   k_time * (reargnss.pose.pose.position.y-frontgnss.pose.pose.position.y);
                t_interpolate[2] = frontgnss.pose.pose.position.z +
                                                   k_time * (reargnss.pose.pose.position.z-frontgnss.pose.pose.position.z);

                //旋转
                Eigen::Quaterniond frontQ;
                Eigen::Quaterniond rearQ;
                frontQ.w() = frontgnss.pose.pose.orientation.w;
                frontQ.x() = frontgnss.pose.pose.orientation.x;
                frontQ.y() = frontgnss.pose.pose.orientation.y;
                frontQ.z() = frontgnss.pose.pose.orientation.z;

                rearQ.w() = reargnss.pose.pose.orientation.w;
                rearQ.x() = reargnss.pose.pose.orientation.x;
                rearQ.y() = reargnss.pose.pose.orientation.y;
                rearQ.z() = reargnss.pose.pose.orientation.z;

                Eigen::Quaterniond currentQ = frontQ.slerp(k_time,rearQ);

//                Eigen::Vector3d thispoint(thisgnssmsg.pose.pose.position.x,thisgnssmsg.pose.pose.position.y,thisgnssmsg.pose.pose.position.z);    //当前点
//
//                Eigen::Vector3d translation = thispoint - startT; //平移向量
//                Eigen::Matrix3d rotation_matrix =  currentQ.toRotationMatrix() * startQ.toRotationMatrix().transpose();//startQ.toRotationMatrix().transpose() * currentQ.toRotationMatrix()
//
////                Eigen::Matrix3d rotation = T_b_n * rotation_matrix;//regard T as translation from gnss to lidar
////                Eigen::Matrix3d rotation = T_b_n.inverse() * rotation_matrix;
//                Eigen::Matrix3d rotation = extrinc_rotation * rotation_matrix * extrinc_rotation.transpose();
////                std::cout<<"******************rotation_matrix******************"<<rotation_matrix<<std::endl;

                transformation_matrix->block<3,3>(0,0) = currentQ.toRotationMatrix();
                transformation_matrix->topRightCorner(3,1) = t_interpolate;


//                rotationinlidar = currentQ;
//                translationinlidar = translation;
            }
            continue;

        }

    }

    PointType deskewPoint(PointType *point, double relTime, const Eigen::Matrix4d& T_w_lidar_start) {  //激光点去畸变
//        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
//comment by fyy
//        if (MappingConfig::deskewFlag == -1 )//判断是否可以去畸变
//            return *point;

        double pointTime = timeScanCur + relTime;//scan时间加相对时间，获得点的时间

//        Eigen::Isometry3d transformation_matrix;
        Eigen::Matrix4d T_w_lidar_end;
        findRotation(pointTime, &T_w_lidar_end);

//        std::cout<< "************************************deskewPoint"<<std::endl;

        Eigen::Vector3d newpoint;
        newpoint << point->x,point->y,point->z;
        Eigen::Matrix4d T_L_B = Eigen::Matrix4d::Identity();
        T_L_B.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        T_L_B.block<3,1>(0,3) = Eigen::Vector3d(0,0,-0.5135);
        Eigen::Matrix4d T_B_L = T_L_B.inverse();
        Eigen::Matrix4d T_lidar_start_now = T_L_B*T_w_lidar_start.inverse()*T_w_lidar_end*T_B_L;
        Eigen::Vector3d deskewnewpoint = T_lidar_start_now.block<3,3>(0,0) * newpoint  + T_lidar_start_now.block<3,1>(0,3);

        PointType newPoint;//get deskewed point
        newPoint.x = deskewnewpoint.x();
        newPoint.y = deskewnewpoint.y();
        newPoint.z = deskewnewpoint.z();
        newPoint.intensity = point->intensity;
//        std::cout<< "************************************deskewPoint"<<newPoint.x<<std::endl;

        return newPoint;
    }

    void projectPointCloud(const double& delta_min_time, const double& delta_max_time, const Eigen::Matrix4d& T_w_lidar_start) {

        int cloudSize = cur_scan_cloud_body->points.size(); //获取原始点云大小
//    std::cout << "point size raw: " << cloudSize << std::endl;
        // range image projection
        for (int i = 0; i < cloudSize; ++i) {  //遍历每一个点
            PointType thisPoint;
            thisPoint.x = cur_scan_cloud_body->points[i].x;//获取xyz、强度
            thisPoint.y = cur_scan_cloud_body->points[i].y;
            thisPoint.z = cur_scan_cloud_body->points[i].z;
            thisPoint.intensity = cur_scan_cloud_body->points[i].intensity;
            float range = pointDistance(thisPoint); //计算点的距离

            if (range < SensorConfig::lidarMinRange || range > SensorConfig::lidarMaxRange)
                continue;

            int rowIdn = cur_scan_cloud_body->points[i].ring;//获取行数
            if (rowIdn < 0 || rowIdn >= SensorConfig::N_SCAN)
                continue;

            if (rowIdn % SensorConfig::downsampleRate != 0)
                continue;

            int columnIdn = -1;//获取列号
            if (SensorConfig::sensor == LidarType::VELODYNE || SensorConfig::sensor == LidarType::OUSTER || SensorConfig::sensor == LidarType::HESAI) {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;//水平角分辨率
                static float ang_res_x = 360.0 / float(SensorConfig::Horizon_SCAN);//Horizon_SCAN=1800,每格0.2度
                //horizonAngle 为[-180,180],horizonAngle -90 为[-270,90],-round 为[-90,270], /ang_res_x 为[-450,1350]
                //+Horizon_SCAN/2后为[450,2250]
                // 即把horizonAngle从[-180,180]映射到[450,2250]
                columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + SensorConfig::Horizon_SCAN / 2;
                //大于1800，则减去1800，相当于把1801～2250映射到1～450
                //先把columnIdn从horizonAngle:(-PI,PI]转换到columnIdn:[H/4,5H/4],
                //然后判断columnIdn大小，把H到5H/4的部分切下来，补到0～H/4的部分。
                //将它的范围转换到了[0,H] (H:Horizon_SCAN)。
                //这样就把扫描开始的地方角度为0与角度为360的连在了一起，非常巧妙。
                //如果前方是x，左侧是y，那么正后方左边是180，右边是-180。这里的操作就是，把它展开成一幅图:
                //                   0
                //   90                        -90
                //          180 || (-180)
                //  (-180)   -----   (-90)  ------  0  ------ 90 -------180
                //变为:  90 ----180(-180) ---- (-90)  ----- (0)    ----- 90

                if (columnIdn >= SensorConfig::Horizon_SCAN)
                    columnIdn -= SensorConfig::Horizon_SCAN;
            } else if (SensorConfig::sensor == LidarType::LIVOX) {
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }

            if (columnIdn < 0 || columnIdn >= SensorConfig::Horizon_SCAN)//检查列号是否在范围内
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;
//            pcl::PointXYZ before = (thisPoint.x(),thisPoint.y(),thisPoint.z()); //temp <- thisPoint, ||temp - thisPoint||

            //very important function@!!!!!!!!!
            thisPoint = deskewPoint(&thisPoint, cur_scan_cloud_body->points[i].latency, T_w_lidar_start);//进行点云去畸变
//            std::cout<<"***************************************************604"<<std::endl;
//            pcl::PointXYZ after = (thisPoint.x,thisPoint.y,thisPoint.z);
//            double distance = pcl::euclideanDistance(point1, point2);;
//            std::cout<<"***************************************************"<<distance<<std::endl;

            rangeMat.at<float>(rowIdn, columnIdn) = range;//将去畸变后的点范围和坐标存入rangeMat
//            std::cout<<"***************************************************610"<<std::endl;

            int index = columnIdn + rowIdn * SensorConfig::Horizon_SCAN;//计算索引index,将去畸变后的点存入fullCloud
            deskewCloud->points[index] = thisPoint;
//            std::cout<<"***************************************************606"<<std::endl;
        }
    }


    void cloudExtraction() {

        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < SensorConfig::N_SCAN; ++i) {
            //提取特征的时候，每一行的前5个和最后5个不考虑
            cloudInfo.startRingIndex[i] = count - 1 + 5;//从第六个点开始找（前面5个不考虑）

            for (int j = 0; j < SensorConfig::Horizon_SCAN; ++j) {
                if (rangeMat.at<float>(i, j) != FLT_MAX) {
                    // mark the points' column index for marking occlusion later
                    //记录激光点对应的Horizon_SCAN方向上的索引
                    cloudInfo.pointColInd[count] = j;
                    // save range info  激光点距离
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud 加入有效激光点
                    extractedCloud->push_back(deskewCloud->points[j + i * SensorConfig::Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;//
        }
    }

    void publishClouds() {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed = publishCloud(pub_origin_cloud, extractedCloud, cloudHeader.stamp, SensorConfig::lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);

        publishCloud(pub_deskew_cloud, deskewCloud, cloudHeader.stamp, SensorConfig::lidarFrame);
    }
};

int main(int argc, char **argv) {

    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif
    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");

    ros::init(argc, argv, "img_proj");

    ImageProjection IP;

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
