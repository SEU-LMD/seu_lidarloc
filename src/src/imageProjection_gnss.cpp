#include "utility.h"
#include "timer.h"

#include "cloud_info.h"

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

static GeographicLib::LocalCartesian geoConverter;

class CloudWithTime{
public:
    pcl::PointCloud<PointXYZIRT>::Ptr cloud;
    double max_ros_timestamp;
    double min_ros_timestamp;
    double min_latency_timestamp;
    CloudWithTime(){
        cloud.reset(new pcl::PointCloud<PointXYZIRT>());
    };
};

class PoseWithTime{
public:
    double ros_time_stamp;
    PoseT pose;
    PoseWithTime(const double& ros_time_stamp_,
                 const PoseT& T_){
        ros_time_stamp = ros_time_stamp_;
        pose = T_;
    };
};

class ImageProjection  {

private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;

    ros::Publisher pub_origin_cloud;
    ros::Publisher pub_origin_cloud_world;
    ros::Publisher pub_deskew_cloud_world;

    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubextractedCloud;
    ros::Publisher pub_gnss_odom;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> T_w_b_odomQueue;
    std::deque<PoseTime> gnssQueue;

    std::deque<CloudWithTime> cloudQueue;
    std::deque<PoseWithTime> poseQueue;//T_w_l

    std::deque<pcl::PointCloud<PointXYZIRT>> cloud_deque;//
    std::mutex cloud_mutex;
    pcl::PointCloud<PointXYZIRT>::Ptr cur_scan_cloud_body;//传入的原始点云
    pcl::PointCloud<PointType>::Ptr deskewCloud_body;//去畸变之后的全部点云
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    cv::Mat rangeMat;

    lio_sam_6axis::cloud_info cloudInfo;

    std_msgs::Header cloudHeader;

    vector<int> columnIdnCountVec;
//    Eigen::Matrix3d extrinc_rotation;//  rotation from gnss to lidar
//    Eigen::Vector3d extrinc_translation;//translation from gnss to lidar
//    Eigen::Matrix3d rotationinlidar;//  rotation from gnss to lidar
//    Eigen::Vector3d translationinlidar;//translation from gnss to lidar
//    Eigen::Isometry3d T_b_n;//translation matrix from gnss to lidar
//    Eigen::Matrix4d T_L_G;

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
        pub_deskew_cloud_world = nh.advertise<sensor_msgs::PointCloud2>("/deskew_cloud_world", 1);

        pubextractedCloud = nh.advertise<sensor_msgs::PointCloud2>("lio_sam_6axis/deskew/cloud_deskewed", 1);


        pub_gnss_odom = nh.advertise<nav_msgs::Odometry>("/gnss_odom", 2000);

        //publish to next node
        pubLaserCloudInfo = nh.advertise<lio_sam_6axis::cloud_info>("lio_sam_6axis/deskew/cloud_info", 1);

        allocateMemory();//内存分配

    }//end function ImageProjection

    void do_work(){
        while(1){
            if(cloudQueue.size()!=0){

                odoLock.lock();
                std::deque<PoseWithTime> odo_poses_copy;
                odo_poses_copy = poseQueue;
                double odo_min_ros_time =  poseQueue.front().ros_time_stamp;
                double odo_max_ros_time = poseQueue.back().ros_time_stamp;
                odoLock.unlock();

                cloud_mutex.lock();
                CloudWithTime cloudinfo_cur = cloudQueue.front();
//                cloudHeader = cloudQueue.front().cloud->header;
                double cloud_min_ros_timestamp = cloudinfo_cur.min_ros_timestamp;
                double cloud_max_ros_timestamp = cloudinfo_cur.max_ros_timestamp;

                if(odo_min_ros_time<cloud_min_ros_timestamp && odo_max_ros_time>cloud_max_ros_timestamp){

                    cloudQueue.pop_front();
                    cloud_mutex.unlock();

//                    EZLOG(INFO)<<"odo_min_ros_time = "<<odo_min_ros_time - odo_min_ros_time<<std::endl;
//                    EZLOG(INFO)<<"odo_max_ros_time = "<<odo_max_ros_time - odo_min_ros_time<<std::endl;
//                    EZLOG(INFO)<<"cloud_min_ros_timestamp = "<<cloud_min_ros_timestamp - odo_min_ros_time<<std::endl;
//                    EZLOG(INFO)<<"cloud_max_ros_timestamp = "<<cloud_max_ros_timestamp - odo_min_ros_time<<std::endl;

                    //do veryt important work
                    //1.deskew
                    PoseT T_lidar_first_pose;
                   double cost_time_findpose =  FindLidarFirstPose(cloudinfo_cur , odo_poses_copy,
                                                                   T_lidar_first_pose);
                   EZLOG(INFO)<<"cost_time_findpose(ms) = "<<cost_time_findpose<<std::endl;

                    //2.imgprojection

                    double cost_time_projpc = projectPointCloud(cloudinfo_cur,odo_poses_copy,T_lidar_first_pose);

                    EZLOG(INFO)<<"cost_time_projpc(ms) = "<<cost_time_projpc<<std::endl;

                    //3.cloudExtraction

                    double cost_time_cloudextraction = cloudExtraction();

                    EZLOG(INFO)<<"cost_time_cloudextraction(ms) = "<<cost_time_cloudextraction<<std::endl;

                    //4. publish topic

                    publishClouds();

                    //5.pop used odom
                    odoLock.lock();
                    while(poseQueue.front().ros_time_stamp < cloud_max_ros_timestamp - 0.02){
                        poseQueue.pop_front();
                    }
                    odoLock.unlock();
                }
                else{
                    cloud_mutex.unlock();
                }
                resetParameters();
            }
            else{
                sleep(0.05);
            }
        }
    }

    void allocateMemory() {
        //
        cur_scan_cloud_body.reset(new pcl::PointCloud<PointXYZIRT>());
        deskewCloud_body.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        deskewCloud_body->points.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);//将fullCloud的点集大小初始化为N_SCAN * Horizon_SCAN

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
        PoseT T_w_b(t_w_b, R_w_b);

        PoseT T_w_l = PoseT(T_w_b.pose*(SensorConfig::T_L_B.inverse()));
        PoseWithTime pose_with_time(gnss_msg->header.stamp.toSec(),T_w_l);\
        odoLock.lock();
        poseQueue.push_back(pose_with_time);
        odoLock.unlock();

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "map";
        Eigen::Vector3d t_w_l = T_w_l.GetT();
        Eigen::Quaterniond q_w_l = T_w_l.GetQ();
        odom_msg.pose.pose.position.x = t_w_l[0];
        odom_msg.pose.pose.position.y = t_w_l[1];
        odom_msg.pose.pose.position.z = t_w_l[2];
        odom_msg.pose.pose.orientation.w = q_w_l.w();
        odom_msg.pose.pose.orientation.x = q_w_l.x();
        odom_msg.pose.pose.orientation.y = q_w_l.y();
        odom_msg.pose.pose.orientation.z = q_w_l.z();
        pub_gnss_odom.publish(odom_msg);
//        EZLOG(INFO)<<"publish odom"<<std::endl;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {

        CloudWithTime cloudinfo;
        //转换格式, 获取每个点的delta time
//        double get_cloudtime_cost_time = GetCloudTime(laserCloudMsg, cloudinfo);
        GetCloudTime(laserCloudMsg, cloudinfo);
        cloudHeader = laserCloudMsg->header;
        cloud_mutex.lock();
        cloudQueue.push_back(cloudinfo);
        cloud_mutex.unlock();
//        Eigen::Matrix4d T_w_lidar_start;
//        if (!FindLidarFirstPose(T_w_lidar_start))  //获取陀螺仪和里程计数据{}
//        {
//            return;
//        }

        //very important function!!!!!!!!!!!!!!!!!!1
//        projectPointCloud(min_timestamp, max_timestamp, T_w_lidar_start);  //投影点云

//        cloudExtraction();  //提取分割点云

        //publishClouds();  //发布去畸变点云

//        resetParameters();
    }

    //
    void GetCloudTime(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg,
                         CloudWithTime& cloud_info) {
        TicToc timer;
        sensor_msgs::PointCloud2 currentCloudMsg;
        currentCloudMsg = std::move(*laserCloudMsg);
        pcl::moveFromROSMsg(currentCloudMsg, *cloud_info.cloud);
//        EZLOG(INFO) << "cur_scan_cloud_body pts size = " << cloud_info.cloud->points.size() << std::endl;

        double lidar_start_ros_time = laserCloudMsg->header.stamp.toSec();

        //2.find the min and max timestamp in one scan and get the delta time stamp
        double min_timestamp = 10000000;
        double max_timestamp = -100000000;
        double t_0 = cloud_info.cloud->points[0].latency;
        for(int i=0; i < cloud_info.cloud->points.size(); ++i)
        {
            cloud_info.cloud->points[i].latency = cloud_info.cloud->points[i].latency - t_0;

            if(cloud_info.cloud->points[i].latency > max_timestamp){
                max_timestamp = cloud_info.cloud->points[i].latency;

            }
            if(cloud_info.cloud->points[i].latency < min_timestamp){
                min_timestamp = cloud_info.cloud->points[i].latency;
            }
        }
        cloud_info.min_ros_timestamp = lidar_start_ros_time;
        cloud_info.max_ros_timestamp = max_timestamp - min_timestamp + cloud_info.min_ros_timestamp;
        cloud_info.min_latency_timestamp = min_timestamp;
//        EZLOG(INFO)<<"min_timestamp = "<<min_timestamp<<std::endl;
//        EZLOG(INFO)<<"max_timestamp = "<<max_timestamp<<std::endl;
//        EZLOG(INFO)<<"one scan cloud delta time(second) = "<< max_timestamp - min_timestamp<<std::endl;
        double cost_time = timer.toc();
        EZLOG(INFO)<<"GetCloudTime cost time(ms) = "<<cost_time<<std::endl;
//        {
//            Eigen::Matrix4f T_B_L = (SensorConfig::T_L_B.inverse()).cast<float>();
//            pcl::PointCloud<PointXYZIRT> tf_cloud;
//            pcl::transformPointCloud(*cur_scan_cloud_body, tf_cloud, T_B_L);
//            *cur_scan_cloud_body = tf_cloud;
//            sensor_msgs::PointCloud2 cur_scan_cloud_body_ros;
//            pcl::toROSMsg(*cur_scan_cloud_body, cur_scan_cloud_body_ros);
//            cur_scan_cloud_body_ros.header.frame_id = "map";
//            pub_origin_cloud.publish(cur_scan_cloud_body_ros);
//        }


//        return cost_time;
    }//end function GetCloudTime

    //找到激光起始时刻的位姿
    double  FindLidarFirstPose(const CloudWithTime& cloudinfo, const std::deque<PoseWithTime>& pose_deque ,
                               PoseT& T_w_b_lidar_start) {

        TicToc timer;

        pcl::PointCloud<PointType> originCloud_w;

        for (int i = 0; i < (int) pose_deque.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

            if (pose_deque[i].ros_time_stamp > cloudinfo.min_ros_timestamp){
                double ktime = (cloudinfo.min_ros_timestamp - pose_deque[i - 1].ros_time_stamp) / (pose_deque[i].ros_time_stamp - pose_deque[i - 1].ros_time_stamp);
                //平移插值
                Eigen::Vector3d t_w_b_lidar_start;
                t_w_b_lidar_start = pose_deque[i - 1].pose.GetT() +
                                                    ktime * (pose_deque[i].pose.GetT() - pose_deque[i - 1].pose.GetT());

                //旋转插值
                Eigen::Quaterniond q_w_b_lidar_start;//旋转
                q_w_b_lidar_start = pose_deque[i-1].pose.GetQ().slerp(ktime,pose_deque[i].pose.GetQ());

                //插值位姿态矩阵

                 T_w_b_lidar_start  = PoseT (t_w_b_lidar_start , q_w_b_lidar_start);

                //pusblish world origin cloud
                pcl::PointCloud<PointXYZIRT> cur_scan_cloud_w;
//                pcl::transformPointCloud(*cur_scan_cloud_body, cur_scan_cloud_w, (T_w_b_lidar_start).pose.cast<float>());
                pcl::transformPointCloud(*cloudinfo.cloud, cur_scan_cloud_w, (T_w_b_lidar_start).pose.cast<float>());
                sensor_msgs::PointCloud2 cur_scan_cloud_w_ros;
                pcl::toROSMsg(cur_scan_cloud_w, cur_scan_cloud_w_ros);
                cur_scan_cloud_w_ros.header.frame_id = "map";
                pub_origin_cloud_world.publish(cur_scan_cloud_w_ros);
            }
            continue;
        }
        return timer.toc();
    }

    double projectPointCloud(const CloudWithTime& cloudinfo, const std::deque<PoseWithTime>& pose_deque, const PoseT& T_w_b_lidar_start) {

        TicToc timer;

        int cloudSize = cloudinfo.cloud->points.size(); //获取原始点云大小

        pcl::PointCloud<PointType> deskewCloud_body_offset;
        // range image projection
        int valid_num = 0;
        int range_outlier = 0;
        int row_outlier = 0;
        int col_outlier = 0;
        int rangemat_outlier = 0;
        for (int i = 0; i < cloudSize; ++i) {  //遍历每一个点
            PointType thisPoint;
            thisPoint.x = cloudinfo.cloud->points[i].x;//获取xyz、强度
            thisPoint.y = cloudinfo.cloud->points[i].y;
            thisPoint.z = cloudinfo.cloud->points[i].z;
            thisPoint.intensity = cloudinfo.cloud->points[i].intensity;
            float range = pointDistance(thisPoint); //计算点的距离

            if (range < SensorConfig::lidarMinRange || range > SensorConfig::lidarMaxRange){
                range_outlier++;
                continue;
            }

            int rowIdn = cloudinfo.cloud->points[i].ring;//获取行数
            if (rowIdn < 0 || rowIdn >= SensorConfig::N_SCAN){
                row_outlier++;
                continue;
            }

//            if (rowIdn % SensorConfig::downsampleRate != 0)
//                continue;

            int columnIdn = -1;//获取列号

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;//水平角分辨率
            static float ang_res_x = 360.0 / float(SensorConfig::Horizon_SCAN);//Horizon_SCAN=1800,每格0.2度

            columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + SensorConfig::Horizon_SCAN / 2;

            if (columnIdn >= SensorConfig::Horizon_SCAN)
                columnIdn -= SensorConfig::Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= SensorConfig::Horizon_SCAN)//检查列号是否在范围内            // }
            {
                col_outlier++;
                continue;
            }

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX){
                rangemat_outlier++;
                continue;
            }

            //very important function@!!!!!!!!!
            thisPoint = deskewPoint(&thisPoint, cloudinfo.cloud->points[i].latency - cloudinfo.min_latency_timestamp, T_w_b_lidar_start,cloudinfo,pose_deque);//进行点云去畸变
            rangeMat.at<float>(rowIdn, columnIdn) = range;//将去畸变后的点范围和坐标存入rangeMat

            int index = columnIdn + rowIdn * SensorConfig::Horizon_SCAN;//计算索引index,将去畸变后的点存入fullCloud
            deskewCloud_body->points[index] = thisPoint;
            //just for show
            auto thisPoint_offset = thisPoint;
            thisPoint_offset.z = thisPoint_offset.z + 20.0;
            deskewCloud_body_offset.points.push_back(thisPoint_offset);
            valid_num++;
        }//end for

//        use for debug
//        EZLOG(INFO)<<"valid_num num = "<<valid_num<<std::endl;
//        EZLOG(INFO)<<"range_outlier num = "<<range_outlier<<std::endl;
//        EZLOG(INFO)<<"row_outlier num = "<<row_outlier<<std::endl;
//        EZLOG(INFO)<<"col_outlier num = "<<col_outlier<<std::endl;
//        EZLOG(INFO)<<"rangemat_outlier num = "<<rangemat_outlier<<std::endl;

        pcl::PointCloud<PointType> deskewCloud_w;
//        EZLOG(INFO)<<"deskewCloud_body size = "<<deskewCloud_body->points.size()<<std::endl;
//        EZLOG(INFO)<<"deskewCloud_body_offset size = "<<deskewCloud_body_offset.points.size()<<std::endl;
        pcl::transformPointCloud(deskewCloud_body_offset, deskewCloud_w, (T_w_b_lidar_start).pose.cast<float>());
//        EZLOG(INFO)<<"deskewCloud_w size = "<<deskewCloud_w.points.size()<<std::endl;
        sensor_msgs::PointCloud2 deskewCloud_w_ros;
        pcl::toROSMsg(deskewCloud_w, deskewCloud_w_ros);
        deskewCloud_w_ros.header.frame_id = "map";
        pub_deskew_cloud_world.publish(deskewCloud_w_ros);

        return timer.toc();
    }

    //find x y z in lidar coordinate,Q in gnss coordinate at the pointtime and calculate translation matrix
    void findRotation(const double pointTime, const CloudWithTime& cloudinfo, const std::deque<PoseWithTime>& pose_deque,
                      PoseT& T_w_b_lidar_now) {//IMU数据中找到与当前点对应时刻的变换矩阵

        for(int i = 0;i < (int)pose_deque.size(); i++){
            if (pose_deque[i].ros_time_stamp > pointTime)
            {
                double k_time = (pointTime - pose_deque[i-1].ros_time_stamp)
                                /(pose_deque[i].ros_time_stamp - pose_deque[i-1].ros_time_stamp);

                //平移插值
                Eigen::Vector3d t_w_b_lidar_now;
                t_w_b_lidar_now = pose_deque[i - 1].pose.GetT() +
                                    k_time * (pose_deque[i].pose.GetT() - pose_deque[i - 1].pose.GetT());

                //旋转插值
                Eigen::Quaterniond q_w_b_lidar_now;
                q_w_b_lidar_now = pose_deque[i-1].pose.GetQ().slerp(k_time,pose_deque[i].pose.GetQ());

                //插值位姿态矩阵
                T_w_b_lidar_now  = PoseT(t_w_b_lidar_now , q_w_b_lidar_now);

            }
            continue;
        }
    }

    PointType deskewPoint(PointType *point, double relTime, const PoseT& T_w_b_lidar_start,const CloudWithTime& cloudinfo, const std::deque<PoseWithTime>& pose_deque) {  //激光点去畸变

        double pointTime = cloudinfo.min_ros_timestamp + relTime;//scan时间加相对时间，获得点的时间

        PoseT T_w_b_lidar_now;
        findRotation(pointTime, cloudinfo,pose_deque,T_w_b_lidar_now);

        Eigen::Vector3d origin_pt;
        origin_pt << point->x,point->y,point->z;
        PoseT T_lidar_start_now = T_w_b_lidar_start.inverse()*T_w_b_lidar_now;
        Eigen::Vector3d deskewnewpoint = PoseT(T_lidar_start_now.inverse())*origin_pt;

        PointType newPoint;//get deskewed point
        newPoint.x = deskewnewpoint.x();
        newPoint.y = deskewnewpoint.y();
        newPoint.z = deskewnewpoint.z();
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    double cloudExtraction() {

        TicToc timer;

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
                    extractedCloud->push_back(deskewCloud_body->points[j + i * SensorConfig::Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;//
        }
        return timer.toc();
    }

    void publishClouds() {

        EZLOG(INFO)<<"**********************publishClouds "<<std::endl;

        cloudInfo.header = cloudHeader;
        cloudInfo.header.frame_id = "map";
        cloudInfo.cloud_deskewed = publishCloud(pubextractedCloud, extractedCloud, cloudHeader.stamp, "map");
        EZLOG(INFO)<<"**********************extractedCloud->size() "<<extractedCloud->size()<<std::endl;
        pubLaserCloudInfo.publish(cloudInfo);

//        publishCloud(pub_deskew_cloud_world, deskewCloud_body, cloudHeader.stamp, SensorConfig::lidarFrame);
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
    std::thread img_proj_thread(&ImageProjection::do_work, &IP);

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
