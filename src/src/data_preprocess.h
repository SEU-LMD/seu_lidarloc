
// Use the Velodyne point format as a common representation
#ifndef SEU_LIDARLOC_IMGPROJECTION_H
#define SEU_LIDARLOC_IMGPROJECTION_H
#include <mutex>
#include <thread>

#include "pubsub/pubusb.h"
//#include "featureExtraction.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "utils/MapSaver.h"
#include "utils/timer.h"
#include "utils/utility.h"
#include "opencv2/opencv.hpp"   // for opencv4
//#include <opencv/cv.h>
#include "pcl/features/normal_3d_omp.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/common/pca.h"
#include <fstream>

#include "utils/udp_thread.h"

#include "udp_helper.h/udp_seralize.h"
#include "udp_helper.h/udp_client.h"

class CloudWithTime{
public:
    CloudTypeXYZIRTPtr cloud_ptr;
    double max_ros_timestamp;
    double min_ros_timestamp;
    double min_latency_timestamp;
};

class DataPreprocess {
public:
    PubSubInterface* pubsub;
    std::mutex cloud_mutex;
    std::mutex work_mutex;
    std::mutex gnssins_mutex;
    std::mutex imuodom_mutex;

    std::thread* do_work_thread;

    std::deque<CloudTypeXYZIRTPtr> deque_cloud;
    std::deque<OdometryType> poseQueue;
    std::deque<OdometryType> IMUOdomQueue;

//    FeatureExtraction* ft_extr_ptr;
//    OPTMapping* opt_mapping_ptr;
//    imu_wheel_dr* imu_pre_ptr;
//    OPTMapping* opt_mapping_ptr;
    std::function<void(const CloudInfo&)> Function_AddCloudInfoToFeatureExtraction;
    std::function<void(const GNSSOdometryType&)> Function_AddGNSSOdometryTypeToFuse;
    std::function<void(const GNSSOdometryType&)>Function_AddGNSSOdometryTypeToOPTMapping;
    std::function<void(const GNSSOdometryType&)>Function_AddFirstGNSSPoint2DR;

   // std::function<void(const OdometryType&)> Function_AddOdometryTypeToOPTMapping;

    bool init = false;
    GeographicLib::LocalCartesian geoConverter;
    int frame_id = 0;

    std::string topic_origin_cloud_world = "/origin_cloud_world";
    std::string topic_deskew_cloud_world = "/deskew_cloud_world";
    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_deskw_cloud_to_ft_world = "/deskw_cloud_to_ft_world";
    std::string topic_imuodom_curlidartime_world = "/imuodom_curlidartime_world";

    std::string topic_ground_world = "/ground_world";
    std::string topic_unground_world = "/unground_world";

    std::string topic_cloud_pillar_world = "/cloud_pillar_world";
    std::string topic_cloud_beam_world = "/cloud_beam_world";
    std::string topic_cloud_facade_world = "/cloud_facade_world";
    std::string topic_cloud_roof_world = "/cloud_roof_world";

    pcl::PointCloud<PointType>::Ptr deskewCloud_body;//去畸变之后的全部点云
    cv::Mat rangeMat;
    int lidarScan_cnt =0;
    double cloud_min_ros_timestamp;
    double cloud_max_ros_timestamp;


    std::shared_ptr<UDP_THREAD> udp_thread;//udp


    void  AllocateMemory() {
        deskewCloud_body.reset(new pcl::PointCloud<PointType>());
        deskewCloud_body->points.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);//将fullCloud的点集大小初始化为N_SCAN * Horizon_SCAN
        rangeMat = cv::Mat(SensorConfig::N_SCAN, SensorConfig::Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    }

    void ResetParameters(){
//        deskewCloud_body.reset();
        rangeMat =  cv::Scalar::all(FLT_MAX);
    }

    double GetCloudTime(CloudTypeXYZIRTPtr cloud_origin, CloudWithTime& cloud_with_time){
        TicToc timer;

        const double& lidar_start_ros_time = cloud_origin->timestamp;
        //find the min and max timestamp in one scan and get the delta time stamp
        double min_timestamp = 10000000;
        double max_timestamp = -100000000;
        double t_0 = cloud_origin->cloud.points[0].latency;
        for(int i=0; i < cloud_origin->cloud.points.size(); ++i)
        {
            cloud_origin->cloud.points[i].latency = cloud_origin->cloud.points[i].latency - t_0;

            if(cloud_origin->cloud.points[i].latency > max_timestamp){
                max_timestamp = cloud_origin->cloud.points[i].latency;

            }
            if(cloud_origin->cloud.points[i].latency < min_timestamp){
                min_timestamp = cloud_origin->cloud.points[i].latency;
            }
        }

        cloud_with_time.cloud_ptr = cloud_origin;
        cloud_with_time.min_ros_timestamp = lidar_start_ros_time;
        cloud_with_time.max_ros_timestamp = max_timestamp - min_timestamp + cloud_with_time.min_ros_timestamp;
        cloud_with_time.min_latency_timestamp = min_timestamp;

        double cost_time = timer.toc();
//        EZLOG(INFO)<<"GetCloudTime cost time(ms) = "<<cost_time<<std::endl;
        return cost_time;
    }

    double  FindIMUOdomPose(const CloudWithTime& cloudinfo, const std::deque<OdometryType>& pose_deque ,
                               PoseT& T_w_l_lidar_start) {
        TicToc timer;

        pcl::PointCloud<PointType> originCloud_w;

        for (int i = 0; i < (int) pose_deque.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

            if (pose_deque[i].timestamp > cloudinfo.min_ros_timestamp){
                double ktime = (cloudinfo.min_ros_timestamp - pose_deque[i - 1].timestamp) / (pose_deque[i].timestamp - pose_deque[i - 1].timestamp);
                //平移插值
                Eigen::Vector3d t_w_b_lidar_start;
                t_w_b_lidar_start = pose_deque[i - 1].pose.GetXYZ() +
                                    ktime * (pose_deque[i].pose.GetXYZ() - pose_deque[i - 1].pose.GetXYZ());

              //  EZLOG(INFO)<<"t_w_b_lidar_start"<<t_w_b_lidar_start[0]<<t_w_b_lidar_start[1]<<t_w_b_lidar_start[2];
                //旋转插值
                Eigen::Quaterniond q_w_b_lidar_start;//旋转
                q_w_b_lidar_start = pose_deque[i-1].pose.GetQ().slerp(ktime,pose_deque[i].pose.GetQ());

                //插值位姿态矩阵
                T_w_l_lidar_start  = PoseT (t_w_b_lidar_start , q_w_b_lidar_start);

                //pusblish world origin cloud
                //for debug use
                if(MappingConfig::if_debug)
                {
                    CloudTypeXYZIRT cur_scan_cloud_w;
                    cur_scan_cloud_w.timestamp = cloudinfo.cloud_ptr->timestamp;
                    cur_scan_cloud_w.frame = "map";
                    pcl::transformPointCloud(cloudinfo.cloud_ptr->cloud, cur_scan_cloud_w.cloud, (T_w_l_lidar_start).pose.cast<float>());
                    pubsub->PublishCloud(topic_origin_cloud_world, cur_scan_cloud_w);
                }
                break;
            }
            continue;
        }
        return timer.toc();
    }//end FindLidarFirstPose

    double FindLidarFirstPose(const CloudWithTime& cloudinfo,const std::deque<OdometryType>& odom_deque,
                           PoseT& imuodom_curlidartime){

        for (int i = 0; i < (int) odom_deque.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

            if (odom_deque[i].timestamp > cloudinfo.min_ros_timestamp){
                double ktime = (cloudinfo.min_ros_timestamp - odom_deque[i - 1].timestamp) / (odom_deque[i].timestamp - odom_deque[i - 1].timestamp);
                //平移插值
                Eigen::Vector3d t_imuodom;
                t_imuodom = odom_deque[i - 1].pose.GetXYZ() +
                                    ktime * (odom_deque[i].pose.GetXYZ() - odom_deque[i - 1].pose.GetXYZ());

                //旋转插值
                Eigen::Quaterniond q_imuodom;//旋转
                q_imuodom = odom_deque[i-1].pose.GetQ().slerp(ktime,odom_deque[i].pose.GetQ());

                //插值位姿态矩阵
                imuodom_curlidartime  = PoseT (t_imuodom , q_imuodom);

                break;
            }
            continue;
        }
    }

    //find x y z in lidar coordinate,Q in gnss coordinate at the pointtime and calculate translation matrix
    void FindRotation(const double pointTime, const CloudWithTime& cloudinfo, const std::deque<OdometryType>& pose_deque,
                      PoseT& T_w_b_lidar_now) {//IMU数据中找到与当前点对应时刻的变换矩阵

        for(std::deque<OdometryType>::const_iterator it = pose_deque.begin();it!=pose_deque.end();it++){
            if (it->timestamp > pointTime)
            {

//                double k_time = (pointTime - pose_deque[i-1].timestamp)
//                                /(pose_deque[i].timestamp - pose_deque[i-1].timestamp);
//
//                //平移插值
//                Eigen::Vector3d t_w_b_lidar_now;
//                t_w_b_lidar_now = pose_deque[i - 1].pose.GetXYZ() +
//                                  k_time * (pose_deque[i].pose.GetXYZ() - pose_deque[i - 1].pose.GetXYZ());
//
//                //旋转插值
//                Eigen::Quaterniond q_w_b_lidar_now;
//                q_w_b_lidar_now = pose_deque[i-1].pose.GetQ().slerp(k_time,pose_deque[i].pose.GetQ());
//
//                //插值位姿态矩阵
//                T_w_b_lidar_now  = PoseT(t_w_b_lidar_now , q_w_b_lidar_now);

//              不做插值，直接赋值
                Eigen::Vector3d t_w_b_lidar_now;
                t_w_b_lidar_now = it->pose.GetXYZ();

                //旋转插值
                Eigen::Quaterniond q_w_b_lidar_now;
                q_w_b_lidar_now = it->pose.GetQ();

                //插值位姿态矩阵
                T_w_b_lidar_now  = PoseT(t_w_b_lidar_now , q_w_b_lidar_now);

                break;
            }
            continue;
        }
    }

    PointType DeskewPoint(PointType *point,
                          double relTime,
                          const PoseT& T_w_b_lidar_start,
                          const CloudWithTime& cloudinfo,
                          const std::deque<OdometryType>& pose_deque) {  //激光点去畸变

        double pointTime = cloudinfo.min_ros_timestamp + relTime;//scan时间加相对时间，获得点的时间

        PoseT T_w_b_lidar_now;
        FindRotation(pointTime, cloudinfo, pose_deque,
                     T_w_b_lidar_now);

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

    //update rangemat and deskewCloud_body
    double ProjectPointCloud(const CloudWithTime& cloudinfo, const std::deque<OdometryType>& pose_deque, const PoseT& T_w_b_lidar_start) {

        TicToc timer;

        int cloudSize = cloudinfo.cloud_ptr->cloud.points.size(); //获取原始点云大小
        pcl::PointCloud<PointType> deskewCloud_body_offset;
        // range image projection
        int valid_num = 0;
        int range_outlier = 0;
        int row_outlier = 0;
        int col_outlier = 0;
        int rangemat_outlier = 0;
        for (int i = 0; i < cloudSize; ++i) {  //遍历每一个点
//          前提，去除线数不合适的点
            int rowIdn = cloudinfo.cloud_ptr->cloud.points[i].ring;//获取行数
            if (rowIdn < SensorConfig::lidarMinRing || rowIdn >= SensorConfig::lidarMaxRing){
                row_outlier++;
                continue;
            }

            PointType thisPoint;
            thisPoint.x = cloudinfo.cloud_ptr->cloud.points[i].x;//获取xyz、强度
            thisPoint.y = cloudinfo.cloud_ptr->cloud.points[i].y;
            thisPoint.z = cloudinfo.cloud_ptr->cloud.points[i].z;
            thisPoint.intensity = cloudinfo.cloud_ptr->cloud.points[i].intensity;
            float range = pointDistance(thisPoint); //计算点的距离

            if (range < SensorConfig::lidarMinRange || range > SensorConfig::lidarMaxRange){
                range_outlier++;
                continue;
            }



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
            //如果重复 那么只保存地一个点
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX){
                rangemat_outlier++;
                continue;
            }
            //very important function@!!!!!!!!!
            if (SensorConfig::use_gnss_deskew){
                if(cloudinfo.cloud_ptr->cloud.points[i].latency - cloudinfo.min_latency_timestamp < 0){
                    EZLOG(INFO)<<"wrong! latency!";
                    continue;
                }
                thisPoint = DeskewPoint(&thisPoint,
                                cloudinfo.cloud_ptr->cloud.points[i].latency - cloudinfo.min_latency_timestamp,
                                T_w_b_lidar_start,cloudinfo,
                                pose_deque);//进行点云去畸变
//                EZLOG(INFO)<<"DeskewPoint with GNSS time: "<<t_deskew.toc();
            }
//            thisPoint = deskewPoint(&thisPoint, cloudinfo.cloud->points[i].latency - cloudinfo.min_latency_timestamp, T_w_b_lidar_start,cloudinfo,pose_deque);//进行点云去畸变
//            rangeMat(rowIdn, columnIdn) = range;//将去畸变后的点范围和坐标存入rangeMat
            rangeMat.at<float>(rowIdn, columnIdn) = range;//将去畸变后的点范围和坐标存入rangeMat
            int index = columnIdn + rowIdn * SensorConfig::Horizon_SCAN;//计算索引index,将去畸变后的点存入fullCloud
            deskewCloud_body->points[index] = thisPoint;

            //just for show
            auto thisPoint_offset = thisPoint;
            thisPoint_offset.z = thisPoint_offset.z + 20.0;
            deskewCloud_body_offset.points.push_back(thisPoint_offset);
            valid_num++;
        }//end for

////        use for debug
//        EZLOG(INFO)<<"valid_num num = "<<valid_num<<std::endl;
//        EZLOG(INFO)<<"range_outlier num = "<<range_outlier<<std::endl;
//        EZLOG(INFO)<<"row_outlier num = "<<row_outlier<<std::endl;
//        EZLOG(INFO)<<"col_outlier num = "<<col_outlier<<std::endl;
//        EZLOG(INFO)<<"rangemat_outlier num = "<<rangemat_outlier<<std::endl;

////        for debug use
        if(MappingConfig::if_debug)
        {
            CloudTypeXYZI cloud_pub;
            cloud_pub.timestamp = cloudinfo.cloud_ptr->timestamp;
            cloud_pub.frame = "map";
            pcl::transformPointCloud(deskewCloud_body_offset, cloud_pub.cloud, (T_w_b_lidar_start).pose.cast<float>());

            pubsub->PublishCloud(topic_deskew_cloud_world, cloud_pub);
            //        EZLOG(INFO)<<"deskewCloud_body size = "<<deskewCloud_body->points.size()<<std::endl;
//        EZLOG(INFO)<<"deskewCloud_body_offset size = "<<deskewCloud_body_offset.points.size()<<std::endl;
//        EZLOG(INFO)<<"deskewCloud_w size = "<<deskewCloud_w.points.size()<<std::endl;
        }

        return timer.toc();
    }


    double CloudExtraction(const PoseT& T_w_l, CloudInfo& cloudInfo) {

        TicToc timer;
        pcl::PointCloud<PointXYZICOLRANGE>::Ptr   extractedCloud(new pcl::PointCloud<PointXYZICOLRANGE>());
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < SensorConfig::N_SCAN; ++i) {
            //提取特征的时候，每一行的前5个和最后5个不考虑
            cloudInfo.startRingIndex[i] = count - 1 + 5;//从第六个点开始找（前面5个不考虑）

            for (int j = 0; j < SensorConfig::Horizon_SCAN; ++j) {
                if (rangeMat.at<float>(i, j) != FLT_MAX) {
                    // mark the points' column index for marking occlusion later
                    //记录激光点对应的Horizon_SCAN方向上的索引
                    PointXYZICOLRANGE pt_info;
                    pt_info.col = j;
                    pt_info.range = rangeMat.at<float>(i, j);
                    auto& pt_tmp = deskewCloud_body->points[j + i * SensorConfig::Horizon_SCAN];
                    pt_info.x = pt_tmp.x;
                    pt_info.y = pt_tmp.y;
                    pt_info.z = pt_tmp.z;
                    pt_info.intensity = pt_tmp.intensity;

//                    cloudInfo.pointColInd[count] = j;
//                    // save range info  激光点距离
//                    cloudInfo.pointRange[count] = rangeMat(i, j);
                    // save extracted cloud 加入有效激光点
                    extractedCloud->push_back(pt_info);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;//
        }
        cloudInfo.cloud_ptr = extractedCloud;

////        //for debug use
//        if(MappingConfig::if_debug)
//        {
//            CloudTypeXYZICOLRANGE cloud_pub;
//            cloud_pub.timestamp = cloudInfo.timestamp;
//            cloud_pub.frame = "map";
//            pcl::transformPointCloud(*extractedCloud, cloud_pub.cloud, T_w_l.pose.cast<float>());
//            pubsub->PublishCloud(topic_deskw_cloud_to_ft_world, cloud_pub);
//        }
        return timer.toc();
    }

    void DoWork() {
        CloudInfo cloudinfo;
        cloudinfo.startRingIndex.assign(SensorConfig::N_SCAN,
                                        0);//为cloudInfo中的startRingIndex、endRingIndex向量分配大小N_SCAN,并初始化为0
        cloudinfo.endRingIndex.assign(SensorConfig::N_SCAN, 0);

        while (1) {
            //  EZLOG(INFO)<<deque_cloud.size()<<endl;
            bool isEmpty = false;
            {
                std::lock_guard<std::mutex> lock(work_mutex);
                isEmpty = deque_cloud.empty();
            }
            if(!isEmpty){
//                EZLOG(INFO)<<"DoWork"<<endl;
                ///0.do something

                std::deque<OdometryType> imuodom_copy;
                imuodom_mutex.lock();
                imuodom_copy = IMUOdomQueue;
                imuodom_mutex.unlock();
                double imuodo_min_ros_time =  imuodom_copy.front().timestamp;
                double imuodo_max_ros_time =  imuodom_copy.back().timestamp;
               // EZLOG(INFO)<<"imuodo_min_ros_time"<<imuodo_max_ros_time - imuodo_min_ros_time<<endl;

//                if(!poseQueue.empty()){
//                    std::deque<OdometryType> odo_poses_copy;
//                    gnssins_mutex.lock();
//                    odo_poses_copy = poseQueue;
//                    gnssins_mutex.unlock();
//                    TicToc t_imgProj;
//
//                    double odo_min_ros_time =  odo_poses_copy.front().timestamp;
//                    double odo_max_ros_time =  odo_poses_copy.back().timestamp;
//
//                    CloudTypeXYZIRTPtr cur_gnss;
//                    cloud_mutex.lock();
//                    cur_gnss = deque_cloud.front();
//
//
//                    CloudWithTime cloud_with_time;
//                    GetCloudTime(cur_gnss , cloud_with_time);
//                    double cloud_min_ros_timestamp = cloud_with_time.min_ros_timestamp;
//                    double cloud_max_ros_timestamp = cloud_with_time.max_ros_timestamp;
//
//
//
//                    if(!poseQueue.empty() && odo_min_ros_time >= cloud_min_ros_timestamp){
//                        auto temp = poseQueue.front();
//                        temp.timestamp = cloud_min_ros_timestamp - 0.01f;
//                        poseQueue.push_front(temp);
//                    }
//
//                    if(odo_min_ros_time<cloud_min_ros_timestamp && odo_max_ros_time>cloud_min_ros_timestamp){
//                        cloud_mutex.unlock();
//                        PoseT odom_curlidartime;
//                        double cost_time_findimupose = FindLidarFirstPose(cloud_with_time, odo_poses_copy,//in
//                                                                       odom_curlidartime);//out
////                        EZLOG(INFO) << "FindIMUOdomPose cost time(ms) = " << cost_time_findimupose << std::endl;
//
//                        OdometryType Odometry_imuodom_curlidartime_pub;
//                        Odometry_imuodom_curlidartime_pub.timestamp = cloud_min_ros_timestamp;
//                        Odometry_imuodom_curlidartime_pub.frame = "map";
//                        Odometry_imuodom_curlidartime_pub.pose = odom_curlidartime;
//
//                        Eigen::Vector3d t_w_gnss;
//                        Eigen::Quaterniond q_w_gnss;
//                        Eigen::Matrix3d q_w_gnss_matrix;
//                        t_w_gnss = odom_curlidartime.GetXYZ();
//                        q_w_gnss = odom_curlidartime.GetQ();
//                        q_w_gnss.normalize();
//                        EZLOG(INFO)<<"t_w_cur"<<t_w_gnss.x()
//                                   <<t_w_gnss.y()
//                                   <<t_w_gnss.z()<<endl;
//
//                        EZLOG(INFO)<<"t_w_cur"<<q_w_gnss.x()
//                                   <<q_w_gnss.y()
//                                   <<q_w_gnss.z()
//                                   <<q_w_gnss.w()<<endl;
//                        //pubsub->PublishOdometry(topic_imuodom_curlidartime_world, Odometry_imuodom_curlidartime_pub);
//
//                        gnssins_mutex.lock();
//                        while(poseQueue.front().timestamp < cloud_max_ros_timestamp - 0.1){
//                            poseQueue.pop_front();
//                        }
//                        gnssins_mutex.unlock();
//                    }else{
//                        cloud_mutex.unlock();
//                    }
//                }

                ///1.get cloud max and min time stamp to
                CloudTypeXYZIRTPtr cur_scan;
                cloud_mutex.lock();
                cur_scan = deque_cloud.front();

                CloudWithTime cloud_with_time;
                GetCloudTime(cur_scan , cloud_with_time);
                double cloud_min_ros_timestamp = cloud_with_time.min_ros_timestamp;
                double cloud_max_ros_timestamp = cloud_with_time.max_ros_timestamp;

                //              gnss lidar gnss---->>>>
                gnssins_mutex.lock();
                while(!poseQueue.empty()){
                    if(poseQueue.front().timestamp > cur_scan->timestamp){
                        cloudinfo.pose = poseQueue.front().pose;
                        poseQueue.clear();
                        break;
                    }
                    else{
                        poseQueue.pop_front();
                    }
                }
                gnssins_mutex.unlock();
                ///2.
//                if(odo_min_ros_time>cloud_min_ros_timestamp){
//                    EZLOG(INFO)<<"odo is larger than lidar" <<endl;
//                    exit(1);
//                }
                if(!IMUOdomQueue.empty() && imuodo_min_ros_time >= cloud_min_ros_timestamp){
                    auto temp = IMUOdomQueue.front();
                    temp.timestamp = cloud_min_ros_timestamp - 0.01f;
                    IMUOdomQueue.push_front(temp);
                }
               // double cur_lidar_time = deque_cloud.front()->timestamp;
               // EZLOG(INFO)<<"TIMESTAMP"<<cur_lidar_time-cloud_min_ros_timestamp<<endl;
                if(imuodo_min_ros_time<cloud_min_ros_timestamp&&imuodo_max_ros_time>cloud_max_ros_timestamp){//odo_min_ros_time<cloud_min_ros_timestamp&&
                   // EZLOG(INFO)<<"get in odo_min_ros_time"<<endl;
//                        if(odo_min_ros_time >= cloud_min_ros_timestamp){
//                            auto temp = poseQueue.front();
//                            temp.timestamp = cloud_min_ros_timestamp - 0.01f;
//                            poseQueue.push_front(temp);
//                        }
                    deque_cloud.pop_front();
                    cloud_mutex.unlock();

                    cloudinfo.frame_id = frame_id++;
                    cloudinfo.timestamp = cur_scan->timestamp;

                    /// 1.FindLidarFirstPose
                    PoseT T_w_l_lidar_first_pose;
                    double cost_time_findpose = FindIMUOdomPose(cloud_with_time, imuodom_copy,//in
                                                                   T_w_l_lidar_first_pose);//out

                    cloudinfo.DRPose = T_w_l_lidar_first_pose;
                    Eigen::Vector3d t_w_cur;
                    Eigen::Quaterniond q_w_cur;
                    Eigen::Matrix3d q_w_cur_matrix;
                    t_w_cur = T_w_l_lidar_first_pose.GetXYZ();
                    q_w_cur = T_w_l_lidar_first_pose.GetQ();
                    q_w_cur.normalize();
                   // EZLOG(INFO)<<"t_w_cur"<<t_w_cur.x()
                     //                     <<t_w_cur.y()
                     //                     <<t_w_cur.z()<<endl;

                   // EZLOG(INFO)<<"t_w_cur"<<q_w_cur.x()
                    //           <<q_w_cur.y()
                    //           <<q_w_cur.z()
                    //           <<q_w_cur.w()<<endl;
                  //  EZLOG(INFO) << "FindLidarFirstPose cost time(ms) = " << cost_time_findpose << std::endl;

                    ///2.imgprojection
                    ///update rangemat and deskewCloud_body
                    double cost_time_projpc = ProjectPointCloud(cloud_with_time, imuodom_copy, T_w_l_lidar_first_pose);
                  //  EZLOG(INFO) << "cost_time_projpc(ms) = " << cost_time_projpc << std::endl;

                    ///3.cloudExtraction
                    double cost_time_cloudextraction = CloudExtraction(T_w_l_lidar_first_pose, cloudinfo);
                  //  EZLOG(INFO)<<"cost_time_cloudextraction(ms) = "<<cost_time_cloudextraction<<std::endl;

//                    ///ground filter
//                  int gf_grid_pt_num_thre = 8;float gf_grid_resolution = 1.5;float gf_max_grid_height_diff; float gf_neighbor_height_diff = 1.5;
//                        float gf_max_ground_height;int gf_down_rate_ground = 15;int gf_down_down_rate_ground = 2;
//                        int gf_downsample_rate_nonground = 1;int gf_reliable_neighbor_grid_thre = 0;int estimate_ground_normal_method;float normal_estimation_radius = 2.0;
//                        int distance_inverse_sampling_method = 0;float standard_distance = 15.0;bool fixed_num_downsampling = false;int ground_down_fixed_num = 500;bool extract_curb_or_not = false;
//                        float intensity_thre = FLT_MAX;bool apply_scanner_filter = false;
//
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_unground (new pcl::PointCloud<PointXYZICOLRANGE>);
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_curb (new pcl::PointCloud<PointXYZICOLRANGE>);
//
//                    TicToc time_gf;
//
//                    EZLOG(INFO)<<"cloudinfo.cloud_ptr->points.size() = "<<cloudinfo.cloud_ptr->points.size();
//
//                    fast_ground_filter(cloudinfo.cloud_ptr,
//                                       cloud_ground,
//                                       cloud_ground_down,
//                                       cloud_unground,
//                                       cloud_curb,
//                                       gf_grid_pt_num_thre, gf_max_ground_height,gf_grid_resolution,
//                                       distance_inverse_sampling_method, standard_distance,gf_downsample_rate_nonground,intensity_thre
//
////                                           gf_max_grid_height_diff,
////						     gf_down_down_rate_ground,
////						     estimate_ground_normal_method, normal_estimation_radius,
////						    fixed_num_downsampling, ground_down_fixed_num, extract_curb_or_not,
////						    apply_scanner_filter
//                    );
//                    double time_ground_filter = time_gf.toc();
//                    EZLOG(INFO)<<"time_ground_filter = "<<time_ground_filter<<endl;
//
//                    if(MappingConfig::if_debug)
//                    {
//                        CloudTypeXYZICOLRANGE ground_pub,unground_pub;
//                        ground_pub.timestamp = cloudinfo.timestamp;
//                        ground_pub.frame = "map";
//                        pcl::transformPointCloud(*cloud_ground, ground_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                        pubsub->PublishCloud(topic_ground_world, ground_pub);
//                        unground_pub.timestamp = cloudinfo.timestamp;
//                        unground_pub.frame = "map";
//                        pcl::transformPointCloud(*cloud_unground, unground_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                        pubsub->PublishCloud(topic_unground_world, unground_pub);
//                    }
//
//                    if(0)
//                    {
//
//                        ///classify_nground_pts
//
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down (new pcl::PointCloud<PointXYZICOLRANGE>);
//                        pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex (new pcl::PointCloud<PointXYZICOLRANGE>);
//
//                        float pca_neighbor_radius = 1.0;int pca_neighbor_k = 30 ;int pca_neighbor_k_min = 8;int pca_down_rate = 1;
//                        float edge_thre = 0.65 ;float planar_thre = 0.65 ; float edge_thre_down = 0.75 ; float planar_thre_down = 0.75;
//                        int extract_vertex_points_method = 2;float curvature_thre = 0.12;float vertex_curvature_non_max_r = 1.5 * pca_neighbor_radius;
//                        float linear_vertical_sin_high_thre = 0.94;float linear_vertical_sin_low_thre = 0.17;
//                        float planar_vertical_sin_high_thre = 0.98; float planar_vertical_sin_low_thre = 0.34;
//
//                        EZLOG(INFO)<<"cloud_unground->points.size() =  "<<cloud_unground->points.size()<<endl;
//
//                        TicToc time_classify_nground_pts;
//
//                        classify_nground_pts(cloud_unground,cloud_pillar,cloud_beam,cloud_facade,cloud_roof,
//                                             cloud_pillar_down,cloud_beam_down,cloud_facade_down,cloud_roof_down,cloud_vertex,
//                                             pca_neighbor_radius, pca_neighbor_k, pca_neighbor_k_min, pca_down_rate,
//                                             edge_thre, planar_thre, edge_thre_down, planar_thre_down,
//                                             extract_vertex_points_method, curvature_thre, vertex_curvature_non_max_r,
//                                             linear_vertical_sin_high_thre, linear_vertical_sin_low_thre,
//                                             planar_vertical_sin_high_thre, planar_vertical_sin_low_thre
////                                         fixed_num_downsampling, pillar_down_fixed_num, facade_down_fixed_num,
////                                         beam_down_fixed_num, roof_down_fixed_num, unground_down_fixed_num,
////                                         beam_height_max, roof_height_min, feature_pts_ratio_guess,
////                                         sharpen_with_nms_on, use_distance_adaptive_pca
//                        );
//                        EZLOG(INFO)<<"time_classify_nground_pts.toc() =  "<<time_classify_nground_pts.toc()<<endl;
//
//                        EZLOG(INFO)<<"cloud_pillar->points.size() =  "<<cloud_pillar->points.size()<<endl;
//                        EZLOG(INFO)<<"cloud_beam->points.size() =  "<<cloud_beam->points.size()<<endl;
//                        EZLOG(INFO)<<"cloud_facade->points.size() =  "<<cloud_facade->points.size()<<endl;
//                        EZLOG(INFO)<<"cloud_roof->points.size() =  "<<cloud_roof->points.size()<<endl;
//
//                        if(MappingConfig::if_debug)
//                        {
//
//                            CloudTypeXYZICOLRANGE cloud_pillar_pub,cloud_beam_pub,cloud_facade_pub,cloud_roof_pub;
//                            cloud_pillar_pub.timestamp = cloudinfo.timestamp;
//                            cloud_beam_pub.timestamp = cloudinfo.timestamp;
//                            cloud_facade_pub.timestamp = cloudinfo.timestamp;
//                            cloud_roof_pub.timestamp = cloudinfo.timestamp;
//                            cloud_pillar_pub.frame = "map";
//                            cloud_beam_pub.frame = "map";
//                            cloud_facade_pub.frame = "map";
//                            cloud_roof_pub.frame = "map";
//                            pcl::transformPointCloud(*cloud_pillar, cloud_pillar_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                            pcl::transformPointCloud(*cloud_beam, cloud_beam_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                            pcl::transformPointCloud(*cloud_facade, cloud_facade_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                            pcl::transformPointCloud(*cloud_roof, cloud_roof_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
//                            pubsub->PublishCloud(topic_cloud_pillar_world, cloud_pillar_pub);
//                            pubsub->PublishCloud(topic_cloud_beam_world, cloud_beam_pub);
//                            pubsub->PublishCloud(topic_cloud_facade_world, cloud_facade_pub);
//                            pubsub->PublishCloud(topic_cloud_roof_world, cloud_roof_pub);
//
//                        }
//
//                    }




                    ///4. send data to feature extraction node
//                        ft_extr_ptr->AddCloudData(cloudinfo);
                    Function_AddCloudInfoToFeatureExtraction(cloudinfo);
                    EZLOG(INFO)<<"2. 1 of 2 data_preprocess send to feature_extraction! current lidar pointCloud size is: "<<cloudinfo.cloud_ptr->points.size();
//                        EZLOG(INFO)<<"cloudinfo.frame_id = "<<cloudinfo.frame_id<<std::endl;
                 //   EZLOG(INFO)<<"cloudinfo.cloud_ptr->size() = "<<cloudinfo.cloud_ptr->size()<<std::endl;


                    ///5.pop used odom
                    imuodom_mutex.lock();
//                        while(poseQueue.front().timestamp < cloud_max_ros_timestamp - 0.1f){
//                            poseQueue.pop_front();
//                        }
                    double thresh = cloud_max_ros_timestamp - 0.05f;
                    while(IMUOdomQueue.front().timestamp < thresh){
                        IMUOdomQueue.pop_front();
                    }
                    imuodom_mutex.unlock();

                    ResetParameters();
                }//end function if
                else{
                    cloud_mutex.unlock();
                }

                ResetParameters();
            }//end if(deque_cloud.size()!=0){
            else{
                sleep(0.01);//线程休息10ms
            }

        }
    }

    void AddCloudData(const CloudTypeXYZIRT& data){
        if((lidarScan_cnt > SensorConfig::lidarScanDownSample && MappingConfig::slam_mode_switch == 1)
            ||MappingConfig::slam_mode_switch == 0){
            //        CloudTypeXYZIRTPtr cloud_ptr = make_shared<CloudTypeXYZIRT>();
            CloudTypeXYZIRTPtr cloud_ptr(new CloudTypeXYZIRT());

            *cloud_ptr = data;//深拷贝
            cloud_mutex.lock();
            deque_cloud.push_back(cloud_ptr);
            cloud_mutex.unlock();
            lidarScan_cnt =0;
        }
        else{
            lidarScan_cnt++;
        }

    }



    void Udp_OdomPub(const PoseT& data){
        Vis_Odometry odom_out;
        std::string fu_str;
        odom_out.type = "gn";
        odom_out.t[0]= data.GetXYZ().x();
        odom_out.t[1]= data.GetXYZ().y();
        odom_out.t[2]= data.GetXYZ().z();


        odom_out.q.x() = data.GetQ().x();
        odom_out.q.y() = data.GetQ().y();
        odom_out.q.z() = data.GetQ().z();
        odom_out.q.w() = data.GetQ().w();

        fu_str = odom_out.ToString();
        udp_thread -> SendUdpMSg(fu_str);
    }

    void AddGNSSINSSData(const GNSSINSType& data){

        if(!init)
        {
            double x,y,z;
            if(MappingConfig::slam_mode_switch){
                std::ifstream downfile(MappingConfig::save_map_path+"Origin.txt");  //打开文件
                std::string line; //字符串
                std::getline(downfile, line);//
                std::istringstream iss(line);
                iss >> x >> y >> z;
                downfile.close(); // 关闭文件
                geoConverter.Reset(x, y, z);
            }
            else{
                geoConverter.Reset(data.lla[0], data.lla[1], data.lla[2]);
            }
            init = true;
            MapSaver::SaveOriginLLA(data.lla);
            return;
        }

//        if(!init)
//        {
//            geoConverter.Reset(data.lla[0], data.lla[1], data.lla[2]);
//            init = true;
//            MapSaver::SaveOriginLLA(data.lla);
//            return;
//        }

        double t_enu[3];
        geoConverter.Forward(data.lla[0], data.lla[1], data.lla[2],
                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate

        Eigen::Matrix3d z_matrix;//calculate Quaternion
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = -data.yaw * 3.1415926535 / 180.0;
        double pitch_Y = data.pitch * 3.1415926535 / 180.0;
        double roll_X = data.roll * 3.1415926535 / 180.0;
        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                sin(heading_Z), cos(heading_Z),  0,
                0,                 0,            1;

        x_matrix << 1,                 0,              0,
                0,            cos(pitch_Y),     -sin(pitch_Y),
                0,            sin(pitch_Y),    cos(pitch_Y);

        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                0,                 1,               0 ,
                -sin(roll_X),      0,         cos(roll_X);

        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb
        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);
        Eigen::Quaterniond q_w_b(R_w_b);//获得局部坐标系的四元数
        PoseT T_w_b(t_w_b, R_w_b);
//        world is GNSS ,base is car, T_w_l =
        PoseT T_w_l = PoseT(T_w_b.pose*(SensorConfig::T_L_B.inverse())); // Pw = Twb * Tbl * Pl
//        T_w_l:
//        -0.973379  -0.229171 0.00382091   -128.353
//        0.2292  -0.973157  0.0207947 0.00482607
//        -0.0010472  0.0211169   0.999776  0.0740949
//        0          0          0          1
//        T_w_b:
//        0.229171   -0.973379  0.00382091    -128.355
//        0.973157      0.2292   0.0207947 -0.00585202
//        -0.0211169  -0.0010472    0.999776    -0.43929
//        0           0           0           1

        OdometryType T_w_b_pub;
        T_w_b_pub.frame = "map";
        T_w_b_pub.timestamp = data.timestamp;
        T_w_b_pub.pose = T_w_b;

       // pubsub->PublishOdometry(topic_gnss_raw_body, T_w_b_pub);

        OdometryType T_w_l_pub;
        T_w_l_pub.frame = "map";
        T_w_l_pub.timestamp = data.timestamp;
        T_w_l_pub.pose = T_w_l;
        pubsub->PublishOdometry(topic_gnss_odom_world, T_w_l_pub);

        GNSSINSType T_w_l_to_mapopt;
        T_w_l_to_mapopt.lla[0] = T_w_l.GetXYZ()[0];
        T_w_l_to_mapopt.lla[1] = T_w_l.GetXYZ()[1];
        T_w_l_to_mapopt.lla[2] = T_w_l.GetXYZ()[2];
//        opt_mapping_ptr->AddGNSSINSData(T_w_l_to_mapopt);

        gnssins_mutex.lock();
        poseQueue.push_back(T_w_l_pub);
        gnssins_mutex.unlock();

        GNSSOdometryType T_w_l_gnss;
        T_w_l_gnss.frame = "map";
        T_w_l_gnss.timestamp = data.timestamp;
        T_w_l_gnss.pose = T_w_l;

         if (MappingConfig::slam_mode_switch ==1){
             Function_AddGNSSOdometryTypeToFuse(T_w_l_gnss);
             EZLOG(INFO)<<"step2. 2 of 2, Data_preprocess to fuse, GNSS pose is :";
             EZLOG(INFO)<<T_w_l_gnss.pose.pose;
             EZLOG(INFO)<<"step2. 2 of 2, Data_preprocess to fuse, GNSS pose end!";
//             static int flag_load_once = 0;
//             if(flag_load_once ==0){
//                 Function_AddFirstGNSSPoint2DR(T_w_l_gnss);
//                 flag_load_once = 1;
//             }
         }
         else{  //mapping
             Function_AddGNSSOdometryTypeToOPTMapping(T_w_l_gnss);
         }
       // Function_AddGNSSOdometryTypeToFuse(T_w_l_gnss);
        //pub gnss odometry in rviz
//        if(MappingConfig::if_debug){
            // TODO T_w_l_pub->>>>>>> is Gnss result need UDP
            Udp_OdomPub(T_w_l);
            pubsub->PublishOdometry(topic_gnss_odom_world, T_w_l_pub);
//        }
    }



    void AddDrOdomData(const OdometryType& data){
        imuodom_mutex.lock();
        IMUOdomQueue.push_back(data);
        imuodom_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_){
        AllocateMemory();
        pubsub = pubsub_;
        pubsub->addPublisher(topic_origin_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskew_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskw_cloud_to_ft_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_imuodom_curlidartime_world, DataType::ODOMETRY,2000);

        pubsub->addPublisher(topic_ground_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_unground_world,DataType::LIDAR,1);

        pubsub->addPublisher(topic_cloud_pillar_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_beam_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_facade_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_roof_world,DataType::LIDAR,1);

        do_work_thread = new std::thread(&DataPreprocess::DoWork, this);
        EZLOG(INFO)<<"DataPreprocess init success!"<<std::endl;
    }

    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_){
        AllocateMemory();
        pubsub = pubsub_;
        udp_thread = udp_thread_;
        pubsub->addPublisher(topic_origin_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskew_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskw_cloud_to_ft_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_imuodom_curlidartime_world, DataType::ODOMETRY,2000);

        pubsub->addPublisher(topic_ground_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_unground_world,DataType::LIDAR,1);

        pubsub->addPublisher(topic_cloud_pillar_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_beam_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_facade_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_cloud_roof_world,DataType::LIDAR,1);

        do_work_thread = new std::thread(&DataPreprocess::DoWork, this);
        EZLOG(INFO)<<"DataPreprocess init success!"<<std::endl;
    }

    struct eigenvalue_t // Eigen Value ,lamada1 > lamada2 > lamada3;
    {
        double lamada1;
        double lamada2;
        double lamada3;
    };

    struct eigenvector_t //the eigen vector corresponding to the eigen value
    {
        Eigen::Vector3f principalDirection;
        Eigen::Vector3f middleDirection;
        Eigen::Vector3f normalDirection;
    };

    struct pca_feature_t //PCA
    {
        eigenvalue_t values;
        eigenvector_t vectors;
        double curvature;
        double linear;
        double planar;
        double spherical;
        double linear_2;
        double planar_2;
        double spherical_2;
        double normal_diff_ang_deg;
        pcl::PointNormal pt;
        int ptId;
        int pt_num = 0;
        std::vector<int> neighbor_indices;
        std::vector<bool> close_to_query_point;
    };

    bool classify_nground_pts(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud_in,//input 非地面点
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_pillar_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_beam_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_facade_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_roof_down,
                              pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_vertex,
                              float neighbor_searching_radius, int neighbor_k, int neigh_k_min, int pca_down_rate, // one in ${pca_down_rate} unground points would be select as the query points for calculating pca, the else would only be used as neighborhood points
                              float edge_thre, float planar_thre, float edge_thre_down, float planar_thre_down,
                              int extract_vertex_points_method, float curvature_thre, float vertex_curvature_non_max_radius,
                              float linear_vertical_sin_high_thre, float linear_vertical_sin_low_thre,
                              float planar_vertical_sin_high_thre, float planar_vertical_sin_low_thre,
                              bool fixed_num_downsampling = false, int pillar_down_fixed_num = 200, int facade_down_fixed_num = 800, int beam_down_fixed_num = 200,
                              int roof_down_fixed_num = 100, int unground_down_fixed_num = 20000,
                              float beam_height_max = FLT_MAX, float roof_height_min = -FLT_MAX,
                              float feature_pts_ratio_guess = 0.3, bool sharpen_with_nms = true,
                              bool use_distance_adaptive_pca = false)
    {

//        if (fixed_num_downsampling) //false
//            random_downsample_pcl(cloud_in, unground_down_fixed_num);

        //Do PCA
//        PrincipleComponentAnalysis<PointT> pca_estimator;
        std::vector<pca_feature_t> cloud_features;

        typename pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr tree(new pcl::KdTreeFLANN<PointXYZICOLRANGE>);
        tree->setInputCloud(cloud_in);

        float unit_distance = 30.0;
        ///1.计算每个非地面点的pca参数
        //output  param = cloud_features
        get_pc_pca_feature(cloud_in, cloud_features, tree, neighbor_searching_radius, neighbor_k, 1, pca_down_rate, use_distance_adaptive_pca, unit_distance);
        //LOG(WARNING)<< "PCA done";

        std::chrono::steady_clock::time_point toc_pca = std::chrono::steady_clock::now();

        //the radius should be larger for far away points
        //1.2.遍历每个非地面点，并根据pca参数为每个点进行归类
        std::vector<int> index_with_feature(cloud_in->points.size(), 0); // 0 - not special points, 1 - pillar, 2 - beam, 3 - facade, 4 - roof
        for (int i = 0; i < cloud_in->points.size(); i++)
        {
            if (cloud_features[i].pt_num > neigh_k_min)//后面所有的代码都在这个if下，离群点不对他归类
            {

                if (cloud_features[i].linear_2 > edge_thre)
                {
                    if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_pillar->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 1;
                    }
                    else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre &&
                             cloud_in->points[i].z < beam_height_max)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_beam->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 2;
                    }
                    else
                    {
                        ;
                    }

                    if (!sharpen_with_nms && cloud_features[i].linear_2 > edge_thre_down)
                    {
                        if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
                            cloud_pillar_down->points.push_back(cloud_in->points[i]);
                        else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                            cloud_beam_down->points.push_back(cloud_in->points[i]);
                        else
                        {
                            ;
                        }
                    }
                }//end if (cloud_features[i].linear_2 > edge_thre)

                else if (cloud_features[i].planar_2 > planar_thre)
                {
                    if (std::abs(cloud_features[i].vectors.normalDirection.z()) > planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], true);
                        cloud_roof->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 4;
                    }
                    else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < planar_vertical_sin_low_thre)
                    {
                        assign_normal(cloud_in->points[i], cloud_features[i], true);
                        cloud_facade->points.push_back(cloud_in->points[i]);
                        index_with_feature[i] = 3;
                    }
                    else
                    {
                        ;
                    }
                    if (!sharpen_with_nms && cloud_features[i].planar_2 > planar_thre_down)
                    {
                        if (std::abs(cloud_features[i].vectors.normalDirection.z()) > planar_vertical_sin_high_thre && cloud_in->points[i].z > roof_height_min)
                            cloud_roof_down->points.push_back(cloud_in->points[i]);
                        else if (std::abs(cloud_features[i].vectors.normalDirection.z()) < planar_vertical_sin_low_thre)
                            cloud_facade_down->points.push_back(cloud_in->points[i]);
                        else
                        {
                            ;
                        }
                    }
                }
            }
        } // end for (int i = 0; i < cloud_in->points.size(); i++)

        //According to the parameter 'extract_vertex_points_method' (0,1,2...)
        if (curvature_thre < 1e-8) // set stablilty_thre as 0 to disable the vertex extraction
            extract_vertex_points_method = 0;

        //Find Edge points by picking high curvature points among the neighborhood of unground geometric feature points (2)
        //1.3.从非特殊点中再根据这个点的周围点的信息，将其归类为beam或者pillar点
        if (extract_vertex_points_method == 2)
        {
            float vertex_feature_ratio_thre = feature_pts_ratio_guess / pca_down_rate;
            for (int i = 0; i < cloud_in->points.size(); i++)
            {
                // if (index_with_feature[i] == 0)
                // 	cloud_vertex->points.push_back(cloud_in->points[i]);
                //这个点是非特殊点，且这个点周围有足够多的点，且这个点的曲率非常大
                if (index_with_feature[i] == 0 &&
                    cloud_features[i].pt_num > neigh_k_min &&
                    cloud_features[i].curvature > curvature_thre) //curvature_thre means curvature_thre here
                {
                    int geo_feature_point_count = 0;
                    for (int j = 0; j < cloud_features[i].neighbor_indices.size(); j++)
                    {
                        if (index_with_feature[cloud_features[i].neighbor_indices[j]])
                            geo_feature_point_count++;
                    }
                    //LOG(INFO)<< "facade neighbor num: " <<geo_feature_point_count;
                    //这个非特殊点周围点特殊点也要足够多
                    if (1.0 * geo_feature_point_count / cloud_features[i].pt_num > vertex_feature_ratio_thre) //most of the neighbors are feature points
                    {
                        //cloud_vertex->points.push_back(cloud_in->points[i]);

                        assign_normal(cloud_in->points[i], cloud_features[i], false);
                        cloud_in->points[i].normal[3] = 5.0 * cloud_features[i].curvature; //save in the un-used normal[3]  (PointNormal4D)
                        if (std::abs(cloud_features[i].vectors.principalDirection.z()) > linear_vertical_sin_high_thre)
                        {
                            cloud_pillar->points.push_back(cloud_in->points[i]);
                            //cloud_pillar_down->points.push_back(cloud_in->points[i]);
                            index_with_feature[i] = 1;
                        }
                        else if (std::abs(cloud_features[i].vectors.principalDirection.z()) < linear_vertical_sin_low_thre && cloud_in->points[i].z < beam_height_max)
                        {
                            cloud_beam->points.push_back(cloud_in->points[i]);
                            //cloud_beam_down->points.push_back(cloud_in->points[i]);
                            index_with_feature[i] = 2;
                        }
                    }
                }
            }
        }

        //if extract_vertex_points_method == 0 ---> do not extract vertex points (0)
//        std::chrono::steady_clock::time_point toc_1 = std::chrono::steady_clock::now();
        //extract neighborhood feature descriptor for pillar points
        //Find Vertex (Edge) points by picking points with maximum local curvature (1)
        //if (extract_vertex_points_method == 1) //Deprecated
        //detect_key_pts(cloud_in, cloud_features, index_with_feature,cloud_vertex, 4.0 * curvature_thre, vertex_curvature_non_max_radius, 0.5 * curvature_thre);
        int min_neighbor_feature_pts = (int)(feature_pts_ratio_guess / pca_down_rate * neighbor_k) - 1;

        //get the vertex keypoints and encode its neighborhood in a simple descriptor
        ///2.为某个点生成描述子
//        encode_stable_points(cloud_in, cloud_vertex, cloud_features, index_with_feature,
//                             0.3 * curvature_thre, min_neighbor_feature_pts, neigh_k_min); //encode the keypoints, we will get a simple descriptor of the putable keypoints

        //LOG(WARNING)<< "encode ncc feature descriptor done";


        //Non_max_suppression of the feature points //TODO: add already built-kd tree here
        ///3.1对pillar cloud_facade beam cloud_roof 特征点进行非最大值抑制
//        if (sharpen_with_nms)
//        {
//            float nms_radius = 0.25 * neighbor_searching_radius;
//#pragma omp parallel sections
//            {
//#pragma omp section
//                {
//                    if (pillar_down_fixed_num > 0)
//                        non_max_suppress(cloud_pillar, cloud_pillar_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (facade_down_fixed_num > 0)
//                        non_max_suppress(cloud_facade, cloud_facade_down, nms_radius);
//                }
//#pragma omp section
//                {
//                    if (beam_down_fixed_num > 0)
//                        non_max_suppress(cloud_beam, cloud_beam_down, nms_radius);
//
//                    if (roof_down_fixed_num > 0)
//                        non_max_suppress(cloud_roof, cloud_roof_down, nms_radius);
//                }
//            }
//        }

        ///3.2.对cloud_facade cloud_beam_down cloud_roof_down 将点云分成不同sector，然后在sector中进行随机采样
//        if (fixed_num_downsampling)
//        {
//            random_downsample_pcl(cloud_pillar_down, pillar_down_fixed_num);
//            int sector_num = 4;
//            xy_normal_balanced_downsample(cloud_facade_down, (int)(facade_down_fixed_num / sector_num), sector_num);
//
//            xy_normal_balanced_downsample(cloud_beam_down, (int)(beam_down_fixed_num / sector_num), sector_num); // here the normal is the primary vector
//            //random_downsample_pcl(cloud_roof_down, 100);
//            random_downsample_pcl(cloud_roof_down, roof_down_fixed_num);
//        }

        //Free the memory
        std::vector<pca_feature_t>().swap(cloud_features);
        std::vector<int>().swap(index_with_feature);



        return 1;
    } //end classify_nground_pts

    //is_palne_feature (true: assign point normal as pca normal vector, false: assign point normal as pca primary direction vector)
    bool assign_normal(PointXYZICOLRANGE &pt, pca_feature_t &pca_feature, bool is_plane_feature = true)
    {
        if (is_plane_feature)
        {
            pt.normal_x = pca_feature.vectors.normalDirection.x();
            pt.normal_y = pca_feature.vectors.normalDirection.y();
            pt.normal_z = pca_feature.vectors.normalDirection.z();
            pt.normal[3] = pca_feature.planar_2; //planrity
        }
        else
        {
            pt.normal_x = pca_feature.vectors.principalDirection.x();
            pt.normal_y = pca_feature.vectors.principalDirection.y();
            pt.normal_z = pca_feature.vectors.principalDirection.z();
            pt.normal[3] = pca_feature.linear_2; //linarity
        }
        return true;
    }

    // R - K neighborhood (with already built-kd tree)
    //within the radius, we would select the nearest K points for calculating PCA
    bool get_pc_pca_feature(typename pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                            std::vector<pca_feature_t> &features,
                            typename pcl::KdTreeFLANN<PointXYZICOLRANGE>::Ptr &tree,
                            float radius, int nearest_k, int min_k = 1, int pca_down_rate = 1,
                            bool distance_adaptive_on = false, float unit_dist = 35.0)
    {
        //LOG(INFO) << "[" << in_cloud->points.size() << "] points used for PCA, pca down rate is [" << pca_down_rate << "]";
        features.resize(in_cloud->points.size());

        for (int i = 0; i < in_cloud->points.size(); i += pca_down_rate) //faster way
        {
            // if (i % pca_down_rate == 0) {//this way is much slower
            std::vector<int> search_indices_used; //points would be stored in sequence (from the closest point to the farthest point within the neighborhood)
            std::vector<int> search_indices;	  //point index vector
            std::vector<float> squared_distances; //distance vector

            float neighborhood_r = radius;
            int neighborhood_k = nearest_k;

            if (distance_adaptive_on)
            {
                double dist = std::sqrt(in_cloud->points[i].x * in_cloud->points[i].x +
                                        in_cloud->points[i].y * in_cloud->points[i].y +
                                        in_cloud->points[i].z * in_cloud->points[i].z);
                if (dist > unit_dist)
                {
                    neighborhood_r = std::sqrt(dist / unit_dist) * radius;
                    //neighborhood_k = (int)(unit_dist / dist * nearest_k));
                }
            }
            //nearest_k=0 --> the knn is disabled, only the rnn is used
            tree->radiusSearch(i, neighborhood_r, search_indices, squared_distances, neighborhood_k);

            features[i].pt.x = in_cloud->points[i].x;
            features[i].pt.y = in_cloud->points[i].y;
            features[i].pt.z = in_cloud->points[i].z;
            features[i].ptId = i;
            features[i].pt_num = search_indices.size();

            //deprecated
            features[i].close_to_query_point.resize(search_indices.size());
            for (int j = 0; j < search_indices.size(); j++)
            {
                if (squared_distances[j] < 0.64 * radius * radius) // 0.5^(2/3)
                    features[i].close_to_query_point[j] = true;
                else
                    features[i].close_to_query_point[j] = false;
            }

            get_pca_feature(in_cloud, search_indices, features[i]);

            if (features[i].pt_num > min_k)
                assign_normal(in_cloud->points[i], features[i]);
            std::vector<int>().swap(search_indices);
            std::vector<int>().swap(search_indices_used);
            std::vector<float>().swap(squared_distances);
        }
        //}
        return true;
    }

    /**
		* \brief Use PCL to accomplish the Principle Component Analysis (PCA)
		* of one point and its neighborhood
		* \param[in] in_cloud is the input Point Cloud Pointer
		* \param[in] search_indices is the neighborhood points' indices of the search point.
		* \param[out]feature is the pca_feature_t of the search point.
		*/
    bool get_pca_feature(typename pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                         std::vector<int> &search_indices,
                         pca_feature_t &feature)
    {
        int pt_num = search_indices.size();

        if (pt_num <= 3)
            return false;

        pcl::PointCloud<PointXYZICOLRANGE>::Ptr selected_cloud(new pcl::PointCloud<PointXYZICOLRANGE>());
        for (int i = 0; i < pt_num; ++i)
            selected_cloud->points.push_back(in_cloud->points[search_indices[i]]);

        pcl::PCA<PointXYZICOLRANGE> pca_operator;
        pca_operator.setInputCloud(selected_cloud);

        // Compute eigen values and eigen vectors
        Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
        Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

        feature.vectors.principalDirection = eigen_vectors.col(0);
        feature.vectors.normalDirection = eigen_vectors.col(2);

        feature.vectors.principalDirection.normalize();
        feature.vectors.normalDirection.normalize();

        feature.values.lamada1 = eigen_values(0);
        feature.values.lamada2 = eigen_values(1);
        feature.values.lamada3 = eigen_values(2);

        if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
            feature.curvature = 0;
        else
            feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);

        // feature.linear_2 = (sqrt(feature.values.lamada1) - sqrt(feature.values.lamada2)) / sqrt(feature.values.lamada1);
        // feature.planar_2 = (sqrt(feature.values.lamada2) - sqrt(feature.values.lamada3)) / sqrt(feature.values.lamada1);
        // feature.spherical_2 = sqrt(feature.values.lamada3) / sqrt(feature.values.lamada1);
        feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) / (feature.values.lamada1);
        feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) / (feature.values.lamada1);
        feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);

        search_indices.swap(feature.neighbor_indices);
        return true;
    }

    void fast_ground_filter(
                            const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud_in,
                            pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground,//地面点
                            pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_ground_down,//降才采样之后的地面点
                            pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_unground,//非地面点
                            pcl::PointCloud<PointXYZICOLRANGE>::Ptr cloud_curb,//废弃了不再使用
                            int min_grid_pt_num, float max_ground_height,float grid_resolution,
                            int distance_weight_downsampling_method,float standard_distance,int nonground_random_down_rate,float intensity_thre = FLT_MAX,
                            bool apply_grid_wise_outlier_filter = false,float outlier_std_scale = 3.0,
                            int reliable_neighbor_grid_num_thre = 0, int ground_random_down_rate = 1,float neighbor_height_diff = 25,
                             float max_height_difference = 0.3,int estimate_ground_normal_method = 3,
                            float normal_estimation_radius = 2.0, bool fixed_num_downsampling = false,int ground_random_down_down_rate = 2

//                            ,  //estimate_ground_normal_method, 0: directly use (0,0,1), 1: estimate normal in fix radius neighborhood , 2: estimate normal in k nearest neighborhood, 3: use ransac to estimate plane coeffs in a grid
//                               //standard distance: the distance where the distance_weight is 1
//                           , int down_ground_fixed_num = 1000,
//                            bool detect_curb_or_not = false,
                                    ){

        //0.主要功能是计算点云统计信息,为地面点提取做参数准备。计算出平均高度、地面点判定阈值、
        //地面最小高度、地下噪声点阈值等,并获取点云范围信息。这些都是构建grid和提取地面点需要的重要参数
        //For some points,  calculating the approximate mean height

        int reliable_grid_pts_count_thre = min_grid_pt_num - 1;
        int count_checkpoint = 0;
        float sum_height = 0.001;
        float appro_mean_height;//当前帧的平均高度
        float min_ground_height = max_ground_height;
        float underground_noise_thre = -FLT_MAX;
        float non_ground_height_thre;//等于平均高度+设定的地面最大高度
        float distance_weight;
        // int ground_random_down_rate_temp = ground_random_down_rate;
        // int nonground_random_down_rate_temp = nonground_random_down_rate;

        for (int j = 0; j < cloud_in->size(); j++)
        {
            if (j % 100 == 0)
            {
                sum_height += cloud_in->points[j].z;
                count_checkpoint++;
            }
        }

        appro_mean_height = sum_height / count_checkpoint; //calculate around height
        non_ground_height_thre = appro_mean_height + max_ground_height;//max_ground_height  = 1.5
        EZLOG(INFO)<<"appro_mean_height "<<appro_mean_height;
        EZLOG(INFO)<<"non_ground_height_thre "<<non_ground_height_thre;

        bounds_t bounds;
        centerpoint_t center_pt;
        this->get_cloud_bbx_cpt(cloud_in, bounds, center_pt);

        //1.构建grid map
        int row, col, num_grid;
        row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);//grid_resolution 默认参数 = 3.0
        col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
        num_grid = row * col;
        EZLOG(INFO)<<"num_grid "<<num_grid;

        grid_t *grid = new grid_t[num_grid];

        //Each grid
        for (int i = 0; i < num_grid; i++)
        {
            grid[i].min_z = FLT_MAX;
            grid[i].neighbor_min_z = FLT_MAX;
        }

        //Each point ---> determine the grid to which the point belongs
        for (int j = 0; j < cloud_in->points.size(); j++)
        {
            int temp_row, temp_col, temp_id;
            temp_col = floor((cloud_in->points[j].x - bounds.min_x) / grid_resolution);
            temp_row = floor((cloud_in->points[j].y - bounds.min_y) / grid_resolution);
            temp_id = temp_row * col + temp_col;
            if (temp_id >= 0 && temp_id < num_grid)
            {
                ///if use distance_weight_downsampling and the point is the first point in grid
                ///calculate distance from grid to origin
                if (distance_weight_downsampling_method > 0 && !grid[temp_id].pts_count)
                {
                    grid[temp_id].dist2station = std::sqrt(cloud_in->points[j].x * cloud_in->points[j].x + cloud_in->points[j].y * cloud_in->points[j].y + cloud_in->points[j].z * cloud_in->points[j].z);
                }

                if (cloud_in->points[j].z > non_ground_height_thre)//non_ground_height_thre = 等于平均高度+设定的地面最大高度
                {
                    //standard_distance 默认参数 = 15.0
                    //要注意这里作者对非地面点做了一个随机下采样
                    distance_weight = 1.0 * standard_distance / (grid[temp_id].dist2station + 0.0001); //avoiding Floating point exception
                    int nonground_random_down_rate_temp = nonground_random_down_rate;
                    if (distance_weight_downsampling_method == 1) //linear weight
                        nonground_random_down_rate_temp = (int)(distance_weight * nonground_random_down_rate + 1);
                    else if (distance_weight_downsampling_method == 2) //quadratic weight
                        nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * nonground_random_down_rate + 1);

                    ///对点进行随机下采样,满足下采样率要求或者强度高的点会被保留,save in cloud_unground.
                    if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[j].intensity > intensity_thre)//intensity_thre = double最大值
                    {
                        cloud_in->points[j].data[3] = cloud_in->points[j].z - (appro_mean_height - 3.0); //data[3] stores the approximate point height above ground
                        cloud_unground->points.push_back(cloud_in->points[j]);
                    }
                }
                    ///处理可能属于地面的点(高度低于非地面阈值,但高于地下噪声阈值)
                else if (cloud_in->points[j].z > underground_noise_thre)
                {
                    grid[temp_id].pts_count++;
                    grid[temp_id].point_id.push_back(j);
                    if (cloud_in->points[j].z < grid[temp_id].min_z) //update min_z and neighbor_min_z
                    {
                        grid[temp_id].min_z = cloud_in->points[j].z;
                        grid[temp_id].neighbor_min_z = cloud_in->points[j].z;
                    }
                }
            } // end if (temp_id >= 0 && temp_id < num_grid)
        } // end for (int j = 0; j < cloud_in.cloud_ptr->points.size(); j++)

//        EZLOG(INFO)<<"num_unground = "<<num_unground<<endl;
//        EZLOG(INFO)<<"cloud_unground->points.size() = "<<cloud_unground->points.size()<<endl;
//
//        TicToc time_3;

        if (apply_grid_wise_outlier_filter)
        {
            //Each grid: Check outlier //calculate mean and standard deviation of z in one grid, then set mean-2*std as the threshold for outliers
            for (int i = 0; i < num_grid; i++)
            {
                if (grid[i].pts_count >= min_grid_pt_num)///判断该grid中的点数量是否满足最小阈值min_grid_pt_num
                {
                    double sum_z = 0, sum_z2 = 0, std_z = 0, mean_z = 0;
                    for (int j = 0; j < grid[i].point_id.size(); j++)
                        sum_z += cloud_in->points[grid[i].point_id[j]].z;
                    mean_z = sum_z / grid[i].pts_count;
                    for (int j = 0; j < grid[i].point_id.size(); j++)
                        sum_z2 += (cloud_in->points[grid[i].point_id[j]].z - mean_z) * (cloud_in->points[grid[i].point_id[j]].z - mean_z);
                    std_z = std::sqrt(sum_z2 / grid[i].pts_count);
                    grid[i].min_z_outlier_thre = mean_z - outlier_std_scale * std_z;///calculate outlier threshold
                    grid[i].min_z = max_(grid[i].min_z, grid[i].min_z_outlier_thre);///更新grid最小高度grid[i].min_z,取原始最小高度和离群点阈值中的最大值
                    grid[i].neighbor_min_z = grid[i].min_z;///将更新后的最小高度同步到相邻grid中
                }
            }
        } //end if (apply_grid_wise_outlier_filter)

//        EZLOG(INFO)<<"time_3.toc() = "<<time_3.toc();
//
//        TicToc time_4;

        //2.遍历所有的grid
        for (int m = 0; m < num_grid; m++)
        {
            int temp_row, temp_col;
            temp_row = m / col;
            temp_col = m % col;
            if (temp_row >= 1 && temp_row <= row - 2 && temp_col >= 1 && temp_col <= col - 2)///判断是否位于边界之内
            {
                ///遍历上下左右的相邻grid
                for (int j = -1; j <= 1; j++) //row
                {
                    for (int k = -1; k <= 1; k++) //col
                    {
                        ///将当前grid的neighbor_min_z更新为相邻grid的min_z的最小值,同时统计相邻grid中点数大于阈值的grid数量,记为reliable_neighbor_grid_num
                        grid[m].neighbor_min_z = min_(grid[m].neighbor_min_z, grid[m + j * col + k].min_z);
//                        EZLOG(INFO)<<"grid[m].neighbor_min_z = "<<grid[m].neighbor_min_z<<endl;
                        if (grid[m + j * col + k].pts_count > reliable_grid_pts_count_thre)
                            grid[m].reliable_neighbor_grid_num++;
                    }
                }
            }
        }
//
        //***********
        std::vector<std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>>> grid_ground_pcs(num_grid);
        std::vector<std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>>> grid_unground_pcs(num_grid);
        for (int i = 0; i < num_grid; i++)
        {
            std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>> grid_ground_pc_temp(new pcl::PointCloud<PointXYZICOLRANGE>);
            grid_ground_pcs[i] = grid_ground_pc_temp;
            std::shared_ptr<pcl::PointCloud<PointXYZICOLRANGE>> grid_unground_pc_temp(new pcl::PointCloud<PointXYZICOLRANGE>);
            grid_unground_pcs[i] = grid_unground_pc_temp;
        }

//        int use_grid_num = 0;
//        EZLOG(INFO)<<"num_grid = "<<num_grid;
        int estimate_normal_num = 0;
        double estimate_normal_time = 0;
        double ransac_time = 0;

//        ///use for thread
//        {
//            void dealwith_grid_thread(const int start_index, const int end_index,...){
//                for(int i = start_index; i < end_index; ++i){
//                    pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground(new pcl::PointCloud<PointXYZICOLRANGE>);
//                    //do_something
//                }
//            }
//            int num_each_grid = num_grid / 4;
//            int num_thread = 4;
//            std::vector<std::thread> threads;
//            for(int i = 0; i < num_thread; ++i){
//                int start = i * num_each_grid;
//                int end = start + num_each_grid;
//                end = std::min(end, num_grid);
//                std::thread th(dealwith_grid_thread, start, end, ...);
//                threads.push_back( std::move(th));
//            }
//            for(int i = 0; i < num_thread; ++i){
//                threads[i].join();
//            }
//        }


//        //For each grid
        for (int i = 0; i < num_grid; i++)
        {
            pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground(new pcl::PointCloud<PointXYZICOLRANGE>);
            ///过滤掉点数太少的grid
            //min_grid_pt_num = 8,reliable_neighbor_grid_num_thre = 0
//            EZLOG(INFO)<<"grid[i].pts_count = "<<grid[i].pts_count<<endl;
//            EZLOG(INFO)<<"grid[i].reliable_neighbor_grid_num = "<<grid[i].reliable_neighbor_grid_num<<endl;
            if (grid[i].pts_count >= min_grid_pt_num && grid[i].reliable_neighbor_grid_num >= reliable_neighbor_grid_num_thre)
            {
//                ++use_grid_num;

                ///计算距离权重,用于地面点和非地面点的下采样率
                int ground_random_down_rate_temp = ground_random_down_rate;
                int nonground_random_down_rate_temp = nonground_random_down_rate;
                distance_weight = 1.0 * standard_distance / (grid[i].dist2station + 0.0001);
                if (distance_weight_downsampling_method == 1) //linear weight
                {
                    ground_random_down_rate_temp = (int)(distance_weight * ground_random_down_rate + 1);
                    nonground_random_down_rate_temp = (int)(distance_weight * nonground_random_down_rate + 1);
                }
                else if (distance_weight_downsampling_method == 2) //quadratic weight
                {
                    ground_random_down_rate_temp = (int)(distance_weight * distance_weight * ground_random_down_rate + 1);
                    nonground_random_down_rate_temp = (int)(distance_weight * distance_weight * nonground_random_down_rate + 1);
                }

                bool last_point_is_ground = false;

                ///如果grid与附近grid差小于阈值,说明是地面grid:
//                EZLOG(INFO)<<"grid[i].min_z  = "<<grid[i].min_z<<endl;
//                EZLOG(INFO)<<"grid[i].neighbor_min_z  = "<<grid[i].neighbor_min_z<<endl;
//                EZLOG(INFO)<<"grid[i].min_z - grid[i].neighbor_min_z = "<<grid[i].min_z - grid[i].neighbor_min_z<<endl;
                if (grid[i].min_z - grid[i].neighbor_min_z < neighbor_height_diff)
                {
//                    TicToc time_7;

                    ///遍历grid中的每个点,根据阈值提取地面点
                    for (int j = 0; j < grid[i].point_id.size(); ++j )
                    {

                        if(last_point_is_ground){
                            grid_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                            last_point_is_ground = false;
                            continue;
                        }
                        /// 判断点的高度是否超过该grid的离群点高度阈值min_z_outlier_thre
                        else if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre)
                        {
                            ///继续判断是否满足地面点条件(点与grid最小高度之差小于阈值max_height_difference)
                            if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference)
                            {
                                /// groud point
                                //cloud_ground_full->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                if (estimate_ground_normal_method == 3)///默认使用这种方法 use ransac to estimate plane coeffs in a grid
                                {
                                    grid_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                    last_point_is_ground = true;
//                                    EZLOG(INFO)<<"grid_ground_pcs.size() "<<grid_ground_pcs.size();
                                }
                                else
                                {
                                    ///否则进行随机下采样,按下采样率将地面点加入grid_ground_pcs
                                    if (j % ground_random_down_rate_temp == 0) // for example 10
                                    {
                                        if (estimate_ground_normal_method == 0)///directly use (0,0,1)
                                        {
                                            cloud_in->points[grid[i].point_id[j]].normal_x = 0.0;
                                            cloud_in->points[grid[i].point_id[j]].normal_y = 0.0;
                                            cloud_in->points[grid[i].point_id[j]].normal_z = 1.0;
                                        }
                                        grid_ground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                        //cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to ground points
                                    }
//                                    EZLOG(INFO)<<"grid_ground_pcs.size() "<<grid_ground_pcs.size();
                                }

                            }
                            else /// inner grid unground points
                            {

                                if (j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre) //extract more points on signs and vehicle license plate
                                {
                                    cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z; //data[3] stores the point height above ground
                                    grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
                                    //cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
                                }
//                                EZLOG(INFO)<<"grid_unground_pcs.size() "<<grid_unground_pcs.size();
                            }
                        }//end if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre)
                    }
                }
//                else //unground grid
//                {
//                    for (int j = 0; j < grid[i].point_id.size(); j++)
//                    {
//                        ///random downsample
//                        if (cloud_in->points[grid[i].point_id[j]].z > grid[i].min_z_outlier_thre &&
//                            (j % nonground_random_down_rate_temp == 0 || cloud_in->points[grid[i].point_id[j]].intensity > intensity_thre))
//                        {
//                            ///在data[3]通道记录非地面点的高度(相对本grid地面高度) save unground point in grid_unground_pcs
//                            cloud_in->points[grid[i].point_id[j]].data[3] = cloud_in->points[grid[i].point_id[j]].z - grid[i].neighbor_min_z; //data[3] stores the point height above ground
//                            grid_unground_pcs[i]->points.push_back(cloud_in->points[grid[i].point_id[j]]);
//                            //cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]); //Add to nonground points
//                        }
//                    }
//                }

//                EZLOG(INFO)<<"grid_unground_pcs.size() "<<grid_unground_pcs.size();
//                TicToc time_8;

                for (int j = 0; j < grid_ground->points.size(); j++)
                {
                        grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
                }

                //使用所有的点拟合平面，并更新地面点的法向量
                ///使用RANSAC算法对grid的地面点进行平面拟合,以获取地面法向量
//                if (estimate_ground_normal_method == 3 && grid_ground->points.size() >= min_grid_pt_num)
//                {
//                    ++estimate_normal_num;
////                    std::chrono::steady_clock::time_point tic_ransac = std::chrono::steady_clock::now();
//                    float normal_x, normal_y, normal_z;
//
//                    //RANSAC iteration number equation: p=1-(1-r^N)^M,
//                    //r is the inlier ratio (> 0.75 in our case), N is 3 in our case (3 points can fit a plane), to get a confidence > 0.99, we need about 20 iteration (M=20)
//                    /// use estimate_ground_normal_by_ransac函数进行RANSAC平面拟合,获取法向量坐标
//
//                    //pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground_tmp(new pcl::PointCloud<PointXYZICOLRANGE>);
//                    for (int j = 0; j < grid_ground->points.size(); j += 10)
//                    {
////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
//
//
//                            grid_ground->points.push_back(grid_ground->points[j]);
//                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
//
//                    }
//
////                    EZLOG(INFO)<<"grid_ground->points.size() = "<<grid_ground->points.size();
////                    TicToc time_9;
//                    estimate_ground_normal_by_ransac(grid_ground, 0.3 * max_height_difference, 20, normal_x, normal_y, normal_z);
//                    ransac_time = ransac_time +time_9.toc();
////                    for (int j = 0; j < grid_ground->points.size(); j++)
////                    {
//////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
////
////                        if (j % 10 == 0 && std::abs(normal_z) > 0.8) //53 deg
////                        {
////                            grid_ground->points[j].normal_x = normal_x;
////                            grid_ground->points[j].normal_y = normal_y;
////                            grid_ground->points[j].normal_z = normal_z;
////                            grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
////                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
////                        }
////                    }
//                    for (int j = 0; j < grid_ground->points.size(); j++)
//                    {
////                        if (j % ground_random_down_rate_temp == 0 && std::abs(normal_z) > 0.8) //53 deg
//
//                        if (std::abs(normal_z) > 0.8) //53 deg
//                        {
//                            grid_ground->points[j].normal_x = normal_x;
//                            grid_ground->points[j].normal_y = normal_y;
//                            grid_ground->points[j].normal_z = normal_z;
//                            grid_ground_pcs[i]->points.push_back(grid_ground->points[j]); //Add to ground points
//                            //cloud_ground->points.push_back(grid_ground->points[j]); //Add to ground points
//                        }
//                    }
////                    std::chrono::steady_clock::time_point toc_ransac = std::chrono::steady_clock::now();
////                    std::chrono::duration<double> ground_ransac_time_per_grid = std::chrono::duration_cast<std::chrono::duration<double>>(toc_ransac - tic_ransac);
////                    consuming_time_ransac += ground_ransac_time_per_grid.count() * 1000.0; //unit: ms
////                    EZLOG(INFO)<<"consuming_time_ransac "<<consuming_time_ransac;
//                }// end if (estimate_ground_normal_method == 3 && grid_ground->points.size() >= min_grid_pt_num)

                pcl::PointCloud<PointXYZICOLRANGE>().swap(*grid_ground);
//                estimate_normal_time = estimate_normal_time + time_8.toc();
//                EZLOG(INFO)<<"time_8.toc() = "<<time_8.toc();
            }
        }//end for (int i = 0; i < num_grid; i++)]

        //combine the ground and unground points
        ///根据grid中的所有点更新最终的地面点和非地面点
        for (int i = 0; i < num_grid; i++)
        {
            cloud_ground->points.insert(cloud_ground->points.end(), grid_ground_pcs[i]->points.begin(), grid_ground_pcs[i]->points.end());
            cloud_unground->points.insert(cloud_unground->points.end(), grid_unground_pcs[i]->points.begin(), grid_unground_pcs[i]->points.end());
        }
        EZLOG(INFO)<<"cloud_ground->points.size() = "<<cloud_ground->points.size();
        EZLOG(INFO)<<"cloud_unground->points.size() = "<<cloud_unground->points.size();
//
        //free memory
        delete[] grid;

//        ///根据设置的estimate_ground_normal_method,对合并后的地面点cloud_ground计算法线
//        int normal_estimation_neighbor_k = 2 * min_grid_pt_num;
//        pcl::PointCloud<pcl::Normal>::Ptr ground_normal(new pcl::PointCloud<pcl::Normal>);
//        if (estimate_ground_normal_method == 1)
////            pca_estimator.get_normal_pcar(cloud_ground, normal_estimation_radius, ground_normal);
//            get_normal_pcar(cloud_ground, normal_estimation_radius, ground_normal);
//        else if (estimate_ground_normal_method == 2)
////            pca_estimator.get_normal_pcak(cloud_ground, normal_estimation_neighbor_k, ground_normal);
//            get_normal_pcak(cloud_ground, normal_estimation_neighbor_k, ground_normal);
//
//        //3.对地面点进行随机下采样,并对每个地面点赋值法线
//        for (int i = 0; i < cloud_ground->points.size(); i++)
//        {
//            if (estimate_ground_normal_method == 1 || estimate_ground_normal_method == 2)
//            {
//                cloud_ground->points[i].normal_x = ground_normal->points[i].normal_x;
//                cloud_ground->points[i].normal_y = ground_normal->points[i].normal_y;
//                cloud_ground->points[i].normal_z = ground_normal->points[i].normal_z;
//            }
//            if (!fixed_num_downsampling)
//            {
//                //LOG(INFO)<<cloud_ground->points[i].normal_x << "," << cloud_ground->points[i].normal_y << "," << cloud_ground->points[i].normal_z;
//                if (i % ground_random_down_down_rate == 0)
//                    cloud_ground_down->points.push_back(cloud_ground->points[i]);
//            }
//        }

//
////        if (fixed_num_downsampling)
////            random_downsample_pcl(cloud_ground, cloud_ground_down, down_ground_fixed_num);

        EZLOG(INFO)<<"finish ground_filter "<<endl;
//
    } // end function fast_ground_filter

    void check_normal(pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        //It is advisable to check the normals before the call to compute()
        for (int i = 0; i < normals->points.size(); i++)
        {
            if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
            {
                normals->points[i].normal_x = 0.577; // 1/ sqrt(3)
                normals->points[i].normal_y = 0.577;
                normals->points[i].normal_z = 0.577;
                //normals->points[i].curvature = 0.0;
            }
        }
    }

    bool get_normal_pcar(pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                         float radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        // Create the normal estimation class, and pass the input dataset to it;
        pcl::NormalEstimation<PointXYZICOLRANGE, pcl::Normal> ne;
        ne.setInputCloud(in_cloud);
        // Create an empty kd-tree representation, and pass it to the normal estimation object;
        pcl::search::KdTree<PointXYZICOLRANGE>::Ptr tree(new pcl::search::KdTree<PointXYZICOLRANGE>());

        ne.setSearchMethod(tree);
        // Use all neighbors in a sphere of radius;
        ne.setRadiusSearch(radius);
        // Compute the normal
        ne.compute(*normals);
        check_normal(normals);
        return true;
    }

    bool get_normal_pcak(pcl::PointCloud<PointXYZICOLRANGE>::Ptr in_cloud,
                         int K,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals)
    {
        // Create the normal estimation class, and pass the input dataset to it;
        pcl::NormalEstimation<PointXYZICOLRANGE, pcl::Normal> ne;
//        ne.setNumberOfThreads(2); //More threads sometimes would not speed up the procedure
        ne.setInputCloud(in_cloud);
        // Create an empty kd-tree representation, and pass it to the normal estimation object;
        pcl::search::KdTree<PointXYZICOLRANGE>::Ptr tree(new pcl::search::KdTree<PointXYZICOLRANGE>());
        ne.setSearchMethod(tree);
        // Use all neighbors in a sphere of radius;
        ne.setKSearch(K);
        // Compute the normal
        ne.compute(*normals);
        check_normal(normals);
        return true;
    }

    bool plane_seg_ransac(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud,
                          float threshold, int max_iter,
                          pcl::PointCloud<PointXYZICOLRANGE>::Ptr &planecloud,
                          pcl::ModelCoefficients::Ptr &coefficients) //Ransac
    {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<PointXYZICOLRANGE> sacseg;//**********************************

        // Optional
        sacseg.setOptimizeCoefficients(true);

        // Mandatory
        sacseg.setModelType(pcl::SACMODEL_PLANE);
        sacseg.setMethodType(pcl::SAC_RANSAC);
        sacseg.setDistanceThreshold(threshold);
        sacseg.setMaxIterations(max_iter);

        sacseg.setInputCloud(cloud);
        sacseg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
        }

        /*cout << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << std::endl;*/

        //LOG(INFO) << "Model inliers number: " << inliers->indices.size() << std::endl;

        for (size_t i = 0; i < inliers->indices.size(); ++i)
        {
            planecloud->push_back(cloud->points[inliers->indices[i]]);
        }
        return 1;
    }

    bool estimate_ground_normal_by_ransac(pcl::PointCloud<PointXYZICOLRANGE>::Ptr &grid_ground,
                                          float dist_thre, int max_iter, float &nx, float &ny, float &nz)
    {

        pcl::PointCloud<PointXYZICOLRANGE>::Ptr grid_ground_fit(new pcl::PointCloud<PointXYZICOLRANGE>);
        pcl::ModelCoefficients::Ptr grid_coeff(new pcl::ModelCoefficients);
        plane_seg_ransac(grid_ground, dist_thre, max_iter, grid_ground_fit, grid_coeff);

        grid_ground.swap(grid_ground_fit);
        nx = grid_coeff->values[0];
        ny = grid_coeff->values[1];
        nz = grid_coeff->values[2];

        //LOG(INFO) << nx << "," << ny << "," << nz;
        return 1;
    }


    struct grid_t
    {
        std::vector<int> point_id;
        float min_z;
        float max_z;
        float delta_z;
        float min_z_x; //X of Lowest Point in the Voxel;
        float min_z_y; //Y of Lowest Point in the Voxel;
        float min_z_outlier_thre;
        float neighbor_min_z;
        int pts_count;
        int reliable_neighbor_grid_num;
        float mean_z;
        float dist2station;//这个grid距离激光的距离

        grid_t()
        {
            min_z = min_z_x = min_z_y = neighbor_min_z = mean_z = 0.f;
            pts_count = 0;
            reliable_neighbor_grid_num = 0;
            delta_z = 0.0;
            dist2station = 0.001;
            min_z_outlier_thre = -FLT_MAX;
        }
    };

    struct bounds_t
    {
        double min_x;
        double min_y;
        double min_z;
        double max_x;
        double max_y;
        double max_z;
        int type;

        bounds_t()
        {
            min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
        }
        void inf_x()
        {
            min_x = -DBL_MAX;
            max_x = DBL_MAX;
        }
        void inf_y()
        {
            min_y = -DBL_MAX;
            max_y = DBL_MAX;
        }
        void inf_z()
        {
            min_z = -DBL_MAX;
            max_z = DBL_MAX;
        }
        void inf_xyz()
        {
            inf_x();
            inf_y();
            inf_z();
        }
    };

    struct centerpoint_t
    {
        double x;
        double y;
        double z;
        centerpoint_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    };

    void get_cloud_bbx_cpt(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud, bounds_t &bound, centerpoint_t &cp)
    {
        get_cloud_bbx(cloud, bound);
        cp.x = 0.5 * (bound.min_x + bound.max_x);
        cp.y = 0.5 * (bound.min_y + bound.max_y);
        cp.z = 0.5 * (bound.min_z + bound.max_z);
    }

    void get_cloud_bbx(const pcl::PointCloud<PointXYZICOLRANGE>::Ptr &cloud, bounds_t &bound)
    {
        double min_x = DBL_MAX;
        double min_y = DBL_MAX;
        double min_z = DBL_MAX;
        double max_x = -DBL_MAX;
        double max_y = -DBL_MAX;
        double max_z = -DBL_MAX;
        //
        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (min_x > cloud->points[i].x)
                min_x = cloud->points[i].x;
            if (min_y > cloud->points[i].y)
                min_y = cloud->points[i].y;
            if (min_z > cloud->points[i].z)
                min_z = cloud->points[i].z;
            if (max_x < cloud->points[i].x)
                max_x = cloud->points[i].x;
            if (max_y < cloud->points[i].y)
                max_y = cloud->points[i].y;
            if (max_z < cloud->points[i].z)
                max_z = cloud->points[i].z;
        }
        bound.min_x = min_x;
        bound.max_x = max_x;
        bound.min_y = min_y;
        bound.max_y = max_y;
        bound.min_z = min_z;
        bound.max_z = max_z;
    }

};
#endif

