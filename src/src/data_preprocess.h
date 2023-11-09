
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
 //   std::function<void(const GNSSOdometryType&)>Function_AddGNSSOdometryTypeToOPTMapping;
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
    std::string topic_lidar_align_gnss = "/lidar_align_gnss";

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

            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                if(deque_cloud.empty()){
                    sleep(0.01);
                    continue;
                }
            }
           // if(!isEmpty){
            {
//                EZLOG(INFO)<<"DoWork"<<endl;
                ///0.do something

                std::deque<OdometryType> imuodom_copy;
                imuodom_mutex.lock();
                imuodom_copy = IMUOdomQueue;
                imuodom_mutex.unlock();
                double imuodo_min_ros_time = imuodom_copy.front().timestamp;
                double imuodo_max_ros_time = imuodom_copy.back().timestamp;
                // EZLOG(INFO)<<"imuodo_min_ros_time"<<imuodo_max_ros_time - imuodo_min_ros_time<<endl;


                ///1.get cloud max and min time stamp to
                CloudTypeXYZIRTPtr cur_scan;
                cloud_mutex.lock();
                cur_scan = deque_cloud.front();

                CloudWithTime cloud_with_time;
                GetCloudTime(cur_scan, cloud_with_time);
                double cloud_min_ros_timestamp = cloud_with_time.min_ros_timestamp;
                double cloud_max_ros_timestamp = cloud_with_time.max_ros_timestamp;

                //              gnss lidar gnss---->>>>
                gnssins_mutex.lock();
                while (!poseQueue.empty()) {
                    if (poseQueue.front().timestamp > cur_scan->timestamp) {
                        cloudinfo.pose = poseQueue.front().pose;
                        poseQueue.clear();
                        break;
                    } else {
                        poseQueue.pop_front();
                    }
                }
                 gnssins_mutex.unlock();
//                while (!poseQueue.empty()) {
//                        gnssins_mutex.lock();
//                    if (poseQueue.front().timestamp< cur_scan->timestamp - 0.1){
//                         poseQueue.pop_front();
//                         gnssins_mutex.unlock();
//                         EZLOG(INFO)<<"get out addGps factor"<<endl;
//                    }
//                    else if (poseQueue.front().timestamp> cur_scan->timestamp + 0.1){
//                        gnssins_mutex.unlock();
//                        break;
//                    }else{
//                        cloudinfo.pose = poseQueue.front().pose;
//                        poseQueue.pop_front();
//                        gnssins_mutex.unlock();
//                    }
//                }

                EZLOG(INFO)<<"gnss result"<< cloudinfo.pose.GetXYZ()<<endl;
               // gnssins_mutex.unlock();
                ///2.
//                if(odo_min_ros_time>cloud_min_ros_timestamp){
//                    EZLOG(INFO)<<"odo is larger than lidar" <<endl;
//                    exit(1);
//                }
                if (!IMUOdomQueue.empty() && imuodo_min_ros_time >= cloud_min_ros_timestamp) {
                    auto temp = IMUOdomQueue.front();
                    temp.timestamp = cloud_min_ros_timestamp - 0.01f;
                    IMUOdomQueue.push_front(temp);
                }
                // double cur_lidar_time = deque_cloud.front()->timestamp;
                // EZLOG(INFO)<<"TIMESTAMP"<<cur_lidar_time-cloud_min_ros_timestamp<<endl;
                if (imuodo_min_ros_time < cloud_min_ros_timestamp &&
                    imuodo_max_ros_time > cloud_max_ros_timestamp) {//odo_min_ros_time<cloud_min_ros_timestamp&&
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

                    ///2.imgprojection
                    ///update rangemat and deskewCloud_body
                    double cost_time_projpc = ProjectPointCloud(cloud_with_time, imuodom_copy, T_w_l_lidar_first_pose);
                    //  EZLOG(INFO) << "cost_time_projpc(ms) = " << cost_time_projpc << std::endl;

                    ///3.cloudExtraction
                    double cost_time_cloudextraction = CloudExtraction(T_w_l_lidar_first_pose, cloudinfo);

                    ///4. send data to feature extraction node
                    OdometryType T_w_fist_pub;
                    T_w_fist_pub.frame = "map";
                    T_w_fist_pub.timestamp = cur_scan->timestamp;;
                    T_w_fist_pub.pose = T_w_l_lidar_first_pose;
                    pubsub->PublishOdometry(topic_lidar_align_gnss, T_w_fist_pub);

                    Function_AddCloudInfoToFeatureExtraction(cloudinfo);
                    EZLOG(INFO)
                            << "2. 1 of 2 data_preprocess send to feature_extraction! current lidar pointCloud size is: "
                            << cloudinfo.cloud_ptr->points.size();
                    ///5.pop used odom
                    imuodom_mutex.lock();

                    double thresh = cloud_max_ros_timestamp - 0.05f;
                    while (IMUOdomQueue.front().timestamp < thresh) {
                        IMUOdomQueue.pop_front();
                    }
                    imuodom_mutex.unlock();

                    ResetParameters();
                }//end function if
                else {
                    cloud_mutex.unlock();
                }

                ResetParameters();
                //  }//end if(deque_cloud.size()!=0){
                // else{
                //      sleep(0.01);//线程休息10ms
                //  }
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

//    void Udp_OdomPub(const PoseT& data){
//        Vis_Odometry odom_out;
//        std::string fu_str;
//        odom_out.type = "gn";
//        odom_out.t[0]= data.GetXYZ().x();
//        odom_out.t[1]= data.GetXYZ().y();
//        odom_out.t[2]= data.GetXYZ().z();
//
//        odom_out.q.x() = data.GetQ().x();
//        odom_out.q.y() = data.GetQ().y();
//        odom_out.q.z() = data.GetQ().z();
//        odom_out.q.w() = data.GetQ().w();
//
//        fu_str = odom_out.ToString();
//        udp_thread -> SendUdpMSg(fu_str);
//    }

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

         }
//         else{  //mapping
//             Function_AddGNSSOdometryTypeToOPTMapping(T_w_l_gnss);
//         }

            // TODO T_w_l_pub->>>>>>> is Gnss result need UDP
           // Udp_OdomPub(T_w_l);
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
        //udp_thread = udp_thread_;
        pubsub->addPublisher(topic_origin_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskew_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskw_cloud_to_ft_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_imuodom_curlidartime_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_lidar_align_gnss, DataType::ODOMETRY,2000);



        do_work_thread = new std::thread(&DataPreprocess::DoWork, this);
        EZLOG(INFO)<<"DataPreprocess init success!"<<std::endl;
    }


};
#endif

