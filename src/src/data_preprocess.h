
// Use the Velodyne point format as a common representation
#ifndef SEU_LIDARLOC_DATAPREPROCESS_H
#define SEU_LIDARLOC_DATAPREPROCESS_H

#define DBL_MAX		__DBL_MAX__//1118 what is this? TODO

#include <mutex>
#include <thread>
#include <fstream>
//3rdParty
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "opencv2/opencv.hpp"   // for opencv4
//homeMade
#include "pubsub/pubusb.h"
#include "feature_extraction.h"
#include "utils/MapSaver.h"
#include "utils/timer.h"
#include "utils/utility.h"
#include "utils/filesys.h"
#include "config/abs_current_path.h"
#include "front_end/front_end.h"
#include "utils/udp_thread.h"
#include "udp_helper_h/udp_seralize.h"
#include "udp_helper_h/udp_client.h"

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
    std::mutex drodom_mutex;
    std::mutex gnss_mutex;
    bool flag_first_gnss = 0;
    PoseT First_gnss_pose;

    std::thread* do_work_thread;

    std::deque<CloudTypeXYZIRTPtr> deque_cloud;
    std::deque<OdometryType> GNSSQueue;
    std::deque<OdometryType> DrOdomQueue;

    std::function<void(const CloudInfo&)> Function_AddCloudInfoToFeatureExtraction;
//    std::function<void(const GNSSOdometryType&)> Function_AddGNSSOdometryTypeToFuse;
    //   std::function<void(const GNSSOdometryType&)>Function_AddGNSSOdometryTypeToOPTMapping;
    std::function<void(const GNSSOdometryType&)>Function_AddFirstGNSSPoint2DR;

    // std::function<void(const OdometryType&)> Function_AddOdometryTypeToOPTMapping;

    bool init = false;
    GeographicLib::LocalCartesian geoConverter;
    int frame_id = 0;

    std::string topic_origin_cloud_world = "/origin_cloud_world";
    std::string topic_deskew_cloud_world = "/deskew_cloud_world";
    std::string topic_gnss_odom_world = "/gnss_odom_world";
    std::string topic_gnss_odom_world_origin = "/gnss_odom_world_origin";
    std::string topic_deskw_cloud_to_ft_world = "/deskw_cloud_to_ft_world";
    std::string topic_imuodom_curlidartime_world = "/imuodom_curlidartime_world";

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
            const double latency = cloud_origin->cloud.points[i].latency - t_0;
            cloud_origin->cloud.points[i].latency = latency;

            if(latency > max_timestamp){
                max_timestamp = latency;

            }
            if(latency < min_timestamp){
                min_timestamp = latency;
            }
        }

        cloud_with_time.cloud_ptr = cloud_origin;
        cloud_with_time.min_ros_timestamp = lidar_start_ros_time;
        cloud_with_time.max_ros_timestamp = max_timestamp - min_timestamp + lidar_start_ros_time;
        cloud_with_time.min_latency_timestamp = min_timestamp;

        double cost_time = timer.toc();
//        EZLOG(INFO)<<"GetCloudTime cost time(ms) = "<<cost_time<<std::endl;
        return cost_time;
    }

    //find closest Lidar pose  TODO 1111 ----------Done
    double  FindLidarPoseinDROdom(const CloudWithTime& cloudinfo, const std::deque<OdometryType>& pose_deque ,
                                  PoseT& T_w_l_lidar_start) {
        TicToc timer;

        //used
        for (int i = 1; i < (int) pose_deque.size(); i++) {  //遍历里程计队列，找到处于当前帧时间之前的第一个里程计数据作为起始位姿

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
                if(FrontEndConfig::if_debug)
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
    }//end FindLidarPoseinDROdom

    //find x y z in lidar coordinate,Q in gnss coordinate at the pointtime and calculate translation matrix
    //find lidar point pose TODO 1111 change function name to FindLidarPointPose -----------Done
    void FindLidarPointPose(const double pointTime,//lidar point time
                            const CloudWithTime& cloudinfo,
                            const std::deque<OdometryType>& pose_deque,
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
        FindLidarPointPose(pointTime, cloudinfo, pose_deque,
                           T_w_b_lidar_now);

        Eigen::Vector3d origin_pt;
        origin_pt << point->x,point->y,point->z;
        PoseT T_lidar_start_now = T_w_b_lidar_start.inverse()*T_w_b_lidar_now;
        //TODO !!!!!!!!!!!!!!!!!!!!inverse big bug   p_s = T_l_s_n * p_n
        Eigen::Vector3d deskewnewpoint = T_lidar_start_now * origin_pt;

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
            if (SensorConfig::use_drodom_deskew){
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
            //TODO 1111 delete!!!!! if(debug)----------Done
            if(FrontEndConfig::if_debug) {
                auto thisPoint_offset = thisPoint;
                thisPoint_offset.z = thisPoint_offset.z + 20.0;
                deskewCloud_body_offset.points.push_back(thisPoint_offset);
            }
            valid_num++;
        }//end for

////        use for debug
//        EZLOG(INFO)<<"valid_num num = "<<valid_num<<std::endl;
//        EZLOG(INFO)<<"range_outlier num = "<<range_outlier<<std::endl;
//        EZLOG(INFO)<<"row_outlier num = "<<row_outlier<<std::endl;
//        EZLOG(INFO)<<"col_outlier num = "<<col_outlier<<std::endl;
//        EZLOG(INFO)<<"rangemat_outlier num = "<<rangemat_outlier<<std::endl;

////        for debug use
        if(FrontEndConfig::if_debug)
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

    //TODO 1111 remove T_w_l----------Done
    double CloudExtraction(CloudInfo& cloudInfo) {

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

        return timer.toc();
    }

    void DoWork() {
        CloudTypeXYZIRTPtr cur_scan;
        CloudWithTime cur_cloud_time;
        double cloud_min_ros_timestamp = 0.0;
        double cloud_max_ros_timestamp = 0.0;
        bool isGetCurrentCloud = false;
        bool isGetCorrespondingGNSS = false;
        long long int GNSS_frames = 0;

        while (1) {
            CloudInfo cloudinfo;//TODO
            cloudinfo.startRingIndex.assign(SensorConfig::N_SCAN,
                                            0);//为cloudInfo中的startRingIndex、endRingIndex向量分配大小N_SCAN,并初始化为0
            cloudinfo.endRingIndex.assign(SensorConfig::N_SCAN, 0);

            {
                sleep(0.005);
                std::lock_guard<std::mutex> lock(cloud_mutex);
                if(deque_cloud.empty()){
                    continue;
                }
            }


            ///0.do something
            double drqueue_min_ros_time ;
            double drqueue_max_ros_time ;
            //TODO 1111
            {
                std::lock_guard<std::mutex> lock(drodom_mutex);
                if(DrOdomQueue.empty()){
                    continue;
                }
                drqueue_min_ros_time = DrOdomQueue.front().timestamp;
                drqueue_max_ros_time = DrOdomQueue.back().timestamp;
            }

            ///1.get cloud max and min time stamp to
//                TicToc time_getin;
            if(!isGetCurrentCloud)
            {
                cloud_mutex.lock();
                cur_scan = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();//unlock TODO 1111

                GetCloudTime(cur_scan, cur_cloud_time);
                cloud_min_ros_timestamp = cur_cloud_time.min_ros_timestamp;
                cloud_max_ros_timestamp = cur_cloud_time.max_ros_timestamp;
//                    time_getin.tic();
                isGetCurrentCloud = true;

            }

            //gnss related!
            if(!isGetCorrespondingGNSS)
            {
                cloudinfo.pose_reliable = false;
                std::lock_guard<std::mutex> lock(gnss_mutex);
                if(!GNSSQueue.empty()){
                    for(int i = 0;i<GNSSQueue.size();++i){
                        if(abs(GNSSQueue[i].timestamp - cur_scan->timestamp) < FrontEndConfig::gnss_align_threshold){
                            cloudinfo.pose = GNSSQueue[i].pose;
                            cloudinfo.cov = GNSSQueue[i].cov;
                            cloudinfo.pose_reliable = true;
                            isGetCorrespondingGNSS = true;
                            break;
                        }
                    }
                    while(!GNSSQueue.empty() && GNSSQueue.front().timestamp - cur_scan->timestamp < FrontEndConfig::gnss_pop_threshold ){
                        GNSSQueue.pop_front();
                    }
                }
            }

            ///2.
            //use absolutue
            //TODO 1118 delete and test!
//                if (!DrOdomQueue.empty() && drqueue_min_ros_time >= cloud_min_ros_timestamp) {
//                    auto temp = DrOdomQueue.front();
//                    temp.timestamp = cloud_min_ros_timestamp - 0.01f;
//                    DrOdomQueue.push_front(temp);//TODO 1118 add lock
//                    //isGetCurrentCloud = false;
//                }
            static int pop_lidar_scan_num = 0; //if time_dr > time_lidar, pop lidar
            if(drqueue_min_ros_time > cloud_min_ros_timestamp){
                ++pop_lidar_scan_num;
                if(pop_lidar_scan_num % 5 == 0)
                    EZLOG(INFO)<< "Pop current lidar scan! " << pop_lidar_scan_num <<" / "<<frame_id<<" = "<<(float)pop_lidar_scan_num/(float)frame_id <<std::endl;
                isGetCurrentCloud = false;
                isGetCorrespondingGNSS = false;
                continue;
            }

            //TODO 1118, add else to test
            if(drqueue_max_ros_time < cloud_max_ros_timestamp){
                continue;
            }

            TicToc time_dataprep;
            TicToc time_dataprep_1;
            isGetCurrentCloud = false;
            isGetCorrespondingGNSS = false;

            std::deque<OdometryType> drqueue_copy;
            drodom_mutex.lock();
            drqueue_copy = DrOdomQueue;
            drodom_mutex.unlock();

            cloudinfo.frame_id = frame_id++;
            cloudinfo.timestamp = cur_scan->timestamp;
          //  EZLOG(INFO)<<"time_dataprep_1.toc() = "<<time_dataprep_1.toc()<<endl;

            if(cloudinfo.pose_reliable == false || frame_id % 100 == 0){
                EZLOG(INFO)<<"find gnss pose num = "<<GNSS_frames<<" / "<<frame_id<<" = "<<(float)GNSS_frames/(float)frame_id;
            }
            ++GNSS_frames;

            /// 1.FindLidarFirstPose
            PoseT T_w_l_lidar_first_pose;
            double cost_time_findpose = FindLidarPoseinDROdom(cur_cloud_time, drqueue_copy,//in
                                                              T_w_l_lidar_first_pose);//out
           // EZLOG(INFO) << "cost_time_findpose(ms) = " << cost_time_findpose << std::endl;

            cloudinfo.DRPose = T_w_l_lidar_first_pose;
            Eigen::Vector3d t_w_cur;
            Eigen::Quaterniond q_w_cur;
            Eigen::Matrix3d q_w_cur_matrix;//TODO remove?
            t_w_cur = T_w_l_lidar_first_pose.GetXYZ();
            q_w_cur = T_w_l_lidar_first_pose.GetQ();
            q_w_cur.normalize();

            ///2.imgprojection
            ///update rangemat and deskewCloud_body
            double cost_time_projpc = ProjectPointCloud(cur_cloud_time, drqueue_copy, T_w_l_lidar_first_pose);
             // EZLOG(INFO) << "cost_time_projpc(ms) = " << cost_time_projpc << std::endl;

            ///3.cloudExtraction
            double cost_time_cloudextraction = CloudExtraction(cloudinfo);
            //  EZLOG(INFO)<<"cost_time_cloudextraction(ms) = "<<cost_time_cloudextraction<<std::endl;

            //        //for debug use
            if(FrontEndConfig::if_debug)
            {
                CloudTypeXYZICOLRANGE cloud_pub;
                cloud_pub.timestamp = cloudinfo.timestamp;
                cloud_pub.frame = "map";
                pcl::transformPointCloud(*cloudinfo.cloud_ptr, cloud_pub.cloud, T_w_l_lidar_first_pose.pose.cast<float>());
                pubsub->PublishCloud(topic_deskw_cloud_to_ft_world, cloud_pub);
            }

            TicToc time_dataprep_2;
            ///4. send data to feature extraction node
            Function_AddCloudInfoToFeatureExtraction(cloudinfo);
           // EZLOG(INFO)<<"time_dataprep_2 = "<<time_dataprep_2.toc()<<endl;
           // EZLOG(INFO)<<"CloudInfo size:"<<cloudinfo.cloud_ptr -> points.size()
           // <<" "<< cloudinfo.cloud_ground_down -> points.size()
           // <<" "<< cloudinfo.cloud_ground -> points.size()
           // <<" "<< cloudinfo.cloud_unground -> points.size()<<endl;

//                    EZLOG(INFO)<<"cloudinfo.frame_id"<< cloudinfo.frame_id<<endl;
//                    EZLOG(INFO)
//                            << "2. 1 of 2 data_preprocess send to feature_extraction! current lidar pointCloud size is: "
//                            << cloudinfo.cloud_ptr->points.size();
            ///5.pop used odom
            TicToc time_dataprep_3;
            const double thresh = cloud_max_ros_timestamp - 0.05f;
            drodom_mutex.lock();
            while(DrOdomQueue.front().timestamp < thresh){
                DrOdomQueue.pop_front();
            }
            drodom_mutex.unlock();
            //EZLOG(INFO)<<"time_dataprep_3 = "<<time_dataprep_3.toc()<<endl;

           // EZLOG(INFO)<<"time_dataprep = "<<time_dataprep.toc()<<endl;

            ResetParameters();
            //end function if
            //TODO 1118

        }//end function while(1)
    }

    void AddCloudData(const CloudTypeXYZIRT& data){

        CloudTypeXYZIRTPtr cloud_ptr(new CloudTypeXYZIRT());
        *cloud_ptr = data;//深拷贝
        cloud_mutex.lock();
        deque_cloud.push_back(cloud_ptr);
        cloud_mutex.unlock();
        lidarScan_cnt =0;
    }


    void Udp_OdomPub(const PoseT& data, const std::string gnss_type){
        Vis_Odometry odom_out;
        std::string fu_str;
        odom_out.type = gnss_type;
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

    bool GnssQualityCheck(const GNSSINSType& data){
//        EZLOG(INFO)<<setprecision(14)<<data.timestamp<<" data.gps_status: "<<data.gps_status;
        if(data.lla[0]<0 || data.lla[1]<0 || data.lla[2]<0){
            //EZLOG(INFO)<<"Bad GNSS in DataPre,drop!";
            return false;
        }
        if(data.gps_status != 42){
            EZLOG(INFO)<<"Bad GNSS value in DataPre,drop!: "<<data.lla.transpose() ;
            return false;
        }
        return true;
    }
    //
    void AddGNSSINSSData(const GNSSINSType& data){

       if(!GnssQualityCheck(data)){
           return ;
       }

        if(!init)
        {
            double x,y,z;
            if(FrontEndConfig::slam_mode_switch == SLAM_MODE::OPT_LOCATION){//loc
                std::ifstream downfile(LocConfig::save_map_path+"Origin.txt");  //打开文件
                std::string line; //字符串
                std::getline(downfile, line);//
                std::istringstream iss(line);
                iss >> x >> y >> z;
                downfile.close(); // 关闭文件
                geoConverter.Reset(x, y, z);
            }
            else if(FrontEndConfig::slam_mode_switch == SLAM_MODE::OPT_MAPPING){//mapping
                geoConverter.Reset(data.lla[0], data.lla[1], data.lla[2]);
                MapSaver::SaveOriginLLA(data.lla);
           //     EZLOG(INFO)<<"save first GNSS points: "<<data.lla[0]<<", "<<data.lla[1]<<", "<<data.lla[2];
            }
            init = true;
            return;
        }

        double t_enu[3];
        geoConverter.Forward(data.lla[0], data.lla[1], data.lla[2],
                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate

        Eigen::Matrix3d z_matrix;//calculate Quaternion
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = -data.yaw * 0.017453293;
        double pitch_Y = data.pitch * 0.017453293;
        double roll_X = data.roll * 0.017453293;
        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                sin(heading_Z), cos(heading_Z),  0,
                0,                 0,            1;

        x_matrix << 1,                 0,              0,
                0,            cos(pitch_Y),     -sin(pitch_Y),
                0,            sin(pitch_Y),    cos(pitch_Y);

        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                0,                 1,               0 ,
                -sin(roll_X),      0,         cos(roll_X);

        Eigen::Matrix3d R_w_b =  (z_matrix*x_matrix*y_matrix);   // Pw = Twb * Pb zxy右前上，zyx前左上
        Eigen::Vector3d t_w_b(t_enu[0], t_enu[1], t_enu[2]);
        Eigen::Quaterniond q_w_b(R_w_b);//获得局部坐标系的四元数
        PoseT T_w_b(t_w_b, R_w_b);

        OdometryType T_w_b_pub_origin;
        T_w_b_pub_origin.frame = "map";
        T_w_b_pub_origin.timestamp = data.timestamp;
        T_w_b_pub_origin.pose = T_w_b;

        pubsub->PublishOdometry(topic_gnss_odom_world_origin, T_w_b_pub_origin);
        PoseT T_w_l = PoseT(T_w_b.pose*(SensorConfig::T_L_B.inverse())); // Pw = Twb * Tbl * Pl
        OdometryType T_w_l_pub;
        T_w_l_pub.frame = "map";
        T_w_l_pub.timestamp = data.timestamp;
        T_w_l_pub.pose = T_w_l;
        T_w_l_pub.cov[0] =  data.lla_sigma[0];
        T_w_l_pub.cov[1] =  data.lla_sigma[1];
        T_w_l_pub.cov[2] =  data.lla_sigma[2];
        T_w_l_pub.cov[3] =  data.rpy_sigma[0];
        T_w_l_pub.cov[4] =  data.rpy_sigma[1];
        T_w_l_pub.cov[5] =  data.rpy_sigma[2];
        T_w_l_pub.GTpose_reliability = false;

        if(FrontEndConfig::if_debug) {
            pubsub->PublishOdometry(topic_gnss_odom_world, T_w_l_pub);
        }

        if(IsFileDirExist(ABS_CURRENT_SOURCE_PATH+"/flag_gnss")) { // if
            if(data.gps_status == 42){
                T_w_l_pub.GTpose_reliability = true;
            }
            gnss_mutex.lock();
            GNSSQueue.push_back(T_w_l_pub);//TODO Receive DR     Done--receive gnss to align with lidar
            gnss_mutex.unlock();
            if (LocConfig::slam_mode_on == 1) {
                Udp_OdomPub(T_w_l,"gn");
            }
        }
        else{
        //TODO
//            EZLOG(INFO)<<"CONTROL! NO GNSS!"<<ABS_CURRENT_SOURCE_PATH+"/flag_gnss";
            Udp_OdomPub(T_w_l,"ng");
//            exit(-1);
        }

    }

    void AddDrOdomData(const OdometryType& data){
        drodom_mutex.lock();
        DrOdomQueue.push_back(data);
        drodom_mutex.unlock();
    }

    // UDP
    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_ = nullptr){
        AllocateMemory();
        pubsub = pubsub_;
        udp_thread = udp_thread_;

        //TODO add MDC compile #ifdefine
        pubsub->addPublisher(topic_origin_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskew_cloud_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_deskw_cloud_to_ft_world,DataType::LIDAR,1);
        pubsub->addPublisher(topic_gnss_odom_world, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_gnss_odom_world_origin, DataType::ODOMETRY,2000);
        pubsub->addPublisher(topic_imuodom_curlidartime_world, DataType::ODOMETRY,2000);

        do_work_thread = new std::thread(&DataPreprocess::DoWork, this);
        EZLOG(INFO)<<"DataPreprocess init success!"<<std::endl;
    }

};
#endif

