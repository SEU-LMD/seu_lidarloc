//
// Created by fyy on 23-9-10.
//

#ifndef SEU_LIDARLOC_OPT_MAPPING_H
#define SEU_LIDARLOC_OPT_MAPPING_H

#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/dataset.h"  // gtsam

#include <std_srvs/Empty.h>
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "imu_preintegration.h"
//#include "GeoGraphicLibInclude/Geocentric.hpp"
//#include "GeoGraphicLibInclude/LocalCartesian.hpp"
//#include "GeoGraphicLibInclude/Geoid.hpp"

#include "easylogging++.h"
#include "utils/utm/utm_convert.h"
#include "utils/timer.h"
#include "utils/MapSaver.h"



class OPTMapping{
public:
    PubSubInterface* pubsub;
    std::thread* do_work_thread;
    std::thread* loop_thread;
    std::thread* save_Map_thread;
    std::thread* save_path_thread;
    std::mutex cloud_mutex;
    std::mutex gnss_ins_mutex;
    std::mutex timer_mutex;
    TicToc timer_cloud;
    MapSaver map_saver;

    //    IMUPreintegration* imu_pre_ptr;
    std::function<void(const OdometryType&)> Function_AddOdometryTypeToIMUPreintegration;

    std::deque<CloudFeature> deque_cloud;
    std::deque<GNSSINSType> deque_gnssins;

//    std::string topic_map_surf = "/map_cloud_surf_global";
//    std::string topic_map_corner = "/map_cloud_corner_global";
    std::string topic_current_pose = "/current_pose";
//    std::string topic_gnss_pose = "/gnss_pose";


    vector <pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector <pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector <pcl::PointCloud<PointType>::Ptr> laserCloudRawKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;// history keyframe
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses2D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;


    pcl::PointCloud<PointType>::Ptr globalCornerCloud;
    pcl::PointCloud<PointType>::Ptr globalCornerCloudDS;
    pcl::PointCloud<PointType>::Ptr globalSurfCloud;
    pcl::PointCloud<PointType>::Ptr globalSurfCloudDS;


    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // downsampled corner feature set from
    pcl::PointCloud<PointType>::Ptr  laserCloudSurfLastDS;  // downsampled surf feature set from

    pcl::PointCloud<PointType>::Ptr laserCloudRaw;    // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS;  // giseop
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>>laserCloudMapContainer;


    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;        // for surrounding key poses of
    pcl::VoxelGrid<PointType> downSizeFilterRaw;

    std::vector <Eigen::Matrix4d> keyframePosestrans;
   // std::vector <nav_msgs::Odometry> keyframeRawOdom;
   // std::vector<double> keyframeTimes;
    //std::vector<sensor_msgs::PointCloud2> keyframeCloudDeskewed;
//    std::vector<double> keyframeDistances;

    std::vector<PointType>laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType>laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    //std::unique_ptr<MapSaver> dataSaverPtr;
   // dataSaverPtr = std::make_unique<MapSaver>();

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Vector3d t_w_cur;
    Eigen::Quaterniond q_w_cur;
    Eigen::Matrix3d q_w_cur_matrix;

    vector <pair<int, int>> loopIndexQueue;
    vector <gtsam::Pose3> loopPoseQueue;
    vector <gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque <std_msgs::Float64MultiArray> loopInfoVec;
    vector<PoseT> opt_poses;
    map<int, int> loopIndexContainer;  // from new to old
    map<int, int> gpsIndexContainer;

    bool flg_exit = false;
    bool aLoopIsClosed = false;
    bool timer_start = false;

    int lastLoopIndex = -1;
    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;
    int current_frame_id;
    int frame_id = -1;

    float q_w_roll, q_w_pitch, q_w_yaw;
    float transformTobeMapped[6]; // roll pitch yaw x y z

    gtsam::Pose3 last_gnss_poses;
    gtsam::ISAM2 *isam;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::Values isamCurrentEstimate;
    //PoseT latest_Estimate_lidar; // 所有关键帧位姿的优化结果
    Eigen::MatrixXd poseCovariance;

    double timeLaserInfoCur;
    bool isDegenerate = false;
    cv::Mat matP;

    Eigen::Vector3d originLLA;
    bool systemInitialized = false;
    bool gpsTransfromInit = false;
    GeographicLib::LocalCartesian geoConverter;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxGpsInfo;
    std::mutex mtxGraph;
//    nav_msgs::Path globalPath;

    void AddCloudData(const CloudFeature& cloud_ft){
        EZLOG(INFO)<<"optmapping_AddCloudData "<<std::endl;
        cloud_mutex.lock();
        deque_cloud.push_back(cloud_ft);
        cloud_mutex.unlock();
        EZLOG(INFO)<<"deque_cloud.size() = "<<deque_cloud.size()<<std::endl;
    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){
        gnss_ins_mutex.lock();
        deque_gnssins.push_back(gnss_ins_data);
        gnss_ins_mutex.unlock();
    }

    void allocateMemory() {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        globalCornerCloud.reset(new pcl::PointCloud<PointType>());
        globalCornerCloudDS.reset(new pcl::PointCloud<PointType>());
        globalSurfCloud.reset(new pcl::PointCloud<PointType>());
        globalSurfCloudDS.reset(new pcl::PointCloud<PointType>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());  // corner feature set from
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());  // surf feature set from
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled corner featuer set
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
        laserCloudRaw.reset(new pcl::PointCloud<PointType>());    // giseop
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>());  // giseop

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        coeffSelCornerVec.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        laserCloudOriCornerFlag.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        laserCloudOriSurfVec.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        coeffSelSurfVec.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);
        laserCloudOriSurfFlag.resize(SensorConfig::N_SCAN * SensorConfig::Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
                  false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(),
                  false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        downSizeFilterCorner.setLeafSize(MappingConfig::mappingCornerLeafSize,
                                         MappingConfig::mappingCornerLeafSize,
                                         MappingConfig::mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(MappingConfig::mappingSurfLeafSize,
                                       MappingConfig::mappingSurfLeafSize,
                                       MappingConfig::mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(MappingConfig::mappingSurfLeafSize,
                                         MappingConfig::mappingSurfLeafSize,
                                      MappingConfig::mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(MappingConfig::surroundingKeyframeDensity,
                                                      MappingConfig::surroundingKeyframeDensity,
                                                      MappingConfig::surroundingKeyframeDensity);
        const float rawMapFilterSize = 0.5;
        downSizeFilterRaw.setLeafSize(rawMapFilterSize, rawMapFilterSize,
                                      rawMapFilterSize);


        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po) {
        po->x = transPointAssociateToMap(0, 0) * pi->x +
                transPointAssociateToMap(0, 1) * pi->y +
                transPointAssociateToMap(0, 2) * pi->z +
                transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x +
                transPointAssociateToMap(1, 1) * pi->y +
                transPointAssociateToMap(1, 2) * pi->z +
                transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x +
                transPointAssociateToMap(2, 1) * pi->y +
                transPointAssociateToMap(2, 2) * pi->z +
                transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                        PointTypePose *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(
                transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
                transformIn->pitch, transformIn->yaw);

//#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i) {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                                    transCur(0, 1) * pointFrom.y +
                                    transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                                    transCur(1, 1) * pointFrom.y +
                                    transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                                    transCur(2, 1) * pointFrom.y +
                                    transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
        return gtsam::Pose3(
                gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch),
                                    double(thisPoint.yaw)),
                gtsam::Point3(double(thisPoint.x), double(thisPoint.y),
                              double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[]) {
        return gtsam::Pose3(
                gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint) {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                                      thisPoint.roll, thisPoint.pitch,
                                      thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) {
        return pcl::getTransformation(transformIn[3], transformIn[4],
                                      transformIn[5], transformIn[0],
                                      transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[]) {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }



    //如果5秒内没有接收到点云数据，就会自动保存
    // isam
//    void savePathThread() {
//        while (1) {
//
//            //保证有了位姿数据
//            if (timer_start == true) {
//
//                timer_mutex.lock();
//                double delta_time = timer_cloud.toc();
//                timer_mutex.unlock();
//                //EZLOG(INFO) << "delta_time = " << delta_time << std::endl;
//                if (delta_time > 5000) {
//                   map_saver.SavePoses(opt_poses);
//                    EZLOG(INFO) << "Saving poses completed! " << endl;
//                   // while (1);
//                }
//            }//end if(delta_time>5000)
//            else{
//                sleep(0.01);
//            }
//
//        }
//    }

//    void loopClosureThread() {
//        if (MappingConfig::loopClosureEnableFlag == false) return;
//        设置回环检测的频率 loopClosureFrequency默认为 1hz
//        ros::Rate rate(MappingConfig::loopClosureFrequency);
//        while (ros::ok()) {
//            ros::spinOnce();
//
//            performLoopClosure();
//            visualizeLoopClosure();
//
//            if (SensorConfig::useGPS) visualGPSConstraint();
//
//            rate.sleep();
//        }
//    }

    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg) {
        std::lock_guard <std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2) return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5) loopInfoVec.pop_front();
    }

 //   void performLoopClosure() {
//        if (cloudKeyPoses3D->points.empty() == true) return;
//
//        mtx.lock();
//        //把存储关键帧额位姿的点云copy出来，避免线程冲突
//        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
//        *copy_cloudKeyPoses2D = *cloudKeyPoses3D;
//        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
//        mtx.unlock();
//
//        // find keys
//        int loopKeyCur;
//        int loopKeyPre;
//
//        //外部通知的回环信息
//        //if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
//        //在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
//        if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;
//
//        // extract cloud
//        //检测回环存在后则可以计算检测出这两帧的位姿变换
//        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
//                new pcl::PointCloud<PointType>());//声明当前关键帧的点云
//        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(
//                new pcl::PointCloud<PointType>());//和历史回环帧周围的点云（局部地图）
//        {
//            //回环帧把自己提取出来
//            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
//            //回环帧把自己周围一些点云取出来，也就是构成一个帧局部地图的一个匹配问题
//            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
//                                  MappingConfig::historyKeyframeSearchNum);
//            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
//                return;
//        }
//        //现在有了当前关键帧投到地图坐标系下的点云和历史回环帧投到地图坐标系下的局部地图，那么接下来就可以进行两者的icp位姿变换求解
//        // ICP Settings
//        static pcl::IterativeClosestPoint <PointType, PointType> icp;//使用简单的icp来进行帧到局部地图的配准
//        //设置最大相关距离 15m
//        icp.setMaxCorrespondenceDistance(MappingConfig::historyKeyframeSearchRadius * 2);
//        //最大优化次数
//        icp.setMaximumIterations(100);
//        //单次变换范围
//        icp.setTransformationEpsilon(1e-6);
//        //残差设置
//        icp.setEuclideanFitnessEpsilon(1e-6);
//        icp.setRANSACIterations(0);
//
//        // Align clouds
//        //设置两个点云
//        icp.setInputSource(cureKeyframeCloud);
//        icp.setInputTarget(prevKeyframeCloud);
//        //执行配准
//        pcl::PointCloud<PointType>::Ptr unused_result(
//                new pcl::PointCloud<PointType>());
//        icp.align(*unused_result);
//        //检测icp是否收敛 且 得分是否满足要求
//        if (icp.hasConverged() == false ||
//            icp.getFitnessScore() > MappingConfig::historyKeyframeFitnessScore)
//            return;
//
//        // publish corrected cloud
//        //把修正后的当前点云发布供可视化使用
////        if (pubIcpKeyFrames.getNumSubscribers() != 0) {
////            pcl::PointCloud<PointType>::Ptr closed_cloud(
////                    new pcl::PointCloud<PointType>());
////            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
////                                     icp.getFinalTransformation());
////            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp,
////                         SensorConfig::odometryFrame);
////        }
//
//        // Get pose transformation
//        float x_lidar, y_lidar, z_lidar, roll_lidar, yaw_lidar, pitch_lidar;
//        Eigen::Affine3f correctionLidarFrame;
//        //获得两个点云的变换矩阵结果，
//        correctionLidarFrame = icp.getFinalTransformation();
//        // transform from world origin to wrong pose
//        //取出当前帧的位姿
//        Eigen::Affine3f tWrong =
//                pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
//        // transform from world origin to corrected pose
//        //将icp结果补偿过去，就是当前帧的更为准确的位姿结果
//        Eigen::Affine3f tCorrect =
//                correctionLidarFrame *
//                tWrong;  // pre-multiplying -> successive rotation about a fixed frame
////          Eigen::Matrix4f map_2_world = SensorConfig::T_L_B.cast<float>(); //T_wm
////          Eigen::Matrix4f tCorrect_world = map_2_world * tCorrect.matrix();// T_wl = T_wm * T_ml
////          Eigen::Transform<float, 3, Eigen::Affine> a3f_transform (tCorrect_world);
//
//        pcl::getTranslationAndEulerAngles(tCorrect, x_lidar, y_lidar, z_lidar,roll_lidar,pitch_lidar,yaw_lidar);
//
//        //将当前帧补偿后的位姿 转换成 gtsam的形式
//        //From 和 To相当于帧间约束的因子，To是历史回环帧的位姿
//        gtsam::Pose3 poseFrom =
//                gtsam::Pose3(gtsam::Rot3::RzRyRx(roll_lidar, pitch_lidar, yaw_lidar), gtsam::Point3(x_lidar, y_lidar, z_lidar));
//
//        gtsam::Pose3 poseTo =
//                pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
//
//        //使用icp的得分作为他们的约束噪声项
//        gtsam::Vector Vector6(6);
//        float noiseScore = icp.getFitnessScore();
//        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
//                noiseScore;
//        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise =
//                gtsam::noiseModel::Diagonal::Variances(Vector6);
//
//        // Add pose constraint
//        //将两帧索引，两帧相对位姿和噪声作为回环约束 送入对列
//        mtx.lock();
//        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));//两帧索引
//        loopPoseQueue.push_back(poseFrom.between(poseTo));//当前帧与历史回环帧的相对位姿
//        loopNoiseQueue.push_back(constraintNoise);//
//        mtx.unlock();
//
//        // add loop constriant
//        //保存已经存在的约束对
//        loopIndexContainer[loopKeyCur] = loopKeyPre;
//        lastLoopIndex = loopKeyCur;
 //   }

     //kdtree问题
 //   bool detectLoopClosureDistance(int *latestID, int *closestID) {
        //检测最新帧是否和其它帧形成回环
//        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
//        int loopKeyPre = -1;
//
//        // check loop constraint added before
//        //检查一下较晚帧是否和别的形成了回环，如果有就算了
//        //因为当前帧刚刚出现，不会和其它帧形成回环，所以基本不会触发
//        auto it = loopIndexContainer.find(loopKeyCur);
//        if (it != loopIndexContainer.end()) return false;
//
//        // tricks
//        // Two consecutive loop edges represent the closed loop of the same scene.
//        // Adding all of them to the pose graph has little meaning and may reduce
//        // the accuracy.
//
//        if (abs(lastLoopIndex - loopKeyCur) < 5 && lastLoopIndex != -1)
//            return false;
//
//        // tricks
//        // sometimes we need to find the corressponding loop pairs
//        // but we do not need to care about the z values of these poses.
//        // Pls note that this is not work for stair case
//        //b不需要关注z值，将其设置为0
//        for (int i = 0; i < copy_cloudKeyPoses2D->size(); ++i) {
//            copy_cloudKeyPoses2D->at(i).z = 0;
//        }
//
//        // find the closest history key frame
//        std::vector<int> pointSearchIndLoop;
//        std::vector<float> pointSearchSqDisLoop;
//
//        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
//        //根据最后一个关键帧的平移信息，寻找离他一定距离内的其它关键帧，搜索范围15米
//        kdtreeHistoryKeyPoses->radiusSearch(
//                copy_cloudKeyPoses2D->back(), MappingConfig::historyKeyframeSearchRadius,
//                pointSearchIndLoop, pointSearchSqDisLoop, 0);
//        //
//        for (int i = 0; i < (int) pointSearchIndLoop.size(); ++i) {
//            int id = pointSearchIndLoop[i];
//            //历史帧，必须比当前帧间隔30s以上
//            if (abs(copy_cloudKeyPoses6D->points[id].time- timeLaserInfoCur) >
//                MappingConfig::historyKeyframeSearchTimeDiff) {
//                loopKeyPre = id;
//                break;
//            }
//        }
//        //如果没有找到回环或者回环找到自己身上去了，就认为是本次回环寻找失败
//        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
//
//        // we also need to care about the accumulated distance between keyframe;
//        // LOOPs that are too close together have no meaning and may reduce
//        // accuracy. For example, the lidar starts to move after being stationary
//        // for 30s in a certain place. At this time, the IMU should be trusted more
//        // than the lidar.
//        //关键帧间的距离应该大于12
//        if (keyframeDistances.size() >= loopKeyCur) {
//            double distance = 0.0;
//            for (int j = loopKeyPre; j < loopKeyCur; ++j) {
//                distance += keyframeDistances.at(j);
//            }
//            if (distance < 12) {
//                std::cout << "CLOSE FRAME MUST FILTER OUT " << distance << std::endl;
//                return false;
//            }
//        }
//        //找到了当前关键帧和历史回环帧，赋值当前帧和历史回环帧的id;
//        *latestID = loopKeyCur;
//        *closestID = loopKeyPre;
//
//        return true;
 //   }

//    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
//                               const int &key, const int &searchNum) {
//        // extract near keyframes
//        nearKeyframes->clear();
//        int cloudSize = copy_cloudKeyPoses6D->size();
//        //通过-searchNum 到 +searchNum遍历帧的搜索范围
//        for (int i = -searchNum; i <= searchNum; ++i) {
//            int keyNear = key + i;
//            if (keyNear < 0 || keyNear >= cloudSize) continue;
//            //把对应角点和面点的点云转到世界坐标系下去
//            *nearKeyframes +=
//                    *transformPointCloud(cornerCloudKeyFrames[keyNear],
//                                         &copy_cloudKeyPoses6D->points[keyNear]);
//            *nearKeyframes += *transformPointCloud(
//                    surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
//        }
//
//        if (nearKeyframes->empty()) return;
//
//        // downsample near keyframes
//        //把点云下采样，存放到cloud_temp
//        pcl::PointCloud<PointType>::Ptr cloud_temp(
//                new pcl::PointCloud<PointType>());
//        downSizeFilterICP.setInputCloud(nearKeyframes);
//        downSizeFilterICP.filter(*cloud_temp);
//        *nearKeyframes = *cloud_temp;
//    }

    void updateInitialGuess() {
        //每帧激光初位姿,将四元数转换到欧拉角
        TicToc timer;

        // initialization the first frame
        if (cloudKeyPoses3D->points.empty()) {
            systemInitialized = false;

            if (SensorConfig::useGPS) {
                ROS_INFO("GPS use to init pose");
                //拿到局部坐标系下GNSS的原点位置
                 double t_enu[3];
                geoConverter.Forward(deque_gnssins.front().lla[0], deque_gnssins.front().lla[1], deque_gnssins.front().lla[2],
                                     t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate

               if (SensorConfig::debugGps) {
                    //std::cout << "initial gps yaw: " << q_w_yaw_cur << std::endl;
                   // std::cout << "GPS Position: " << t_enutranspose() << std::endl;
                    //std::cout << "GPS LLA: " << originLLA.transpose() << std::endl;
                    ROS_WARN("GPS init success");
                }

                /** add the first factor, we need this origin GPS point for prior map based localization,
                    * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                PointType gnssPoint;
                gnssPoint.x = t_enu[0],
                gnssPoint.y = t_enu[1],
                gnssPoint.z = t_enu[2];
                //?????????  协方差怎么拿过来
                double noise_x = 0.00001;
                double noise_y = 0.00001;
                double noise_z = 0.00001;
                /** if we get reliable origin point, we adjust the weight of this gps factor */
                if (!SensorConfig::updateOrigin) {
                    noise_x *= 1e-4;
                    noise_y *= 1e-4;
                    noise_z *= 1e-4;
                }
                gtsam::Vector Vector3(3);
                Vector3 << noise_x, noise_y, noise_z;
                gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                        gtsam::noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(
                        0, gtsam::Point3(gnssPoint.x, gnssPoint.y, gnssPoint.z),
                        gps_noise);
                keyframeGPSfactor.push_back(gps_factor);
                cloudKeyGPSPoses3D->points.push_back(gnssPoint);

                transformTobeMapped[0] = q_w_roll;
                transformTobeMapped[1] = q_w_pitch;
                transformTobeMapped[2] = q_w_yaw;

                systemInitialized = true;

            }
            else{
                systemInitialized = true;
                return;
            }

        }

        if (!systemInitialized) {
            ROS_ERROR("sysyem need to be initialized");
            return;
        }

        /***
         * transback 可以直接给值
         */

        Eigen::Affine3f transBack = pcl::getTransformation(
                t_w_cur[0], t_w_cur[1], t_w_cur[2], q_w_roll, q_w_pitch,
                q_w_yaw);
            pcl::getTranslationAndEulerAngles(
                    transBack, transformTobeMapped[3], transformTobeMapped[4],
                    transformTobeMapped[5], transformTobeMapped[0],
                    transformTobeMapped[1], transformTobeMapped[2]);

//            EZLOG(INFO) << "Translation x" << transformTobeMapped[3] << std::endl;
//            EZLOG(INFO) << "Translation y" << transformTobeMapped[4] << std::endl;
//            EZLOG(INFO) << "Translation z" << transformTobeMapped[5] << std::endl;
//            EZLOG(INFO) << "roll" << transformTobeMapped[0] << std::endl;
//            EZLOG(INFO) << "pitch" << transformTobeMapped[1] << std::endl;
//            EZLOG(INFO) << "yaw" << transformTobeMapped[2] << std::endl;
            double updateinit_cost_time = timer.toc();
            EZLOG(INFO)<<"updateinit_cost_time"<<updateinit_cost_time<<endl;
            //ROS_WARN("transformtobemapped success！！！");

        return;
    }

    void extractNearby() {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(
                new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        //kdtree的输入,历史所有关键帧位姿集合(全局关键帧位姿集合)
        kdtreeSurroundingKeyPoses->setInputCloud(
                cloudKeyPoses3D);  // create kd-tree
        //urroundingKeyframeSearchRadius是搜索半径，pointSearchInd应该是返回的index，
        // 根据最后一个关键帧的位姿进行最近邻搜索,pointSearchSqDis应该是依次距离中心点的距离
        kdtreeSurroundingKeyPoses->radiusSearch(
                cloudKeyPoses3D->back(), (double) MappingConfig::surroundingKeyframeSearchRadius,
                pointSearchInd, pointSearchSqDis);

        for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];
            //保存附近关键帧,加入相邻关键帧位姿集合中
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }
        //把相邻关键帧位姿集合，进行下采样，滤波后存入surroundingKeyPosesDS
        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        for (auto &pt: surroundingKeyPosesDS->points) {
            // std::cout << "point value: " << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;
            //k近邻搜索,找出最近的k个节点（这里是1）
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd,
                                                      pointSearchSqDis);
            //就是索引，只不过这里借用intensity结构来存放
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one
        // position
        //提取了一些最新的关键帧，以防机器人在一个位置原地旋转
        //上述提取了空间上较近的关键帧,然后再提取一些时间上较近的关键帧,把最近的10秒保存下来
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {
            //把10s内的关键帧也加到surroundingKeyPosesDS中,注意是“也”，原先已经装了下采样的位姿(位置)
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        //对降采样后的点云进行提取出边缘点和平面点对应的localmap
        //从降采样后的相邻关键帧集合的点云中提取出角点和面点对应的localmap;
        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int) cloudToExtract->size(); ++i) {

            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) >
                MappingConfig::surroundingKeyframeSearchRadius)
                continue;
            //取出提出来的关键帧的索引
            int thisKeyInd = (int) cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) !=
                laserCloudMapContainer.end()) {
                //如果这个关键帧对应的点云信息已经存储在一个地图容器里
                //直接从容器中取出来加到局部地图中
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                //这个点云没有实现存储，那就通过该帧对应的位姿，把该帧点云从当前帧的位姿转到世界坐标系下
                pcl::PointCloud <PointType> laserCloudCornerTemp =
                        *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud <PointType> laserCloudSurfTemp =
                        *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);
                //点云转换之后加入到局部地图
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                //转换后的面点和角点存进容器中,方便后续直接加入点云地图，避免点云转换的操作，节约时间
                laserCloudMapContainer[thisKeyInd] =
                        make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000) laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames() {
//        TicToc timer;
        if (cloudKeyPoses3D->points.empty() == true) return;

        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();
        // } else {
        //     extractNearby();
        // }

        extractNearby();
//        double exactsurrounding_key_frames_cost_time = timer.toc();
//        EZLOG(INFO)<<"exactsurrounding_key_frames_cost_time"<<exactsurrounding_key_frames_cost_time<<endl;
    }

    void downsampleCurrentScan() {
//        TicToc timer;
        laserCloudRawDS->clear();
        //对当前帧点云降采样  刚刚完成了周围关键帧的降采样 ,为了使点云稀疏化,加快匹配以及实时性要求
        downSizeFilterRaw.setInputCloud(laserCloudRaw);
        downSizeFilterRaw.filter(*laserCloudRawDS);

        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();

//        double downsample_current_sacn_cost_time = timer.toc();
//        EZLOG(INFO)<<"downsample_current_sacn_cost_time"<<downsample_current_sacn_cost_time<<endl;
    }

    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization() {
        //当前帧的先验位姿（初值估计那来的） 将欧拉角转成eigen的形式
        updatePointAssociateToMap();

        //#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            //将该点从当前帧通过初始的位姿变换到地图坐标系下去
            pointAssociateToMap(&pointOri, &pointSel);
            //在角点地图里面寻找距离当前点比较近的5个点
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                                pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
            //计算找到的点中距离当前点最远的点，如果距离太大那说明这个约束不可信，就跳过
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    //计算5个点的均值
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;
                //计算协方差矩阵
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax =
                            laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay =
                            laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az =
                            laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;
                //特征值分解 为 证明这5个点是一条直线
                cv::eigen(matA1, matD1, matV1);
                //这是线特征 ， 要求最大特征值 大于3倍的次大特征值
                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    //进行直线的构建
                    //通过点的均值往两边拓展,因为最大特征值对应的特征向量对应的就是直线的方向向量,所以用的matV1的第0行
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                    //假设直线上的两点为A和B,O为当前帧角点转换到map坐标系下的点
                    //下式计算为AO向量叉乘BO向量,结果为垂直为平面向上的向量
                    float a012 =
                            sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                                 ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                                 ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                                 ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                                 ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                                 ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));
                    //直线AB的模长
                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                                     (z1 - z2) * (z1 - z2));
                    //下式计算为BA向量叉乘OC向量
                    float la =
                            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                            a012 / l12;

                    float lb =
                            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;

                    float lc =
                            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;
                    //求距离,也就是残差
                    float ld2 = a012 / l12;
                    // 核函数，残差越大 权重越低
                    float s = 1 - 0.9 * fabs(ld2);
                    //残差 x y z 代表方向 intensity 为大小
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;
                    //残差小于10cm,认为是有效的约束
                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization() {
        //将当前帧的先验位姿（初值估计那来的） 将欧拉角转成eigen的形式
        updatePointAssociateToMap();
// long startTime= System.currentTimeMillis();
//#pragma omp parallel for num_threads(numberOfCores)

        for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            //将当前帧通过初始位姿变换到map系
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                              pointSearchSqDis);
            //与角点优化不同的是，不是通过特征值分解来求解特征向量的，而是通过超定方程来求解
            Eigen::Matrix<float, 5, 3> matA0;//5 个点3个未知量，所以是5*3；
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            //平方方程 Ax + By + Cz + 1 = 0
            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                //求解 Ax = B 这个超定方程，对矩阵进行分解
                matX0 = matA0.colPivHouseholderQr().solve(matB0);
                //求出来x的就是这个平面的法向量，假设平面方程为ax+by+ca+d=0,这就是方程的系数abc,d=1;
                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;
                //对法向量单位化ua，求解单位法向量；
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;
                // 检查平面是否合格，如果5个点中有点到平面的距离超过0.2m，那么认为这些点太分散了，不构成平面
                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                             pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }
                //计算当前激光帧到平面的距离
                //点(x0,y0,z0)到了平面Ax+By+Cz+D=0的距离为：d=|Ax0+By0+Cz0+D|/√(A^2+B^2+C^2)
                //
                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) /
                                  sqrt(sqrt(pointOri.x * pointOri.x +
                                            pointOri.y * pointOri.y +
                                            pointOri.z * pointOri.z));
                    //残差到权重的换算，为什么开两次方？？？
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;
                    //如果权重大于阈值，就认为是一个有效的约束
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }
    //提取当前帧中与局部map匹配上了的角点、平面点，加入同一集合
    void combineOptimizationCoeffs() {
        // combine corner coeffs
        // 遍历当前帧角点集合，提取出与局部map匹配上了的角点
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i) {
            if (laserCloudOriSurfFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        //清空标记
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
                  false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(),
                  false);
    }

    bool LMOptimization(int iterCount) {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to
        // cope with coordinate transformation lidar <- camera      ---     camera
        // <- lidar x = z                ---     x = y y = x                --- y =
        // z z = y                ---     z = x roll = yaw           ---     roll =
        // pitch pitch = roll         ---     pitch = yaw yaw = pitch          ---
        // yaw = roll

        // lidar -> camera
        /***
         * camera <- lidar 就是雷达到相机坐标系的变换是这样的：
         * x=y
         * y=z
         * z=x
         * roll=pitch
         * pitch=yaw
         * yaw=roll
         */

        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        //当前帧匹配特征点数应该大于50
        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;
        //遍历匹配特征点，构建Jacobian矩阵
        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            // 求雅克比矩阵中的元素，距离d对roll角度的偏导量即d(d)/d(roll)
            //各种cos sin的是旋转矩阵对roll求导，pointOri.x是点的坐标，coeff.x等是距离到局部点的偏导
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
                         srx * sry * pointOri.z) *
                        coeff.x +
                        (-srx * srz * pointOri.x - crz * srx * pointOri.y -
                         crx * pointOri.z) *
                        coeff.y +
                        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
                         cry * srx * pointOri.z) *
                        coeff.z;
            //对pitch的偏导
            float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
                         (sry * srz + cry * crz * srx) * pointOri.y +
                         crx * cry * pointOri.z) *
                        coeff.x +
                        ((-cry * crz - srx * sry * srz) * pointOri.x +
                         (cry * srz - crz * srx * sry) * pointOri.y -
                         crx * sry * pointOri.z) *
                        coeff.z;
            //对yaw的偏导
            float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                         (-cry * crz - srx * sry * srz) * pointOri.y) *
                        coeff.x +
                        (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                        ((sry * srz + cry * crz * srx) * pointOri.x +
                         (crz * sry - cry * srx * srz) * pointOri.y) *
                        coeff.z;
            // camera -> lidar
            //matA就是误差对旋转和平移变量的雅克比矩阵
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            //对平移求误差就是法向量
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            //残差项







            matB.at<float>(i, 0) = -coeff.intensity;
        }
        // 将矩阵由matA转置生成matAt
        // 先进行计算，以便于后边调用 cv::solve求解
        cv::transpose(matA, matAt);
        //JTJ
        matAtA = matAt * matA;
        //-JTe
        matAtB = matAt * matB;
        // 高斯牛顿法的原型是J^(T)*J * delta(x) = -J*f(x)
        // CV的方法求解JTJX=-JTe
        // 通过QR分解的方式，求解matAtA*matX=matAtB，得到解matX
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        //iterCount==0 说明是第一次迭代，需要初始化
        if (iterCount == 0) {
            //对近似的Hessian矩阵JTJ特征值分解
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            //初次优化时，特征值门限设置为100，小于这个值认为是退化了
            float eignThre[6] = {100, 100, 100, 100, 100, 100};

            //检查是否有退化情况，对JTJ进行特征分解
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }
        //如果发生退化，就对增量进行修改，退化方向不更新
        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        //增量更新,更新当前位姿x=x+deltax
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
        //
        float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));
        // 旋转或者平移量足够小就停止这次迭代过程
        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;  // converged
        }
        return false;  // keep optimizing
    }

    void scan2MapOptimization() {
        TicToc timer;
        if (cloudKeyPoses3D->points.empty()) return;
        //判断当前帧的角点数和面点数是否足够,角点要求10,面点要求100
        if (laserCloudCornerLastDSNum > MappingConfig::edgeFeatureMinValidNum &&
            laserCloudSurfLastDSNum > MappingConfig::surfFeatureMinValidNum) {
            //分别把角点和面点 局部地图构建 kdtree
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            //迭代求解，迭代30次,里面是手写的优化器了用的LM优化方法
            for (int iterCount = 0; iterCount < 30; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true) { break; }
            }
            //把优化后的结果和imu进行一次加权融合；
//            double scantomap_cost_time = timer.toc();
//            EZLOG(INFO)<<"scantomap_cost_time"<<scantomap_cost_time<<endl;
//            transformUpdate();
        } else {
//            ROS_WARN(
//                    "Not enough features! Only %d edge and %d planar features available.",
//                    laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate() {
        //用imu数据对激光里程计位姿进行加权优化
    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool saveFrame() {
        if (cloudKeyPoses3D->points.empty()) return true;

        // if (sensor == SensorType::LIVOX) {
        if (SensorConfig::sensor == LidarType::LIVOX) {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0) return true;
        }
        //取出上一个关键帧的位姿
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        //当前帧位姿
        Eigen::Affine3f transFinal = pcl::getTransformation(
                transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        //位姿变换增量
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        //旋转和平移量都较小，当前帧不设为关键帧
        if (abs(roll) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < MappingConfig::surroundingkeyframeAddingDistThreshold)
            return false;

        // std::cout << "distance gap: " << sqrt(x * x + y * y) << std::endl;
//        keyframeDistances.push_back(sqrt(x * x + y * y));

        return true;
    }

    void addOdomFactor() {
        // EZLOG(I)
        //对于第一帧关键帧，则将置信度设置差一点。尤其是平移和yaw角
        //将Lidar转到世界坐标系

//            float current_T_w[6];
//             lidar2word(current_T_w_l,current_T_w);

        if (cloudKeyPoses3D->points.empty()) {
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                    .finished());  // rad*rad, meter*meter
            //增加先验约束 ， 对第 0 个节点增加约束
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped),
                                              priorNoise));
            //加入节点信息 初始值
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        } else {
            //不是第一帧增加帧间约束，明显观察到约束置信度设置较高；
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            //上一关键帧位姿转成gtsam的 格式
//                float pclpoint_in_world[6];
//                float points_back_world[6];
//                 points_back_world[3] = cloudKeyPoses6D->points.back().x;
//                 points_back_world[4] = cloudKeyPoses6D->points.back().y;
//                 points_back_world[5] = cloudKeyPoses6D->points.back().z;
//                 points_back_world[0] = cloudKeyPoses6D->points.back().roll;
//                 points_back_world[1] = cloudKeyPoses6D->points.back().pitch;
//                 points_back_world[2] = cloudKeyPoses6D->points.back().yaw;

//                lidar2word(points_back_world,pclpoint_in_world);
//                EZLOG(INFO)<<"points_back_world"<<points_back_world[3]<<endl;
//                EZLOG(INFO)<<"points_back_world"<<points_back_world[4]<<endl;
//                EZLOG(INFO)<<"points_back_world"<<points_back_world[5]<<endl;

            gtsam::Pose3 poseFrom =
                    pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            //当前关键帧位姿转成gtsam的 格式
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);

            mtxGraph.lock();
            // 帧间约束 ，分别输入两个节点的id，帧间约束大小以及置信度
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
                    poseFrom.between(poseTo), odometryNoise));
            //加入节点信息 先验位姿
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            mtxGraph.unlock();
        }
    }

    void addGPSFactor() {
        if (deque_gnssins.empty()) return;

        // wait for system initialized and settles down
        //第一个关键帧和最后一个关键帧相差很近，也就算了，要么刚起步，要么会触发回环
        if (cloudKeyPoses3D->points.empty() || cloudKeyPoses3D->points.size() == 1)
            return;
        else {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back())
                < 5.0)
                return;
        }
        //EZLOG(INFO)<<"00000000"<<endl;
        // pose covariance small, no need to correct
        //gtsam 反馈的当前 x、y 的置信度，如果置信度比较高 也不需要 gps来进行 优化
        //3和4应该是x和y（roll，pitch，yaw，x，y，z）
//                    if (poseCovariance(3, 3) <SensorConfig::poseCovThreshold && poseCovariance(4,
//                    4) < poseCovThreshold)
//                        return;

        // last gps position
        static PointType lastGPSPoint;
        //nav_msgs::Odometry thisGPS;

        //  if (syncGPS(gpsQueue, thisGPS, timeLaserInfoCur, 1.0 / SensorConfig::gpsFrequence)) {
        //GPS too noisy, skip

//                float noise_x = thisGPS.pose.covariance[0];
//                float noise_y = thisGPS.pose.covariance[7];
//                float noise_z = thisGPS.pose.covariance[14];
        EZLOG(INFO)<<"22222222"<<endl;
        float noise_x = 0.0001;
        float noise_y = 0.0001;
        float noise_z = 0.0001;

        // make sure the gps data is stable encough
        //如果gps的置信度也不高，也没有必要使用了

        if (abs(noise_x) > SensorConfig::gpsCovThreshold || abs(noise_y) > SensorConfig::gpsCovThreshold)
            return;
//                 double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
//                Eigen::Vector3d LLA(t_w_cur[0],t_w_cur[1], t_w_cur[2]);
//                geo_converter.Forward(LLA[0], LLA[1], LLA[2], gps_x, gps_y, gps_z);
        //取出gps的位置

        double gps_x = deque_gnssins.front().lla[0];
        double gps_y = deque_gnssins.front().lla[1];
        double gps_z = deque_gnssins.front().lla[2];


        //通常gps 的z 没有 x y准，因此这里可以不使用z值 useGpsElevation 默认为0
        //直接拿里程计的z 并 设置 高置信度
        if (!SensorConfig::useGpsElevation) {
            gps_z = transformTobeMapped[5];
            noise_z = 0.01;
        }
        // GPS not properly initialized (0,0,0)
        //如果gps 的 x 或者 y 太小，说明还没有初始化好
        if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) return;

        // Add GPS every a few meters
        //加入gps观测不宜太频繁，相邻不能超过5m
        PointType curGPSPoint;
        curGPSPoint.x = gps_x;
        curGPSPoint.y = gps_y;
        curGPSPoint.z = gps_z;

        EZLOG(INFO)<<"x"<< gps_x<<endl;
        EZLOG(INFO)<<"y"<< gps_y<<endl;
        EZLOG(INFO)<<"z"<< gps_z<<endl;

        if (pointDistance(curGPSPoint, lastGPSPoint) < SensorConfig::gpsDistance)
            return;
        else
            lastGPSPoint = curGPSPoint;

        if (SensorConfig::debugGps) {
//            ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
//                    ROS_INFO("curr gps cov: %f, %f , %f", thisGPS.pose.covariance[0],
//                             thisGPS.pose.covariance[7], thisGPS.pose.covariance[14]);
        }
        //gps 的 置信度，标准差设置成最小1m，也就是不会特别信任gps信息
        gtsam::Vector Vector3(3);
        Vector3 << noise_x, noise_y, noise_z;
        Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
                gtsam::noiseModel::Diagonal::Variances(Vector3);
        //double gps_world[3];
        //Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(t_w_cur);
        // Eigen::Matrix4f map_2_world = SensorConfig::T_L_B.cast<float>();
        //调用gtsam 中集成的gps 约束
        gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(),
                                    gtsam::Point3(gps_x, gps_y, gps_z),
                                    gps_noise);
        keyframeGPSfactor.push_back(gps_factor);
        cloudKeyGPSPoses3D->points.push_back(curGPSPoint);

        // only a trick!
        // we need to accumulate some accurate gps points to initialize the
        // transform between gps coordinate system and LIO coordinate system and
        // then we can add gps points one by one into the pose graph or the whole
        // pose graph will crashed if giving some respectively bad gps points at
        // first.
        if (keyframeGPSfactor.size() < 20) {
            ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
            return;
        }

        if (!gpsTransfromInit) {
            ROS_INFO("Initialize GNSS transform!");
            for (int i = 0; i < keyframeGPSfactor.size(); ++i) {
                gtsam::GPSFactor gpsFactor = keyframeGPSfactor.at(i);
                gtSAMgraph.add(gpsFactor);
                gpsIndexContainer[gpsFactor.key()] = i;
            }
            gpsTransfromInit = true;
        } else {
            // After the coordinate systems are aligned, in theory, the GPS z and
            // the z estimated by the current LIO system should not be too
            // different. Otherwise, there is a problem with the quality of the
            // secondary GPS point.
            //                if (abs(gps_z - cloudKeyPoses3D->back().z) > 10.0) {
            //                    // ROS_WARN("Too large GNSS z noise %f", noise_z);
            //                    gtsam::Vector Vector3(3);
            //                    Vector3 << max(noise_x, 10000.0f), max(noise_y,
            //                    10000.0f), max(noise_z, 100000.0f);
            //                    // gps_noise =
            //                    noiseModel::Diagonal::Variances(Vector3);
            //                    // gps_factor =
            //                    gtsam::GPSFactor(cloudKeyPoses3D->size(),
            //                    gtsam::Point3(gps_x, gps_y, gps_z),
            //                    // gps_noise);
            //                }
            // add loop constriant
            mtxGraph.lock();
            gtSAMgraph.add(gps_factor);
            mtxGraph.unlock();
            gpsIndexContainer[cloudKeyPoses3D->size()] =
                    cloudKeyGPSPoses3D->size() - 1;
        }
        //加入gps 之后 等同于回环，需要触发较多的isam update
        aLoopIsClosed = true;

    }

    void addLoopFactor() {
        if (loopIndexQueue.empty()) return;

        for (int i = 0; i < (int) loopIndexQueue.size(); ++i) {
            int indexFrom = loopIndexQueue[i].first;//当前帧
            int indexTo = loopIndexQueue[i].second;//回环帧
            //帧间约束
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            //回环的置信度就是icp的得分
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            mtxGraph.lock();
            //加入约束
            gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
            mtxGraph.unlock();
        }
        //清空回环相关队列
        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor() {
        TicToc timer;
        //通过旋转和平移增量来判断是否是关键帧，不是则不会添加factor;
        if (saveFrame() == false) return;

        // odom factor
        addOdomFactor();
        // gps factor
        if (SensorConfig::useGPS) { addGPSFactor(); }

        // loop factor
        addLoopFactor();


        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // add raw odom
        //关键帧原始里程计的信息
//        nav_msgs::Odometry laserOdometryROS;
//        transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS,
//                            transformTobeMapped);
//        keyframeRawOdom.push_back(laserOdometryROS);

        // update iSAM
        //调用isam,更新图模型
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        // 如果加入了gps约束或者回环约束，isam需要进行更多次的优化
        if (aLoopIsClosed == true) {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        //约束和节点信息清空
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 latestEstimate;
        //通过接口获得所以变量的状态，获得优化结果
        isamCurrentEstimate = isam->calculateEstimate();
        //取出优化后的最新关键帧位姿
        latestEstimate =
                isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
//        PoseT latest_Estimate_lidar;
//         latest_Estimate_lidar = latestEstimate.matrix();
//        opt_poses.push_back(latest_Estimate_lidar);
//        map_saver.SavePoses(opt_poses);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");
        //平移信息取出来保存进clouKeyPoses 3D这个结构中，其中索引作为 intensity
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();

        thisPose3D.intensity =
                cloudKeyPoses3D->size();  // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        //6D姿态同样保留下来

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time= timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl <<
        // endl;
        //保存当前位姿的置信度 用于是否使用gps的判断
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        //优化后的位姿更新到transformTobeMapped数组中，作为当前最佳估计值
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        //当前帧的点云的角点和面点和原始点云分别拷贝一下
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
        // pcl::copyPointCloud(*laserCloudRawDS, *thislaserCloudRawKeyFrame);
        pcl::copyPointCloud(*laserCloudRaw, *thislaserCloudRawKeyFrame);

        // save key frame cloud
        //保存关键帧的点云
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

//        CloudInfoFt cloud_info;
//        cloud_info.frame_id = ++frame_id;
//        cloud_info.corner_cloud = thisCornerKeyFrame;
//        cloud_info.surf_cloud = thisSurfKeyFrame;
//        map_saver.AddCloudToSave(cloud_info);

//        pcl::PointCloud<PointType>::Ptr globalCornerCloud(
//                new pcl::PointCloud<PointType>());
//        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(
//                new pcl::PointCloud<PointType>());
//        pcl::PointCloud<PointType>::Ptr globalSurfCloud(
//                new pcl::PointCloud<PointType>());
//        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(
//                new pcl::PointCloud<PointType>());
//
//        for (int i = 0; i < (int) cloudKeyPoses3D->size(); i++) {
//            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],
//                                                       &cloudKeyPoses6D->points[i]);
//            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i],
//                                                     &cloudKeyPoses6D->points[i]);
//        }
//
//        downSizeFilterCorner.setInputCloud(globalCornerCloud);
//        downSizeFilterCorner.setLeafSize(MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize);
//        downSizeFilterCorner.filter(*globalCornerCloudDS);
//        // dataSaverPtr->saveCornerClouMap(*globalCornerCloudDS);
//
//        downSizeFilterSurf.setInputCloud(globalSurfCloud);
//        downSizeFilterSurf.setLeafSize(MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize);
//        downSizeFilterSurf.filter(*globalSurfCloudDS);

//        CloudInfoFt cloud_info;
//        cloud_info.frame_id = ++frame_id;
//        cloud_info.corner_cloud = globalCornerCloudDS;
//        cloud_info.surf_cloud = globalSurfCloudDS;
//        map_saver.AddCloudToSave(cloud_info);

//        updatePath(thisPose6D);
    }

    void correctPoses() {
        TicToc timer;
        if (cloudKeyPoses3D->points.empty()) return;
        //只有回环以及gps信息这些会触发全局调整信息才会触发
        if (aLoopIsClosed == true) {
            // clear map cache
            //存放关键帧的位姿和点云的容器清空
            laserCloudMapContainer.clear();
            // clear path
            //清空path
//            globalPath.poses.clear();

            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i) {
                //更新所有关键帧的位姿
                cloudKeyPoses3D->points[i].x =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw =
                        isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();
                //更新path
//                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;

            double correctposes = timer.toc();
            EZLOG(INFO)<<"dorrectposes cost time "<< correctposes<<endl;

        }
    }
    /***
     * 用的ROS,后续需要摘除
     * @param pose_in
     */
    void updatePath(const PointTypePose &pose_in) {
        geometry_msgs::PoseStamped pose_stamped;


        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = SensorConfig::odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;

        tf::Quaternion q =
                tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

//        globalPath.poses.push_back(pose_stamped);
    }

    //发布优化后的里程计
    void publishOdometry() {
        // Publish odometry for ROS (global)
        //发布优化后的里程计
        TicToc timer;
        OdometryType current_lidar_pose_world;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(transformTobeMapped);
        current_lidar_pose_world.frame = "map";
        current_lidar_pose_world.timestamp = timeLaserInfoCur;
        current_lidar_pose_world.pose = PoseT(Lidarodom_2_map.matrix().cast<double>());
        pubsub->PublishOdometry(topic_current_pose, current_lidar_pose_world);
        Function_AddOdometryTypeToIMUPreintegration(current_lidar_pose_world);
        EZLOG(INFO)<<"pub topic_current_pose  "<<std::endl;

//        if (SensorConfig::useGPS) {
//            if (gpsTransfromInit && SensorConfig::updateOrigin) {
//                /** we first update the initial GPS origin points since it may not fix here */
//                Eigen::Vector3d origin_point(cloudKeyPoses6D->at(0).x,
//                                             cloudKeyPoses6D->at(0).y,
//                                             cloudKeyPoses6D->at(0).z);
//                // ENU->LLA
//                Eigen::Vector3d update_origin_lla;
//                geoConverter.Reverse(origin_point[0], origin_point[1], origin_point[2], update_origin_lla[0],
//                                      update_origin_lla[1],update_origin_lla[2]);
//
//                geoConverter.Reset(update_origin_lla[0], update_origin_lla[1], update_origin_lla[2]);
//                std::cout << " origin points: " << originLLA.transpose() << std::endl;
//                std::cout << " update origin points: " << update_origin_lla.transpose() << std::endl;
//                originLLA = update_origin_lla;
//                SensorConfig::updateOrigin = false;
//                ROS_WARN("UPDATE MAP ORIGIN SUCCESS!");
//            }
//
//            /** we transform the optimized ENU point to LLA point for visualization with rviz_satellite*/
//            float current_gnss_point[6];
//                  (
//                    cloudKeyPoses6D->back().x,
//                    cloudKeyPoses6D->back().y,
//                    cloudKeyPoses6D->back().z,
//                    cloudKeyPoses6D->back().roll,
//                    cloudKeyPoses6D->back().pitch,
//                    cloudKeyPoses6D->back().yaw
//                    );
//
//            Eigen::Vector3d current_gnss_lla;
//            // ENU->LLA
//            geoConverter.Reverse(current_gnss_point[0], current_gnss_point[1], current_gnss_point[2], current_gnss_lla[0], current_gnss_lla[1],
//                                  current_gnss_lla[2]);
//
//            Eigen::Affine3f gnss_pose = trans2Affine3f(current_gnss_point);
//
//            OdometryType current_gnss_pose;
//            current_gnss_pose.frame = "map";
//            current_gnss_pose.timestamp = timeLaserInfoCur;
//            current_gnss_pose.pose = PoseT(gnss_pose.matrix().cast<double>());
//            pubsub->PublishOdometry(topic_gnss_pose, current_gnss_pose);
////            Function_AddOdometryTypeToIMUPreintegration(current_gnss_pose);
////            imu_pre_ptr->AddOdomData(current_gnss_pose);//new
////            imu_pre_ptr->AddOdomData(current_gnss_pose);
//
//        }
    }

    void pub_CornerAndSurfGlobalMap()
    {
//        TicToc timer;
////      PriorMap surf
//        CloudTypeXYZI Global_Map_surf_pub;
//        Global_Map_surf_pub.frame = "map";
//        Global_Map_surf_pub.timestamp = timeLaserInfoCur;
//        Global_Map_surf_pub.cloud = *laserCloudCornerLastDS;
//        pubsub->PublishCloud(topic_map_surf, Global_Map_surf_pub);
//        std::cout << "Pub Surf Global Map!"<<std::endl;
//
////      PriorMap Corner
//        CloudTypeXYZI Global_Map_corner_pub;
//        Global_Map_corner_pub.frame = "map";
//        Global_Map_corner_pub.timestamp = timeLaserInfoCur;
//        Global_Map_corner_pub.cloud = *laserCloudSurfLastDS;
//        pubsub->PublishCloud(topic_map_corner, Global_Map_corner_pub);
//        std::cout << "Pub Corner Map!"<<std::endl;
//       double pub_map_cost_time = timer.toc();
//       EZLOG(INFO)<<"pub_map_cost_time"<<pub_map_cost_time<<endl;
    }


    void DoWork(){
           allocateMemory();
        while(1){
            if(deque_cloud.size()!=0){
//                EZLOG(INFO)<<"optmapping_DoWork "<<std::endl;
//                EZLOG(INFO)<<"deque_cloud.size() = "<<deque_cloud.size()<<std::endl;
                CloudFeature cur_ft;
                cloud_mutex.lock();
                cur_ft = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();
                TicToc timer;
                //just do something
//                code
                timeLaserInfoCur = cur_ft.timestamp;
//                auto cur_surf = *cur_ft.surfaceCloud;
//                auto cur_corner = *cur_ft.cornerCloud;
                laserCloudCornerLast = cur_ft.cornerCloud;
                laserCloudSurfLast = cur_ft.surfaceCloud;
                t_w_cur = cur_ft.pose.GetXYZ();
                q_w_cur = cur_ft.pose.GetQ();
                q_w_cur.normalize();
                q_w_cur_matrix = q_w_cur.toRotationMatrix();
                current_frame_id = cur_ft.frame_id;
                // 提取欧拉角（Z-Y-X旋转顺序）q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;
                q_w_pitch = asin(-q_w_cur_matrix(2, 0)); // 计算pitch
                if (cos(q_w_pitch) != 0) {
                    q_w_roll = atan2(q_w_cur_matrix(2, 1), q_w_cur_matrix(2, 2)); // 计算roll
                    q_w_yaw = atan2(q_w_cur_matrix(1, 0), q_w_cur_matrix(0, 0));  // 计算yaw
                } else {
                    q_w_roll = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
                    q_w_yaw = atan2(-q_w_cur_matrix(0, 1), q_w_cur_matrix(1, 1)); // 计算yaw
                }
                double q_to_rpy_cost_time = timer.toc();
//                EZLOG(INFO)<<"get q_to_rpy_cost_time(ms)"<<q_to_rpy_cost_time<<endl;
                std::lock_guard<std::mutex> lock(mtx);
                static double timeLastProcessing = -1;
                if (timeLaserInfoCur - timeLastProcessing >= MappingConfig::mappingProcessInterval) {
                    timeLastProcessing = timeLaserInfoCur;

                    updateInitialGuess();
                    EZLOG(INFO)<<"------------updateInitialGuess finish---------------" <<std::endl;
                    if (systemInitialized) {
                        extractSurroundingKeyFrames();

                        downsampleCurrentScan();

                        scan2MapOptimization();

                        saveKeyFramesAndFactor();

                        correctPoses();

                        publishOdometry();

                        pub_CornerAndSurfGlobalMap();

                    }
                }

            }
        }
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

//        pubsub->addPublisher(topic_map_surf,DataType::LIDAR,10);
//        pubsub->addPublisher(topic_map_corner,DataType::LIDAR,10);
        pubsub->addPublisher(topic_current_pose,DataType::ODOMETRY,10);
//        pubsub->addPublisher(topic_gnss_pose,DataType::ODOMETRY,10);

        do_work_thread = new std::thread(&OPTMapping::DoWork, this);
       // loop_thread =new std::thread(&OPTMapping::loopClosureThread, this);
        //save_Map_thread = new std::thread(&MapSaver::do_work, &(OPTMapping::map_saver));
        //save_path_thread = new std::thread (&OPTMapping::savePathThread, this);

    }
};
#endif //SEU_LIDARLOC_OPT_MAPPING_H
