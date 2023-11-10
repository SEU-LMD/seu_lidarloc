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
#include "imu_wheel_dr.h"
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
   // std::thread* save_path_thread;
    std::mutex cloud_mutex;
    std::mutex gnss_ins_mutex;
    std::mutex work_mutex;
    std::mutex imuodom_mutex;
    std::mutex gnssins_mutex;
    //std::mutex timer_mutex;
    TicToc timer_cloud;
    MapSaver map_saver;

    //    imu_wheel_dr* imu_pre_ptr;
    std::function<void(const OdometryType&)> Function_AddOdometryTypeToIMUPreintegration;


     std::deque<CloudFeature> deque_cloud;
     std::deque<GNSSINSType> deque_gnssins;
    // std::deque<OdometryType> deque_imu;
     std::deque<OdometryType> poseQueue;
     std::deque<GNSSOdometryType> GnssQueue;

//    std::string topic_map_surf = "/map_cloud_surf_global";
//    std::string topic_map_corner = "/map_cloud_corner_global";
      std::string topic_current_pose = "/current_pose";
      std::string topic_corner_cur_world = "/corner_cur_world";
      std::string topic_surf_cur_world = "/surf_cur_world";
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

   // pcl::PointCloud<PointType>::Ptr laserCloudRaw;    // giseop
  //  pcl::PointCloud<PointType>::Ptr laserCloudRawDS;  // giseop
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
    std::vector<double> keyframeDistances;

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
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Vector3d t_gnss_cur;

    vector <pair<int, int>> loopIndexQueue;
    vector <gtsam::Pose3> loopPoseQueue;
    vector <gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
  //  deque <std_msgs::Float64MultiArray> loopInfoVec;
    vector<PoseT> opt_poses;
    map<int, int> loopIndexContainer;  // from new to old
    map<int, int> gpsIndexContainer;

    bool flg_exit = false;
    bool isAddloopFrame = false;
    bool isAddGnssKeyFrame = false;
    bool isAddOdomKeyFrame = false;
    bool timer_start = false;
    bool init = false;

    int lastLoopIndex = -1;
    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;
    int current_frame_id;
    int frame_id = -1;
    int LMsuccess = 0;
    int LMfail = 0;
    int graph_size = 0;

    float current_T_m_l[6]; // roll pitch yaw x y z

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

    std::mutex mtxloop;
   // std::mutex mtxLoopInfo;
   // std::mutex mtxGpsInfo;
    std::mutex mtxGraph;
    //nav_msgs::Path globalPath;
//    nav_msgs::Path globalPath;

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
       // laserCloudRaw.reset(new pcl::PointCloud<PointType>());    // giseop
       // laserCloudRawDS.reset(new pcl::PointCloud<PointType>());  // giseop

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
            current_T_m_l[i] = 0;
        }
       // matP = Eigen::MatrixXf(6,6);
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

    void loopClosureThread() {
        if (MappingConfig::loopClosureEnableFlag == false) return;
        //设置回环的
       // ros::Rate rate(MappingConfig::loopClosureFrequency);
        while (1) {
            //ros::spinOnce();
            TicToc t1;
            performLoopClosure();
          //  EZLOG(INFO)<<"loopClosure"<<t1.toc()<<endl;

            sleep(0.05);
        }
    }

   /**
    * 1.在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    * 2.提取当前关键帧特征点集合，降采样；提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
    * 3.执行scan-to-map优化，调用icp方法，得到优化后位姿，构造闭环因子需要的数据，在因子图优化中一并加入更新位姿
    * note:闭环
    */
    void performLoopClosure() {
    //    EZLOG(INFO)<<"gwi in loop"<<endl;
        if (cloudKeyPoses3D->points.empty() == true) return;

        mtxloop.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses2D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtxloop.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
       // if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
       //在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(
                new pcl::PointCloud<PointType>());
        {
            //提取当前关键帧特征点集合，降采样;
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            //提取闭环匹配关键帧前后25帧的关键帧特征点集合，降采样
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                                  MappingConfig::historyKeyframeSearchNum);
            //如果特征点较少，返回
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
           // if (pubHistoryKeyFrames.getNumSubscribers() != 0)
              //  publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
              //               SensorConfig::odometryFrame);
        }
     ///  EZLOG(INFO)<<"get  in ICP"<<endl;
        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(MappingConfig::historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        //ICP配准，源点云是当前帧，历史关键帧是目标点云；
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(
                new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false ||
            icp.getFitnessScore() > MappingConfig::historyKeyframeFitnessScore)
            return;
         //  EZLOG(INFO)<<"lOOP success!!!"<<endl;
        // publish corrected cloud
//        if (pubIcpKeyFrames.getNumSubscribers() != 0) {
//            pcl::PointCloud<PointType>::Ptr closed_cloud(
//                    new pcl::PointCloud<PointType>());
//            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
//                                     icp.getFinalTransformation());
//            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp,
//                         SensorConfig::odometryFrame);
//        }

        // Get pose transformation
        //闭环优化得到的当前帧与闭环关键帧之间的位姿变换
      // EZLOG(INFO)<<"get  OUT OF ICP"<<endl;
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong
        //闭环优化前当前帧的位姿
        Eigen::Affine3f tWrong =
                pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
       pcl::getTranslationAndEulerAngles(tWrong, x, y, z, roll, pitch, yaw);
     //  EZLOG(INFO)<<"Before loop current pose"<< x <<" "<<y<<" "<<z<<" "<<endl;

       Eigen::Affine3f tPreCur =
               pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyPre]);

       pcl::getTranslationAndEulerAngles(tPreCur, x, y, z, roll, pitch, yaw);
      //  EZLOG(INFO)<<"Loop Candidate pose"<< x <<" "<<y<<" "<<z<<" "<<endl;

        //闭环优化后当前帧的位姿
        Eigen::Affine3f tCorrect =
                correctionLidarFrame *
                tWrong;  // pre-multiplying -> successive rotation about a fixed frame

        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        //当前帧的位姿
      //  EZLOG(INFO)<<"after loop current pose"<< x <<" "<<y<<" "<<z<<" "<<endl;

     //  EZLOG(INFO)<<"get IN gtsam "<<endl;
        gtsam::Pose3 poseFrom =
                gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        //闭环匹配帧的位姿
        gtsam::Pose3 poseTo =
                pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
     //  EZLOG(INFO)<<"get IN gtsam "<<poseTo.translation()<<endl;
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
                noiseScore;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise =
                gtsam::noiseModel::Diagonal::Variances(Vector6);
     //  EZLOG(INFO)<<"get out of gtsam "<<endl;
        // Add pose constraint
        mtxloop.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtxloop.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
        lastLoopIndex = loopKeyCur;
     //  EZLOG(INFO)<<"get out of loop "<<endl;
    }
//    double distance(const pcl::PointCloud<PointType>::Ptr &frame1,
//                    const pcl::PointCloud<PointType>::Ptr &frame2){
//        double dx =
//    }

    bool detectLoopClosureDistance(int *latestID, int *closestID) {
       // EZLOG(INFO)<<"get  in find detectLoopClosure"<<endl;
        //取出最新帧的索引
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        //检查当前帧是否已经添加了回环约束，如果已经添加了回环约束，则将不再继续添加
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        // tricks
        // Two consecutive loop edges represent the closed loop of the same scene.
        // Adding all of them to the pose graph has little meaning and may reduce
        // the accuracy.
        if (abs(lastLoopIndex - loopKeyCur) < 5 && lastLoopIndex != -1)
            return false;

        // tricks
        // sometimes we need to find the corressponding loop pairs
        // but we do not need to care about the z values of these poses.
        // Pls note that this is not work for stair case
        for (int i = 0; i < copy_cloudKeyPoses2D->size(); ++i) {
            copy_cloudKeyPoses2D->at(i).z = 0;
        }

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
        kdtreeHistoryKeyPoses->radiusSearch(
                copy_cloudKeyPoses2D->back(), MappingConfig::historyKeyframeSearchRadius,
                pointSearchIndLoop, pointSearchSqDisLoop, 0);

       //在候选关键帧中，选与当前帧时间相隔60秒的关键帧作为候选帧
//       for (int i = 0;i < (int ) pointSearchIndLoop.size();++i){
//           int id = pointSearchIndLoop[i];
//              double totalDistance += distance(copy_cloudKeyPoses2D->points[id],copy_cloudKeyPoses2D->points[id])
//       }
        for (int i = 0; i < (int) pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) >
                MappingConfig::historyKeyframeSearchTimeDiff) {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;

        // we also need to care about the accumulated distance between keyframe;
        // LOOPs that are too close together have no meaning and may reduce
        // accuracy. For example, the lidar starts to move after being stationary
        // for 30s in a certain place. At this time, the IMU should be trusted more
        // than the lidar.
        if (keyframeDistances.size() >= loopKeyCur) {
            double distance = 0.0;
            for (int j = loopKeyPre; j < loopKeyCur; ++j) {
                distance += keyframeDistances.at(j);
            }
            if (distance < 12) {
                std::cout << "CLOSE FRAME MUST FILTER OUT " << distance << std::endl;
                return false;
            }
        }
       // EZLOG(INFO)<<"get  out of find detectLoopClosure"<<endl;
        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
      //  EZLOG(INFO)<<"get  out of find detectLoopClosure"<<endl;
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;
      //  EZLOG(INFO)<<"get  out of find detectLoopClosure"<<endl;
        return true;
    }


    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                               const int &key, const int &searchNum) {
      //  EZLOG(INFO)<<"get  in find nearkeyframes"<<endl;
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        //提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
        for (int i = -searchNum; i <= searchNum; ++i) {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize) continue;
            *nearKeyframes +=
                    *transformPointCloud(cornerCloudKeyFrames[keyNear],
                                         &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(
                    surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty()) return;

        // downsample near keyframes
        //降采样
        pcl::PointCloud<PointType>::Ptr cloud_temp(
                new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    //    EZLOG(INFO)<<"get  out of find nearkeyframes"<<endl;
    }

    void updateInitialGuess(CloudFeature& cur_ft) {


        Eigen::Vector3d t_lidar_cur;
        Eigen::Quaterniond q_lidar_cur;
        Eigen::Matrix3d q_lidar_cur_matrix;
        float q_lidar_roll, q_lidar_pitch, q_lidar_yaw;
        t_gnss_cur = cur_ft.pose.GetXYZ();
        t_lidar_cur = cur_ft.DRPose.GetXYZ();

        q_lidar_cur = cur_ft.DRPose.GetQ();
        q_lidar_cur.normalize();
        q_lidar_cur_matrix = q_lidar_cur.toRotationMatrix();

        // 提取欧拉角（Z-Y-X旋转顺序）q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;
        q_lidar_pitch = asin(-q_lidar_cur_matrix(2, 0)); // 计算pitch
        if (cos(q_lidar_pitch) != 0) {
            q_lidar_roll = atan2(q_lidar_cur_matrix(2, 1), q_lidar_cur_matrix(2, 2)); // 计算roll
            q_lidar_yaw = atan2(q_lidar_cur_matrix(1, 0), q_lidar_cur_matrix(0, 0));  // 计算yaw
        } else {
            q_lidar_roll = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
            q_lidar_yaw = atan2(-q_lidar_cur_matrix(0, 1), q_lidar_cur_matrix(1, 1)); // 计算yaw
        }

        incrementalOdometryAffineFront = trans2Affine3f(current_T_m_l);//TODO delete!!
        if (cloudKeyPoses3D->points.empty()) {
            systemInitialized = false;

            if (SensorConfig::useGPS) {

                /** add the first factor, we need this origin GPS point for prior map based localization,
                    * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                double t_enu[3];
                 //TODO 1029
//                geoConverter.Reset(deque_gnssins.front().lla[0], deque_gnssins.front().lla[1], deque_gnssins.front().lla[2]);
//                geoConverter.Forward(deque_gnssins.front().lla[0], deque_gnssins.front().lla[1], deque_gnssins.front().lla[2],
//                                     t_enu[0], t_enu[1], t_enu[2]);
                PointType gnssPoint;
                gnssPoint.x = t_gnss_cur[0];
                gnssPoint.y = t_gnss_cur[1];
                gnssPoint.z = t_gnss_cur[2];
//
//                gnssPoint.x = t_enu[0];
//                gnssPoint.y = t_enu[1];
//                gnssPoint.z = t_enu[2];
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
                //mark1 by wxy
                //TODO change name
                current_T_m_l[0] = q_lidar_roll;
                current_T_m_l[1] = q_lidar_pitch;
                current_T_m_l[2] = q_lidar_yaw;
                current_T_m_l[3] = 0;
                current_T_m_l[4] = 0;
                current_T_m_l[5] = 0;

              //  EZLOG(INFO)<<"LIDAR ORIGIN POINT"<<current_T_m_l[3]<<","<<current_T_m_l[4]<<","<<current_T_m_l[5];
                //TODO why not to send value to [3~5]
                systemInitialized = true;

            }
            else{
                //取激光帧信息中的IMU原始数据的RPY进行初始化
                current_T_m_l[3] = 0;
                current_T_m_l[4] = 0;
                current_T_m_l[5] = 0;
                current_T_m_l[0] = q_lidar_roll;
                current_T_m_l[1] = q_lidar_pitch;
                current_T_m_l[2] = q_lidar_yaw;
                ////TODO why not to send value to [0~2]
                systemInitialized = true;
                return;
            }

        }

        if (!systemInitialized) {
           EZLOG(INFO)<<"sysyem need to be initialized"<<endl;
            return;
        }

        static bool lastDRAvailable = false;
        static Eigen::Affine3f lastDRPose;
        Eigen::Affine3f CurrentDRPose = pcl::getTransformation(
                t_lidar_cur[0], t_lidar_cur[1], t_lidar_cur[2], q_lidar_roll, q_lidar_pitch,
                q_lidar_yaw);
        if (lastDRAvailable == false) {
            lastDRPose = CurrentDRPose;
            lastDRAvailable = true;
        } else {
            Eigen::Affine3f transIncre =
                    lastDRPose.inverse() * CurrentDRPose;

            Eigen::Affine3f transTobe = trans2Affine3f(current_T_m_l);
            Eigen::Affine3f transFinal = transTobe * transIncre ;
            pcl::getTranslationAndEulerAngles(
                    transFinal, current_T_m_l[3], current_T_m_l[4],
                    current_T_m_l[5], current_T_m_l[0],
                    current_T_m_l[1], current_T_m_l[2]);

            lastDRPose = CurrentDRPose;

            return;
        }

    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
        // fuse the map
       // EZLOG(INFO)<<"get in extractCloud "<<endl;
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
      //  TicToc t1;
        for (int i = 0; i < (int) cloudToExtract->size(); ++i) {

            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) >
                MappingConfig::surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int) cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) !=
                laserCloudMapContainer.end()) {

                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
              //  TicToc t7;
              //如果这个点云没有实现存储，那就通过该帧对应的位姿，把该帧点云从当前帧的位姿转到世界坐标系下
                pcl::PointCloud <PointType> laserCloudCornerTemp =
                        *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud <PointType> laserCloudSurfTemp =
                        *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);

                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;

              //点云转换之后加到局部地图中
                laserCloudMapContainer[thisKeyInd] =
                        make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }
       // EZLOG(INFO)<<" (int) cloudToExtract->size(): "<<t1.toc();
      //  TicToc t2;
        // Downsample the surrounding corner key frames (or map)
//        downSizeFilterCorner.;
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
      //  EZLOG(INFO)<<"corner map number"<<laserCloudCornerFromMapDSNum<<endl;
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
      //  EZLOG(INFO)<<"surf map number"<<laserCloudSurfFromMapDSNum<<endl;

        if (laserCloudMapContainer.size() > 500) laserCloudMapContainer.clear();

    }
    //TODO
    void extractSurroundingKeyFrames() {

        if (cloudKeyPoses3D->points.empty() == true) { return; }
     //   TicToc t9;
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(
                new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;


        kdtreeSurroundingKeyPoses->setInputCloud(
                cloudKeyPoses3D);  // create kd-tree

        kdtreeSurroundingKeyPoses->radiusSearch(
                cloudKeyPoses3D->back(), (double) MappingConfig::surroundingKeyframeSearchRadius,
                pointSearchInd, pointSearchSqDis);

        for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];

            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        //确认每个下采样后的点的索引
        for (auto &pt: surroundingKeyPosesDS->points) {
            // std::cout << "point value: " << point.x << " " << point.y << " " << point.z << " " << point.intensity << std::endl;

            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd,
                                                      pointSearchSqDis);
            //就是索引，只不过这里借用intensity结构来存放
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        //提取最近10秒的关键帧保存下来
        //TODO change stragegy? Use Key frame?
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i) {

            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time > 10.0) {
                break;
            }
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        }
        //  TicToc t8;
        //根据挑选出的关键帧进行局部地图构建
        extractCloud(surroundingKeyPosesDS);
    }

    void downsampleCurrentScan() {
      //  TicToc timer;
       // laserCloudRawDS->clear();

        // Downsample cloud from current scan
        //EZLOG(INFO)<<"get in downsampleCurrentScan "<<endl;
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
       // EZLOG(INFO)<<"get out downsampleCurrentScan "<<endl;

        //double downsample_current_sacn_cost_time = timer.toc();
       // EZLOG(INFO)<<"downsample_current_sacn_cost_time"<<downsample_current_sacn_cost_time<<endl;
    }

    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(current_T_m_l);
    }

    void cornerOptimization() {
        EZLOG(INFO)<<"get in cornerOptimization "<<endl;

        updatePointAssociateToMap();

        //#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            EZLOG(INFO)<<pointOri.x<<","<<pointOri.y<<","<<pointOri.z<<endl;

            pointAssociateToMap(&pointOri, &pointSel);
            EZLOG(INFO)<<"get in kdtreeCornerFromMap "<<endl;
          //  EZLOG(INFO)<<"POINTSel"<<pointOri.x<<" "<<pointOri.y<<" "<<pointOri.z;
            EZLOG(INFO)<<pointSel.x<<","<<pointSel.y<<","<<pointSel.z<<endl;
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                                pointSearchSqDis);

           // EZLOG(INFO)<<pointSel.x<<","<<pointSel.y<<","<<pointSel.z<<endl;
            EZLOG(INFO)<<"get out of corner opt kdtreeCornerFromMap  "<<endl;
            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
            //计算找到的点中距离当前点最远的点，如果距离太大那说明这个约束不可信，就跳过
          //  EZLOG(INFO)<<"get out pointSearchSqDis "<<endl;
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {

                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx *= 0.2;
                cy *= 0.2;
                cz *= 0.2;

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
                a11 *= 0.2;
                a12 *= 0.2;
                a13 *= 0.2;
                a22 *= 0.2;
                a23 *= 0.2;
                a33 *= 0.2;

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

//                matV1_eigen()
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
                    float a012_x,a012_y,a012_z;
                    a012_x = (x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1);
                    a012_y = (x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1);
                    a012_z = (y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1);
                    float a012 =
                            sqrt(a012_x * a012_x +
                                 a012_y * a012_y +
                                 a012_z * a012_z);

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                                     (z1 - z2) * (z1 - z2));
//
                    float la =
                            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
                            a012 / l12;
                    // float la = matv1_f[0];
                     EZLOG(INFO)<<"la"<<la<<endl;
                    float lb =
                            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;
                   // float lb = matv1_f[1];
                    EZLOG(INFO)<<"lb"<<lb<<endl;

                    float lc =
                            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;

                  //  float lc = matv1_f[2];
                    EZLOG(INFO)<<"lc"<<lc<<endl;
                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }

            }
        }
        EZLOG(INFO)<<"get out cornerOptimization "<<endl;
    }

    void surfOptimization() {
      //  EZLOG(INFO)<<"get in surfOptimization "<<endl;
        updatePointAssociateToMap();

        for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];

            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                              pointSearchSqDis);

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

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

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

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) /
                                  sqrt(sqrt(pointOri.x * pointOri.x +
                                            pointOri.y * pointOri.y +
                                            pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
       // EZLOG(INFO)<<"get out surfOptimization "<<endl;
    }

    void combineOptimizationCoeffs() {
        //EZLOG(INFO)<<"get in combineOptimizationCoeffs "<<endl;

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

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
                  false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(),
                  false);
    }

    bool LMOptimization(int iterCount) {
        TicToc timer;

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

        float srx = sin(current_T_m_l[1]);
        float crx = cos(current_T_m_l[1]);
        float sry = sin(current_T_m_l[2]);
        float cry = cos(current_T_m_l[2]);
        float srz = sin(current_T_m_l[0]);
        float crz = cos(current_T_m_l[0]);

        float a11 = crx * sry * srz;
        float a12 = crx * crz * sry;
        float a13 = srx * sry;
        float a14 = srx * srz ;
        float a15 = crz * srx ;
        float a16 = crx * cry * srz;
        float a17 = crx * cry * crz;
        float a18 = cry * srx;

        float a21 =  cry * srx * srz - crz * sry;
        float a22 =  sry * srz + cry * crz * srx;
        float a23 =  crx * cry ;
        float a24 =  -cry * crz - srx * sry * srz;
        float a25 =  cry * srz - crz * srx * sry;
        float a26 =   crx * sry;

        float a31 = crz * srx * sry - cry * srz;
        float a32 =  -cry * crz - srx * sry * srz;
        float a33 =  crx * crz;
        float a34 =  crx * srz;
        float a35 =  sry * srz + cry * crz * srx;
        float a36 =  crz * sry - cry * srx * srz;


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

//            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
//                         srx
//                         * sry * pointOri.z) *
//                        coeff.x +
//                        (-srx * srz * pointOri.x - crz * srx * pointOri.y -
//                         crx * pointOri.z) *
//                        coeff.y +
//                        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
//                         cry * srx * pointOri.z) *
//                        coeff.z;

            float arx = (a11 * pointOri.x +a12 * pointOri.y - a13 * pointOri.z) * coeff.x+
                        (-a14 * pointOri.x - a15 * pointOri.y - crx * pointOri.z) * coeff.y +
                        (a16 * pointOri.x + a17 * pointOri.y - a18 * pointOri.z) * coeff.z;

//            float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
//                         (sry * srz + cry * crz * srx) * pointOri.y +
//                         crx * cry * pointOri.z) *
//                        coeff.x +
//                        ((-cry * crz - srx * sry * srz) * pointOri.x +
//                         (cry * srz - crz * srx * sry) * pointOri.y -
//                         crx * sry * pointOri.z) *
//                        coeff.z;
           float ary   =  (a21 * pointOri.x +a22 * pointOri.y + a23 * pointOri.z) * coeff.x +
                          (a24 *pointOri.x + a25 * pointOri.y - a26 * pointOri.z) * coeff.z;

//            float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
//                         (-cry * crz - srx * sry * srz) * pointOri.y) *
//                        coeff.x +
//                        (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
//                        ((sry * srz + cry * crz * srx) * pointOri.x +
//                         (crz * sry - cry * srx * srz) * pointOri.y) *
//                        coeff.z;
           float arz = (a31 * pointOri.x + a32 * pointOri.y ) * coeff.x +
                       (a33 * pointOri.x - a34 * pointOri.y) * coeff.y +
                       (a35 * pointOri.x + a36 * pointOri.y) * coeff.z;

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

            float eignThre[6] = {100, 100, 100, 100, 100, 100};

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

        if (isDegenerate) {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        //增量更新,更新当前位姿x=x+deltax
        current_T_m_l[0] += matX.at<float>(0, 0);
        current_T_m_l[1] += matX.at<float>(1, 0);
        current_T_m_l[2] += matX.at<float>(2, 0);
        current_T_m_l[3] += matX.at<float>(3, 0);
        current_T_m_l[4] += matX.at<float>(4, 0);
        current_T_m_l[5] += matX.at<float>(5, 0);
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
       // EZLOG(INFO)<<"get out LMOptimization "<<endl;
      //  EZLOG(INFO)<<"LM cost "<< timer.toc()<<endl;
        return false;  // keep optimizing
    }

    void scan2MapOptimization() {
       // TicToc timer;
       // EZLOG(INFO)<<"get in scan2MapOptimization "<<endl;
        if (cloudKeyPoses3D->points.empty()) return;
           EZLOG(INFO)<<"laserCloudCornerLastDSNum number"<<laserCloudCornerLastDSNum<<endl;
           EZLOG(INFO)<<"laserCloudSurfLastDSNum number"<<laserCloudCornerLastDSNum<<endl;
        if (laserCloudCornerLastDSNum > MappingConfig::edgeFeatureMinValidNum &&
            laserCloudSurfLastDSNum > MappingConfig::surfFeatureMinValidNum) {

            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            const int maxIters = 20;
            for (int iterCount = 0; iterCount < maxIters; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();
                 TicToc t1;
                cornerOptimization();
               // EZLOG(INFO)<<"cornerOptimization cost time" << t1.toc()<<endl;
                 TicToc t2;
                surfOptimization();
              //  EZLOG(INFO)<<"surfOptimization cost time " << t2.toc()<<endl;

                combineOptimizationCoeffs();

               // TicToc timer;
                if (LMOptimization(iterCount) == true)
                {
                    ++LMsuccess;
              //  EZLOG(INFO)<<"LMOptimization success and inter count is"<<iterCount<<endl;
             //   EZLOG(INFO)<<"LMsuccess count is"<<LMsuccess<<endl;
                    break;
                }

                if (iterCount == maxIters - 1){
                    ++LMfail;
              //      EZLOG(INFO)<<"LMfail count is"<<LMfail<<endl;
                }
                //把转换后加入到container
            }
            //把优化后的结果和imu进行一次加权融合；
        }

    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool SaveLidarKeyFrame() {
       // EZLOG(INFO)<<"get in saveFrame "<<endl;
        if (cloudKeyPoses3D->points.empty()) return true;

        // if (sensor == SensorType::LIVOX) {
        if (SensorConfig::sensor == LidarType::LIVOX) {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0) return true;
        }
        //取出上一个关键帧的位姿
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        //当前帧位姿
        Eigen::Affine3f transFinal = pcl::getTransformation(
                current_T_m_l[3], current_T_m_l[4], current_T_m_l[5],
                current_T_m_l[0], current_T_m_l[1], current_T_m_l[2]);
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
        keyframeDistances.push_back(sqrt(x * x + y * y));
        //EZLOG(INFO)<<"get out saveFrame "<<endl;
        return true;
    }

    void addOdomFactor() {
        // EZLOG(I)
        //对于第一帧关键帧，则将置信度设置差一点。尤其是平移和yaw角
        //将Lidar转到世界坐标系

        //EZLOG(INFO)<<"get in addOdomFactor "<<endl;
        if (cloudKeyPoses3D->points.empty()) {
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                    .finished());  // rad*rad, meter*meter
            //增加先验约束 ， 对第 0 个节点增加约束
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(current_T_m_l),
                                              priorNoise));
            //加入节点信息 初始值
            initialEstimate.insert(0, trans2gtsamPose(current_T_m_l));
        } else {
            //不是第一帧增加帧间约束，明显观察到约束置信度设置较高；
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());


            gtsam::Pose3 poseFrom =
                    pclPointTogtsamPose3(cloudKeyPoses6D->points.back());

            gtsam::Pose3 poseTo = trans2gtsamPose(current_T_m_l);

           // {
                //std::lock_guard<std::mutex> lock(mtxGraph);//TODO delete?
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                        cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
                        poseFrom.between(poseTo), odometryNoise));

                initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            isAddOdomKeyFrame = true;
          //  }
           // EZLOG(INFO)<<"get in addOdomFactor "<<endl;
        }

    }

    void addGPSFactor() {
        /***
         * 接入GPS status
         */
        //cloud_mutex.lock();
        if (deque_cloud.empty()) return;
        //cloud_mutex.unlock();

        // wait for system initialized and settles down
        //第一个关键帧和最后一个关键帧相差很近，也就算了，要么刚起步，要么会触发回环
        if (cloudKeyPoses3D->points.empty() || cloudKeyPoses3D->points.size() == 1)
            return;

        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0){
                return;
        }

        static PointType lastGPSPoint;
        while(1) {

            cloud_mutex.lock();
            if(deque_cloud.empty()){
                cloud_mutex.unlock();
                break;
            }
            cloud_mutex.unlock();
            double noise_x = 1;
            double noise_y = 1;
            double noise_z = 1;

            if (abs(noise_x) > SensorConfig::gpsCovThreshold || abs(noise_y) > SensorConfig::gpsCovThreshold)
                continue;

            double gps_x = t_gnss_cur[0];
            double gps_y = t_gnss_cur[1];
            double gps_z = t_gnss_cur[2];

          //  EZLOG(INFO) << "GPS" << gps_x << " " << gps_y << " " << gps_z;

            if (!SensorConfig::useGpsElevation) {
                gps_z = current_T_m_l[5];
                noise_z = 0.001;
            }

            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;
            //  EZLOG(INFO)<<"get out addGps factor"<<endl;
            // Add GPS every a few meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;

            if (pointDistance(curGPSPoint, lastGPSPoint) < 4.0)
                break;
            lastGPSPoint = curGPSPoint;

            gtsam::Vector Vector3(3);
            Vector3 << noise_x, noise_y, noise_z;
            //Vector3 << max(noise_x, 1.0), max(noise_y, 1.0), max(noise_z, 1.0);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
            gtSAMgraph.add(gps_factor);
           // EZLOG(INFO) << "ADD GPS successful and gps factor is" << curGPSPoint.x << curGPSPoint.y << curGPSPoint.z
          //              << endl;
            //cloud_mutex.unlock();
            isAddGnssKeyFrame == true;
            break;
        }
    }
    void addLoopFactor() {
        if (loopIndexQueue.empty()) return;

        for (int i = 0; i < (int) loopIndexQueue.size(); ++i) {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
           // mtxGraph.lock();
            gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
           // mtxGraph.unlock();
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        isAddloopFrame = true;
       // EZLOG(INFO)<<"get out of addLoop factor"<<endl;
    }
    //TODO: FactorOpt and Save cloud
    void FactorOptandSavecloud() {
        gtsam::ISAM2Params parameters = gtsam::ISAM2Params();
        parameters.factorization = gtsam::ISAM2Params::QR;
       // parameters.factorization = ISAM2Params::QR;
     //   TicToc timer;
       // EZLOG(INFO)<<"get in saveKeyFramesAndFactor "<<endl;
        //通过旋转和平移增量来判断是否是关键帧，不是则不会添加factor;
        //TODO change function name!!!
        if (SaveLidarKeyFrame() == false) return;
        // odom factor
        addOdomFactor();

        if (SensorConfig::useGPS) {
            addGPSFactor();
        }
        // gps factor
        addLoopFactor();

        gtSAMgraph.print("GTSAM Graph:\n");// update iSAM
        //调用isam,更新图模型
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        // 如果加入了gps约束或者回环约束，isam需要进行更多次的优化
        isam->update();
        //TODO???
//
//        ++graph_size;
//        EZLOG(INFO)<<"GTSAM graph size is"<<graph_size <<endl;
//        if (graph_size>40){
//            gtSAMgraph.resize(0);
//            graph_size =0;
//        }
        //约束和节点信息清空
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 latestEstimate;
        //通过接口获得所以变量的状态，获得优化结果
        isamCurrentEstimate = isam->calculateEstimate();
        //double isam = timer.toc();
       // EZLOG(INFO)<<"ISAM"<<isam<<endl;
        //取出优化后的最新关键帧位姿
        latestEstimate =
                isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

//        PoseT latest_Estimate_lidar;//TODO!!!! should save all poses after opt not current pose
//            latest_Estimate_lidar = latestEstimate.matrix();
//            opt_poses.push_back(latest_Estimate_lidar);
//           map_saver.SavePoses(opt_poses);

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

       // poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        //优化后的位姿更新到transformTobeMapped数组中，作为当前最佳估计值
        current_T_m_l[0] = latestEstimate.rotation().roll();
        current_T_m_l[1] = latestEstimate.rotation().pitch();
        current_T_m_l[2] = latestEstimate.rotation().yaw();
        current_T_m_l[3] = latestEstimate.translation().x();
        current_T_m_l[4] = latestEstimate.translation().y();
        current_T_m_l[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        //当前帧的点云的角点和面点和原始点云分别拷贝一下
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        //pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        //保存关键帧的点云
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        CloudTypeXYZI corner_pub,surf_pub;
        corner_pub.frame = "map";
        corner_pub.timestamp = timeLaserInfoCur;
        corner_pub.cloud = *thisCornerKeyFrame;
        surf_pub.frame = "map";
        surf_pub.timestamp = timeLaserInfoCur;
        surf_pub.cloud = *thisSurfKeyFrame;
        pubsub->PublishCloud(topic_corner_cur_world, corner_pub);
        pubsub->PublishCloud(topic_surf_cur_world,surf_pub);

        CloudInfoFt cloud_info;
        cloud_info.frame_id = ++frame_id;
        cloud_info.corner_cloud = thisCornerKeyFrame;
        cloud_info.surf_cloud = thisSurfKeyFrame;
        map_saver.AddCloudToSave(cloud_info);

    }

    void correctPoses() {
       // TicToc timer;
        if (cloudKeyPoses3D->points.empty()) return;
        //只有回环以及gps信息这些会触发全局调整信息才会触发
        ////TODO
        if (isAddloopFrame == true || isAddGnssKeyFrame == true ) {
            // clear map cache
            //存放关键帧的位姿和点云的容器清空
            laserCloudMapContainer.clear();
            int numPoses = isamCurrentEstimate.size();
            int lastGnsskeyFrame = 0;
            int start_idx = 0;
           // if(isAddloopFrame ==true|| isAddGnssKeyFrame == true ){
                //idea1
              //  start_idx = min(0,numPoses - 30);
                //idea2
               // lastGnsskeyFrame = numPoses;
          //  }

            // update key poses
            for (int i = start_idx; i < numPoses; ++i)  {
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

//
                //lastGnsskeyFrame = numPoses;
                //更新path
            }

//            for(int i = start_idx; i < numPoses; ++i){
//                PoseT gloabal_corrected_pose;
//                gloabal_corrected_pose = isamCurrentEstimate.at<gtsam::Pose3>(i).matrix();
//                opt_poses.push_back(gloabal_corrected_pose);
//                map_saver.SavePoses(opt_poses);
//            }

            isAddloopFrame = false;
            isAddGnssKeyFrame == false;
            isAddOdomKeyFrame == false;

        }
//                PoseT gloabal_corrected_pose;
//                gloabal_corrected_pose = isamCurrentEstimate.at<gtsam::Pose3>(i).matrix();
//                opt_poses.push_back(gloabal_corrected_pose);
//                map_saver.SavePoses(opt_poses);

    }

    //发布优化后的里程计
    void publishOdometry() {
        // Publish odometry for ROS (global)
        //发布优化后的里程计
       // EZLOG(INFO)<<"get in publishOdometry "<<endl;

        OdometryType current_lidar_pose_world;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(current_T_m_l);

        current_lidar_pose_world.frame = "map";
        current_lidar_pose_world.timestamp = timeLaserInfoCur;
        current_lidar_pose_world.pose.pose = Lidarodom_2_map.matrix().cast<double>();
       // current_lidar_pose_world.pose = PoseT(Lidarodom_2_map.matrix().cast<double>());
        pubsub->PublishOdometry(topic_current_pose, current_lidar_pose_world);
       // Function_AddOdometryTypeToIMUPreintegration(current_lidar_pose_world);

    }

    void DoWork(){

        while(1){

            bool isEmpty = false;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                isEmpty = deque_cloud.empty();
            }

            if(!isEmpty){

                EZLOG(INFO)<<deque_cloud.size()<<endl;
                CloudFeature cur_ft;
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex);
                    cur_ft = deque_cloud.front();
                    deque_cloud.pop_front();
                }
                //TODO !!!!!!!!
                current_frame_id = cur_ft.frame_id;
                timeLaserInfoCur = cur_ft.timestamp;
               // auto cur_surf = *cur_ft.cornerCloud;
                laserCloudCornerLast = cur_ft.cornerCloud;
                //auto cur_corner = *cur_ft.cornerCloud;
                laserCloudSurfLast = cur_ft.surfaceCloud;

                static double timeLastProcessing = -1;
                if (timeLaserInfoCur - timeLastProcessing >= MappingConfig::mappingProcessInterval) {
                    //std::lock_guard<std::mutex> lock(cloud_mutex);
                    timeLastProcessing = timeLaserInfoCur;
                    TicToc t0;
                    updateInitialGuess(cur_ft);//TODO
                   // EZLOG(INFO)<<" updateInitialGuess COST TIME"<<t0.toc()<<endl;
                    if (systemInitialized) {
                        TicToc t1;
                        extractSurroundingKeyFrames();
                       // EZLOG(INFO)<<" extractSurroundingKeyFrames COST TIME"<<t1.toc()<<endl;
                       // TicToc t2;
                        downsampleCurrentScan();
                       // EZLOG(INFO)<<" downsampleCurrentScan COST TIME"<<t2.toc()<<endl;
                        TicToc t3;
                        scan2MapOptimization();
                        //EZLOG(INFO)<<" scan2MapOptimization COST TIME"<<t3.toc()<<endl;
                        TicToc t4;
                        FactorOptandSavecloud();
                       // EZLOG(INFO)<<" saveKeyFramesAndFactor COST TIME"<<t4.toc()<<endl;
                        TicToc t5;
                        correctPoses();
                       // EZLOG(INFO)<<" correctPoses COST TIME"<<t5.toc()<<endl;
                        publishOdometry();

                    }

                }

            }
            else{
                sleep(0.01);
            }
        }
    }

    void AddCloudData(const CloudFeature& cloud_ft){
        std::lock_guard<std::mutex> lock(cloud_mutex);
        deque_cloud.push_back(cloud_ft);
       // EZLOG(INFO)<<deque_cloud.size()<<endl;
    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){
        std::lock_guard<std::mutex> lock(gnss_ins_mutex);
        deque_gnssins.push_back(gnss_ins_data);
    }

//    void AddGNSSToOpt(const GNSSOdometryType &gnss_odom){
//        gnssins_mutex.lock();
//        GnssQueue.push_back(gnss_odom);
//        gnssins_mutex.unlock();
//
//    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        allocateMemory();
        pubsub->addPublisher(topic_current_pose,DataType::ODOMETRY,10);
        pubsub->addPublisher(topic_corner_cur_world, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_surf_cur_world, DataType::LIDAR, 10);

        do_work_thread = new std::thread(&OPTMapping::DoWork, this);
        loop_thread =new std::thread(&OPTMapping::loopClosureThread, this);
        save_Map_thread = new std::thread(&MapSaver::do_work, &(OPTMapping::map_saver));

    }
};
#endif //SEU_LIDARLOC_OPT_MAPPING_H
