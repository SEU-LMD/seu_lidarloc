//
// Created by fyy on 23-9-10.
//

#ifndef SEU_LIDARLOC_OPT_MAPPING_H
#define SEU_LIDARLOC_OPT_MAPPING_H
//3rdParty
#include <assert.h>
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
#include "easylogging++.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
//homeMade
#include "imu_wheel_dr.h"
#include "utils/utm/utm_convert.h"
#include "utils/timer.h"
#include "utils/MapSaver.h"
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"

class OPTMapping{
public:
    PubSubInterface* pubsub;
    std::thread* do_work_thread;
    std::thread* loop_thread;
    std::thread* save_Map_thread;
    std::thread* savepathThread;
    std::mutex cloud_mutex;
    std::mutex mtx;
    std::mutex mtxGraph;
    std::mutex timer_mutex;

    MapSaver map_saver;

    std::function<void(const OdometryType&)> Function_AddOdometryTypeToIMUPreintegration;
    std::deque<CloudFeature> deque_cloud;
    std::string topic_current_pose = "/current_pose";
    std::string topic_this_Corner_Keyframe= "/this_Corner_Keyframe";
    std::string topic_this_Surf_Keyframe = "/this_Surf_Keyframe";
    std::string topic_cloudOut_Keyframe = "/cloudOut_Keyframe";
    std::string topic_localmapOut_Keyframe = "/localmapOut_Keyframe";

    vector <pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector <pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;// history keyframe
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses2D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // downsampled corner feature set from
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;  // downsampled surf feature set from

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

    std::vector<double> keyframeDistances;
    std::vector<PointType>laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType>laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;
    //Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Vector3d q_gnss_rpy;
    Eigen::Vector3d q_lidar_rpy;
    Eigen::Vector3d t_gnss_cur;
    Eigen::Vector3d t_lidar_cur;
    Eigen::Matrix3d R_gnss_cur;
    Eigen::Matrix3d R_lidar_cur;
    Eigen::Affine3f lastDRPose;
    Eigen::Affine3f currentDRPose;
    PoseT last_DRPose;
    PoseT current_DRPose;
    PoseT current_GNSSPose;
    PoseT last_GNSSPose;
   // float q_gnss_roll, q_gnss_pitch, q_gnss_yaw;
   // float q_lidar_roll, q_lidar_pitch, q_lidar_yaw;

    double noise_x,noise_y,noise_z,noise_roll,noise_pitch,noise_yaw;

    vector <pair<int, int>> loopIndexQueue;
    vector <gtsam::Pose3> loopPoseQueue;
    vector <gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    vector<PoseT> opt_poses;
    vector<PoseT> opt_correct_poses;
    map<int, int> loopIndexContainer;  // from new to old

    bool isAddloopFrame = false;
    bool isAddGnssKeyFrame = false;
    bool isAddOdomKeyFrame = false;
    bool init = false;
    bool  IsRealiableGnss =false;
    bool timer_start = false;

    int lastLoopIndex = -1;
    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;
    int current_frame_id;
    int frame_id = -1;
    int LMsuccess = 0;
    int LMfail = 0;
    int OptCornerNum;
    int OptSurfNum;
    int loopKeyCur;
    int loopKeyPre;

    float current_T_m_l[6]; // roll pitch yaw x y z
    float current_gnss_pose[6];
    gtsam::Pose3 last_gnss_poses;
    gtsam::ISAM2 *isam;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::NonlinearFactorGraph gtSAMgraphloop;

    gtsam::Values initialEstimate;
    gtsam::Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    double timeLaserInfoCur;
    bool isDegenerate = false;
    cv::Mat matP;
    Eigen::Vector3d originLLA;
    bool systemInitialized = false;
    bool DRInitialized;
    GeographicLib::LocalCartesian geoConverter;
    TicToc timer_cloud;

    OPTMapping() : DRInitialized(false) , OptCornerNum(0), OptSurfNum(0) {}

    void allocateMemory() {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());  // corner feature set from
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());  // surf feature set from
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled corner featuer set
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());  // downsampled surf featuer set from

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

        for (int i = 0; i < 6; ++i) {
            current_T_m_l[i] = 0;
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

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(
                transformIn->x, transformIn->y, transformIn->z, transformIn->roll,
                transformIn->pitch, transformIn->yaw);

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

//    void savePathThread() {
//        while (1) {
//
//            //保证有了位姿数据
//            if (timer_start == true) {
//                timer_mutex.lock();
//                double delta_time = timer_cloud.toc();
//                timer_mutex.unlock();
//                EZLOG(INFO)<<"delta_time = "<<delta_time<<std::endl;
//                if(delta_time>5000){
//                   // EZLOG(INFO)<<"1"<<endl;
//                   // int numPoses = opt_poses.size();
//                   //1.构建因子图gtSAMgraphloop；
//                    //addLoopOdomFactor();
//                    //addLoopFactor();
//                    gtsam::Vector Vector6(6);
//                    Vector6 << 1e-2,1e-2,1e-2,1e-2,1e-2,1e-2;
//                    gtsam::noiseModel::Diagonal::shared_ptr unaryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
//
//                    //3.给定初值
//                    gtsam::Values initialLoop;
//                    int numPoses = isamCurrentEstimate.size();
//                    EZLOG(INFO)<<"numpose size"<<numPoses<<endl;
//
//                    for (int i = 0; i <numPoses; ++i){
//                        gtsam::Pose3 initialloopEstimate = isamCurrentEstimate.at<gtsam::Pose3>(i);
//                        initialLoop.insert(numPoses,initialloopEstimate);
//                    }
//                    EZLOG(INFO)<<"3"<<endl;
//                   //3.优化
//                    gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraphloop, initialLoop);
//                    gtsam::Values  resultloop =optimizer.optimize();
//                    EZLOG(INFO)<<"5"<<endl;
//
//                    int num = resultloop.size();
//                    PoseT gloabal_corrected_pose;
//                    gloabal_corrected_pose = isamCurrentEstimate.at<gtsam::Pose3>(num - 1).matrix();
//                    opt_poses.push_back(gloabal_corrected_pose);
//                    map_saver.SavePoses(opt_poses);
////                    for (int i = 0; i<num;++i){
////                        PoseT gloabal_corrected_pose;
////                        gloabal_corrected_pose = resultloop.at<gtsam::Pose3>(i).matrix();;
////                        opt_correct_poses[i] = gloabal_corrected_pose;
////                        map_saver.SaveCorrectPoses(opt_correct_poses);
////                    }
//
//                }
//            }//end if(delta_time>5000)
//
//            sleep(0.1);
//        }
//    }

    void loopClosureThread() {
        if (MappingConfig::loopClosureEnableFlag == false) return;

        while (1) {

            performLoopClosure();
            sleep(1);
        }
    }

    void performLoopClosure() {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();//
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses2D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;// with quaternioan
        mtx.unlock();

//        int loopKeyCur;
//        int loopKeyPre;

        if (detectLoopClosureDistance(loopKeyCur, loopKeyPre) == false) return;
        EZLOG(INFO)<< "Start loop opt" << std::endl;

        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(
                new pcl::PointCloud<PointType>());
        {

            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                                  MappingConfig::historyKeyframeSearchNum);

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;

        }

        pcl::IterativeClosestPoint<PointType, PointType> icp;//TODO 1118 remove static
        icp.setMaxCorrespondenceDistance(MappingConfig::historyKeyframeSearchRadius * 2);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(
                new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false ||
            icp.getFitnessScore() > MappingConfig::historyKeyframeFitnessScore)
            return;

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f T_wt_w;
        T_wt_w = icp.getFinalTransformation();//it is the transformation from source cloud to target cloud;

        Eigen::Affine3f T_w_cur =
                pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
       pcl::getTranslationAndEulerAngles(T_w_cur, x, y, z, roll, pitch, yaw);
       EZLOG(INFO)<<"Current pose before loop Opt: "<< x <<" "<<y<<" "<<z<<" "<<endl;

       Eigen::Affine3f T_wt_pre =
               pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyPre]);

       pcl::getTranslationAndEulerAngles(T_wt_pre, x, y, z, roll, pitch, yaw);
        EZLOG(INFO)<<"Loop Candidate pose:"<< x <<" "<<y<<" "<<z<<" "<<endl;

        //Eigen::Affine3f loopOptCurPose = T_wt_pre * T_wt_w;  // pre-multiplying -> successive rotation about a fixed frame
        Eigen::Affine3f  loopOptCurPose = T_wt_w * T_w_cur;

        pcl::getTranslationAndEulerAngles(loopOptCurPose, x, y, z, roll, pitch, yaw);
        EZLOG(INFO)<<"after loop current pose"<< x <<" "<<y<<" "<<z<<" "<<endl;
        //loopCurKey
        gtsam::Pose3 poseFrom =
                gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
        //loopPreKey
        gtsam::Pose3 poseTo =
                pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore() ;
        EZLOG(INFO)<<"LOOP noise is "<<noiseScore<<endl;
      //  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
       //         noiseScore;
        Vector6<<0.001,0.001,0.001,0.001,0.001,0.001;
        gtsam::noiseModel::Diagonal::shared_ptr constraintNoise =
                gtsam::noiseModel::Diagonal::Variances(Vector6);
        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));//T_cur_pre
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
        lastLoopIndex = loopKeyCur;

    }

    //int& lastID TODO 1118
    bool detectLoopClosureDistance(int& latestID, int& closestID) {

        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        //TODO 1118 hack logic remove or change strategy!unnecessary!
//        if (abs(lastLoopIndex - loopKeyCur) < 5 && lastLoopIndex != -1)
//            return false;

        for (int i = 0; i < copy_cloudKeyPoses2D->size(); ++i) {
            copy_cloudKeyPoses2D->at(i).z = 0;
        }

        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
        kdtreeHistoryKeyPoses->radiusSearch(
                copy_cloudKeyPoses2D->back(), MappingConfig::historyKeyframeSearchRadius,
                pointSearchIndLoop, pointSearchSqDisLoop, 0);

        //TODO 1118!
        //just find a point histort,now need to find many points
        for (int i = 0; i < (int) pointSearchIndLoop.size(); ++i) {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) <
                MappingConfig::historyKeyframeSearchTimeDiff) {
                continue;
            }
            double distance = 0.0;
            for (int j = id; j < loopKeyCur; ++j) {
                distance += keyframeDistances.at(j);
            }
            if (distance > 200) {
                loopKeyPre = id;
                break;
            }

        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;//TODO 1118 delete

//        //TODO 1118 what is the meaning of below if?
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
       // float loopKeyPre_x = copy_cloudKeyPoses2D->at(loopKeyPre).x;
      //  float loopKeyPre_y = copy_cloudKeyPoses2D->at(loopKeyPre).y;
       // float detectedDistance = sqrt(copy_cloudKeyPoses2D->at(loopKeyPre).x));

       // if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;//TODO delete backwards
        latestID = loopKeyCur;
        closestID = loopKeyPre;
        return true;
    }


    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                               const int &key, const int &searchNum) {

        //nearKeyframes->clear();//TODO delete
       // nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();

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

        pcl::PointCloud<PointType>::Ptr cloud_temp(
                new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;

    }

        void GetloopOptimization(){
        //1.得到触发回环后当前帧矫正之后的结果；
        //2.当前帧的最邻近的地图
        if (cloudKeyPoses3D->points.empty())//TODO remove copy
            return;

        if(loopKeyPre == -1)
            return;

        EZLOG(INFO)<<" loop happen!"<<endl;

        if(surfCloudKeyFrames[loopKeyCur]->size() ==0)
            return;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        pcl::KdTreeFLANN<PointType>::Ptr LoopkdtreeKeyFrame(new pcl::KdTreeFLANN<PointType>());
        pcl::KdTreeFLANN<PointType>::Ptr LoopkdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr LoopsurroundingKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr LoopsurroundingMap(new pcl::PointCloud<PointType>());
        vector <pcl::PointCloud<PointType>::Ptr> surfCloudKeyFramesContainer;

        LoopkdtreeKeyFrame->setInputCloud(
                cloudKeyPoses3D);//TODO remove copy
//TODO remove copy
        LoopkdtreeKeyFrame->radiusSearch(cloudKeyPoses3D->points[loopKeyCur], (double) MappingConfig::surroundingKeyframeSearchRadius,
                pointSearchInd, pointSearchSqDis);

//        for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
//                int id = pointSearchInd[i];
//            LoopsurroundingKeyFrames->push_back(copy_cloudKeyPoses3D->points[id]);
//        }
        //3.提取关键帧的点云构建局部地图;
            //extractCloud(surroundingKeyFrames);
        LoopsurroundingMap->clear();
        for (int i = 1; i < pointSearchInd.size(); ++i){
            int id = pointSearchInd[i];
            int thisKeyInd = (int)cloudKeyPoses3D->points[id].intensity;//TODO remove copy
                pcl::PointCloud <PointType> laserCloudSurfTemp =
                        *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);
                *LoopsurroundingMap += laserCloudSurfTemp;

        }

            LoopkdtreeSurfFromMap->setInputCloud(LoopsurroundingMap);
        //3.构建点面残差
            float current_loop_pose[6];
            current_loop_pose[0] = cloudKeyPoses6D->points.at(loopKeyCur).roll;
            current_loop_pose[1] = cloudKeyPoses6D->points.at(loopKeyCur).pitch;
            current_loop_pose[2] = cloudKeyPoses6D->points.at(loopKeyCur).yaw;
            current_loop_pose[3] = cloudKeyPoses6D->points.at(loopKeyCur).x;
            current_loop_pose[4] = cloudKeyPoses6D->points.at(loopKeyCur).y;
            current_loop_pose[5] = cloudKeyPoses6D->points.at(loopKeyCur).z;
           // EZLOG(INFO)<<"Loop current pose is "<<current_loop_pose<<endl;
            transPointAssociateToMap = trans2Affine3f(current_loop_pose);
            int loop_cur_num =  surfCloudKeyFrames[loopKeyCur]->size();
            EZLOG(INFO)<<"Loop current number is "<<loop_cur_num<<endl;

            float total_distance = 0.0f;
            int vaild_num = 0;

            for (int i = 0; i < loop_cur_num; i++) {
                PointType pointOri, pointSel, coeff;
                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;

                pointOri = surfCloudKeyFrames[loopKeyCur]->points[i];

                pointAssociateToMap(&pointOri, &pointSel);
                LoopkdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
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
                        matA0(j, 0) = LoopsurroundingMap->points[pointSearchInd[j]].x;
                        matA0(j, 1) = LoopsurroundingMap->points[pointSearchInd[j]].y;
                        matA0(j, 2) = LoopsurroundingMap->points[pointSearchInd[j]].z;
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
                        if (fabs(pa * LoopsurroundingMap->points[pointSearchInd[j]].x +
                                 pb * LoopsurroundingMap->points[pointSearchInd[j]].y +
                                 pc * LoopsurroundingMap->points[pointSearchInd[j]].z +
                                 pd) > 0.2) {
                            planeValid = false;
                            break;
                        }
                    }

                    ++vaild_num;

                    if (planeValid) {
                        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                        total_distance += abs(pd2);
                        EZLOG(INFO)<<"loop point to map is "<<pd2<<endl;
                    }
                }
            }
            EZLOG(INFO)<<"avarage distance error : "<< total_distance / vaild_num <<endl;

    }




//    void dataprocess(CloudFeature& cur_ft){
        //gnss data preprocess
//        Eigen::Quaterniond q_gnss_cur;
//        t_gnss_cur = cur_ft.pose.GetXYZ();
//        R_gnss_cur = cur_ft.pose.GetR();
//        q_gnss_cur = cur_ft.pose.GetQ();
//        q_gnss_rpy =Qua2Euler(q_gnss_cur);
//
//        //lidar data preprocess
//        Eigen::Quaterniond q_lidar_cur;
//        t_lidar_cur = cur_ft.DRPose.GetXYZ();
//        q_lidar_cur = cur_ft.DRPose.GetQ();
//        R_lidar_cur = cur_ft.DRPose.GetR();
//        current_DRPose = PoseT(t_lidar_cur,R_lidar_cur);
//        q_lidar_rpy = Qua2Euler(q_lidar_cur);
        //EZLOG(INFO)<<"q_lidar_rpy"<<q_lidar_rpy[0]<<q_lidar_rpy[1]<<q_lidar_rpy[2]<<endl;

//        noise_x = cur_ft.cov(0, 0);
//        noise_y = cur_ft.cov(1,0);
//        noise_z = cur_ft.cov(2,0) ;
//        EZLOG(INFO)<<"noise_x"<<noise_x<<"noise_y"<<noise_y<<"noise_z"<<noise_z;
//        noise_roll = cur_ft.cov(3,0);
//        noise_pitch = cur_ft.cov(4,0);
//        noise_yaw = cur_ft.cov(5,0);
//        EZLOG(INFO)<<"noise_roll"<<noise_roll<<"noise_pitch"<<noise_pitch<<"noise_yaw"<<noise_yaw;

//        IsRealiableGnss = cur_ft.pose_reliable;

 //   }

    void updateInitialGuess(CloudFeature& cur_ft) {
        //gnss data preprocess
        Eigen::Quaterniond q_gnss_cur;
        t_gnss_cur = cur_ft.pose.GetXYZ();
        R_gnss_cur = cur_ft.pose.GetR();
        q_gnss_cur = cur_ft.pose.GetQ();
        q_gnss_rpy =Qua2Euler(q_gnss_cur);

        //lidar data preprocess
        Eigen::Quaterniond q_lidar_cur;
        t_lidar_cur = cur_ft.DRPose.GetXYZ();
        q_lidar_cur = cur_ft.DRPose.GetQ();
        R_lidar_cur = cur_ft.DRPose.GetR();
        current_DRPose = PoseT(t_lidar_cur,R_lidar_cur);
        q_lidar_rpy = Qua2Euler(q_lidar_cur);

        IsRealiableGnss = cur_ft.pose_reliable;

       // incrementalOdometryAffineFront = trans2Affine3f(current_T_m_l);

        if (cloudKeyPoses3D->points.empty()) {
                if(cur_ft.pose_reliable == true){
                    EZLOG(INFO)<<"InitialGuess use GNSS"<<endl;
                    current_T_m_l[0] = q_gnss_rpy[0];
                    current_T_m_l[1] = q_gnss_rpy[1];
                    current_T_m_l[2] = q_gnss_rpy[2];

                    current_T_m_l[3] = t_gnss_cur[0];
                    current_T_m_l[4] = t_gnss_cur[1];
                    current_T_m_l[5] = t_gnss_cur[2];
                }else{
                    EZLOG(INFO)<<"InitialGuess use DR"<<endl;
                    current_T_m_l[0] = q_lidar_rpy[0];
                    current_T_m_l[1] = q_lidar_rpy[1];
                    current_T_m_l[2] = q_lidar_rpy[2];

                    current_T_m_l[3] = t_lidar_cur[0];
                    current_T_m_l[4] = t_lidar_cur[1];
                    current_T_m_l[5] = t_lidar_cur[2];
                }

            systemInitialized = true;
        }

        if (!systemInitialized) {
            EZLOG(INFO) << "sysyem need to be initialized. EXIT!!!" << endl;
            exit(1);
            return;
        }

        currentDRPose = pcl::getTransformation(
                t_lidar_cur[0], t_lidar_cur[1], t_lidar_cur[2],  q_lidar_rpy[0],  q_lidar_rpy[1],
                q_lidar_rpy[2]);

        if (DRInitialized == false) {
            lastDRPose = currentDRPose;
            last_DRPose = PoseT(t_lidar_cur,R_lidar_cur);
            DRInitialized = true;
            return;
        }
        Eigen::Affine3f transIncre =
                lastDRPose.inverse() * currentDRPose;
        Eigen::Affine3f transTobe = trans2Affine3f(current_T_m_l);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(
                transFinal, current_T_m_l[3], current_T_m_l[4],
                current_T_m_l[5], current_T_m_l[0],
                current_T_m_l[1], current_T_m_l[2]);

        lastDRPose = currentDRPose;
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {

        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();

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
                pcl::PointCloud <PointType> laserCloudCornerTemp =
                        *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud <PointType> laserCloudSurfTemp =
                        *transformPointCloud(surfCloudKeyFrames[thisKeyInd],
                                             &cloudKeyPoses6D->points[thisKeyInd]);

                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;

                laserCloudMapContainer[thisKeyInd] =
                        make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();

        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        if (laserCloudMapContainer.size() > 500)
            laserCloudMapContainer.clear();

    }

    void extractSurroundingKeyFrames() {

        if (cloudKeyPoses3D->points.empty()) {
            return;
        }
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
//
//        //TODO 1118 remove
//        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
//        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);//TODO 1118 ?
//
//
//        for (auto &pt: surroundingKeyPosesDS->points) {
//
//            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd,
//                                                      pointSearchSqDis);
//            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
//        }

        extractCloud(surroundingKeyPoses);
    }

    void downsampleCurrentScan() {

        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
        EZLOG(INFO)<<"Current lidar scan downsample corner: "
        << laserCloudCornerLastDSNum << " / " << laserCloudCornerLast->size() << std::endl;
        EZLOG(INFO)<<"Current lidar scan downsample corner: "
        << laserCloudSurfLastDSNum << " / " << laserCloudSurfLast->size() << std::endl;
    }

    //
    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(current_T_m_l);
    }

    void cornerOptimization() {

        updatePointAssociateToMap();

        for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                                pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

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

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;

                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

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

                    float lb =
                            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;

                    float lc =
                            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
                            a012 / l12;

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

    }

    void surfOptimization() {

        updatePointAssociateToMap();

        for (int i = 0; i < laserCloudSurfLastDSNum; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];

            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                              pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
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

    }

    void combineOptimizationCoeffs() {

        for (int i = 0; i < laserCloudCornerLastDSNum; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                ++OptCornerNum;
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }

        for (int i = 0; i < laserCloudSurfLastDSNum; ++i) {
            if (laserCloudOriSurfFlag[i] == true) {
                ++OptSurfNum;
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
                  false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(),
                  false);
    }

    bool LMOptimization(int iterCount) {

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

            float arx = (a11 * pointOri.x +a12 * pointOri.y - a13 * pointOri.z) * coeff.x+
                        (-a14 * pointOri.x - a15 * pointOri.y - crx * pointOri.z) * coeff.y +
                        (a16 * pointOri.x + a17 * pointOri.y - a18 * pointOri.z) * coeff.z;

           float ary   =  (a21 * pointOri.x +a22 * pointOri.y + a23 * pointOri.z) * coeff.x +
                          (a24 *pointOri.x + a25 * pointOri.y - a26 * pointOri.z) * coeff.z;

           float arz = (a31 * pointOri.x + a32 * pointOri.y ) * coeff.x +
                       (a33 * pointOri.x - a34 * pointOri.y) * coeff.y +
                       (a35 * pointOri.x + a36 * pointOri.y) * coeff.z;

            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;

            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);

        matAtA = matAt * matA;
        matAtB = matAt * matB;

        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);


        if (iterCount == 0) {

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

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true;  // converged
        }

        return false;  // keep optimizing
    }

    void scan2MapOptimization() {

        if (cloudKeyPoses3D->points.empty()) return;
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
                //EZLOG(INFO)<<"cornerOptimization cost time" << t1.toc()<<endl;
                 TicToc t2;
                surfOptimization();
                //EZLOG(INFO)<<"surfOptimization cost time " << t2.toc()<<endl;

                combineOptimizationCoeffs();

               // TicToc timer;
                if (LMOptimization(iterCount) == true)
                {
                    ++LMsuccess;
                    EZLOG(INFO)<<"LMOptimization success and OptCornerNum is: "
                               <<OptCornerNum << "/" << laserCloudCornerLastDSNum <<endl;
                    EZLOG(INFO)<<"LMOptimization success and OptSurfNum is: "
                               <<OptSurfNum << "/" << laserCloudSurfLastDSNum <<endl;
                    break;
                }

                if (iterCount == maxIters - 1){
                    ++LMfail;
                    EZLOG(INFO)<<"LMOptimization fail and OptCornerNum is: "
                               <<OptCornerNum << "/" << laserCloudCornerLastDSNum <<endl;
                    EZLOG(INFO)<<"LMOptimization fail and OptSurfNum is: "
                               <<OptSurfNum << "/" << laserCloudSurfLastDSNum <<endl;
                    EZLOG(INFO)<<"LMfail count is" << LMfail << "/" << LMsuccess << endl;
                }

            }

        }

    }

    //TODO 1118???
//    float constraintTransformation(float value, float limit) {
//        if (value < -limit) value = -limit;
//        if (value > limit) value = limit;
//
//        return value;
//    }

    bool IsKeyFrame() {

        if (cloudKeyPoses3D->points.empty()) return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());

        Eigen::Affine3f transFinal = pcl::getTransformation(
                current_T_m_l[3], current_T_m_l[4], current_T_m_l[5],
                current_T_m_l[0], current_T_m_l[1], current_T_m_l[2]);


        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        //TODO 1118 judge yaw only!
//        if (abs(roll) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
//            abs(pitch) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
//            abs(yaw) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
//            sqrt(x * x + y * y + z * z) < MappingConfig::surroundingkeyframeAddingDistThreshold)
//            return false;

        if (abs(yaw) < MappingConfig::surroundingkeyframeAddingAngleThreshold * 0.017453293  &&
            sqrt(x * x + y * y + z * z) < MappingConfig::surroundingkeyframeAddingDistThreshold)
            return false;

        keyframeDistances.push_back(sqrt(x * x + y * y));

        return true;
    }

    void addOdomFactor() {

        if (cloudKeyPoses3D->points.empty()) {
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-1, 1e-1, M_PI * M_PI, 1e8, 1e8, 1e8)//rpy xyz
                                    .finished());  // rad*rad, meter*meter

            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(cloudKeyPoses3D->size(), trans2gtsamPose(current_T_m_l),
                                              priorNoise));

            initialEstimate.insert(0, trans2gtsamPose(current_T_m_l));
        } else {

            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

            gtsam::Pose3 poseFrom =
                    pclPointTogtsamPose3(cloudKeyPoses6D->points.back());

            gtsam::Pose3 poseTo = trans2gtsamPose(current_T_m_l);

            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(),
                    poseFrom.between(poseTo), odometryNoise));

            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
            //isAddOdomKeyFrame = true;//TODO 1118

        }

    }
    void addDRFactor(){
        gtsam::Pose3 DR_current(gtsam::Rot3(current_DRPose.GetR()),
                                gtsam::Point3(current_DRPose.GetXYZ()));
        gtsam::Pose3 DR_last(gtsam::Rot3(last_DRPose.GetR()),
                             gtsam::Point3(last_DRPose.GetXYZ()));
        gtsam::noiseModel::Diagonal::shared_ptr DRNoise =
                gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 8).finished());
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(),
                DR_last.between(DR_current), DRNoise));
        last_DRPose = current_DRPose;
//        gtsam::Pose3 DR_current;
//        lastDRPose = CurrentDRPose;
    }


    void addGPSFactor() {

        if (deque_cloud.empty()) return;
        if (cloudKeyPoses3D->points.empty() )//|| cloudKeyPoses3D->points.size() == 1
            return;
        static float magic_number = 50.0f;
        if(IsRealiableGnss == false){
            magic_number = 50.0f;
            return;
        }
        EZLOG(INFO)<<"IsRealiableGnss is good!!"<<endl;

        //static PointType lastGPSPoint;
        //TODO 1118

             // EZLOG(INFO)<<"noise x "<<noise_x<<","<<noise_y<<" ,"<<noise_z<<","<<noise_roll<<","<<noise_pitch<<","<<noise_yaw<<","<<endl;

        if (abs(noise_x) > SensorConfig::gpsCovThreshold || abs(noise_y) > SensorConfig::gpsCovThreshold)
            return;

        double gps_x = t_gnss_cur[0];
        double gps_y = t_gnss_cur[1];
        const float s = 0.05;
       // double gps_z = s * t_gnss_cur[2] + (1 - s) * current_T_m_l[5];
        double gps_z = current_T_m_l[5];

        if (!SensorConfig::useGpsElevation) {
            gps_z = current_T_m_l[5];
            noise_z = 0.001;
        }

//            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
//                break;
//            PointType curGPSPoint;
//            curGPSPoint.x = gps_x;
//            curGPSPoint.y = gps_y;
//            curGPSPoint.z = gps_z;
//
//            if (pointDistance(curGPSPoint, lastGPSPoint) < 4.0)
//                break;
//            lastGPSPoint = curGPSPoint;

        gtsam::Vector Vector6(6);
       // Vector3 << max(noise_x, 1.0), max(noise_y, 1.0), max(noise_z, 1.0);
       //Vector6<<
        Vector6<< 1e-1,1e-1,1e-1,2e-2 * magic_number,2e-2 * magic_number,2e-2 * magic_number;
       // Vector6 << 1e-4,1e-4,1e-4,1e-4,1e-4,1e-4;
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

        gtsam::Pose3 gnss_pose_gtsam = gtsam::Pose3(
                gtsam::Rot3(R_gnss_cur),
                gtsam::Point3(t_gnss_cur[0], t_gnss_cur[1], t_gnss_cur[2]));

        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(cloudKeyPoses3D->size() , gnss_pose_gtsam,
                                                    priorNoise));

        //EZLOG(INFO)<<"ADD GPS factor successfully!"<<endl;
        isAddGnssKeyFrame == true;

        magic_number = max( 1.0f , magic_number - 3.0f);

    }

    void addLoopFactor() {
        if (loopIndexQueue.empty()) return;
        EZLOG(INFO)<< "loopIndexQueue size"<< loopIndexQueue.size();
        for (int i = 0; i < (int) loopIndexQueue.size(); ++i) {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
           // mtxGraph.lock();
            gtSAMgraph.add(
                    gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
           // mtxGraph.unlock();
           // EZLOG(INFO)<<"ADD loop factor successfully"<<endl;
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        isAddloopFrame = true;
    }

    //TODO: FactorOpt and Save cloud
    void FactorOptAndSaveCloud() {
        gtsam::ISAM2Params parameters = gtsam::ISAM2Params();//TODO is work?
        parameters.factorization = gtsam::ISAM2Params::QR;

        //TODO change function name!!!
        if (IsKeyFrame() == false) return;
        // odom factor
        addOdomFactor();
        //EZLOG(INFO)<<""
        if(cloudKeyPoses3D->size() > 0){
            addDRFactor();
        }

        //useGPS TODO 1118
        addGPSFactor();
        addLoopFactor();

        gtSAMgraph.print("GTSAM Graph:\n");// update iSAM

        isam->update(gtSAMgraph, initialEstimate);
        if(isAddloopFrame){
            for(int i = 0; i < 3; ++i){
                isam->update();
            }
        }
        isam->update();
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 latestEstimate;
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate =
                isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();

        thisPose3D.intensity =
                cloudKeyPoses3D->size();  // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time= timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        current_T_m_l[0] = latestEstimate.rotation().roll();
        current_T_m_l[1] = latestEstimate.rotation().pitch();
        current_T_m_l[2] = latestEstimate.rotation().yaw();
        current_T_m_l[3] = latestEstimate.translation().x();
        current_T_m_l[4] = latestEstimate.translation().y();
        current_T_m_l[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        //pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        CloudInfoFt cloud_info;
        cloud_info.frame_id = ++frame_id;
        cloud_info.corner_cloud = thisCornerKeyFrame;
        cloud_info.surf_cloud = thisSurfKeyFrame;
        map_saver.AddCloudToSave(cloud_info);

    }

    void correctPoses() {
        if (cloudKeyPoses3D->points.empty())
            return;

        int numPoses = isamCurrentEstimate.size();
        PoseT gloabal_corrected_pose;
        gloabal_corrected_pose = isamCurrentEstimate.at<gtsam::Pose3>(numPoses - 1).matrix();
        opt_poses.push_back(gloabal_corrected_pose);
        static int loopNum = 0;
        ////TODO
        //if (isAddloopFrame == true || isAddGnssKeyFrame == true )
        {
           // if (isAddGnssKeyFrame == true ) {
            // clear map cache
            laserCloudMapContainer.clear();
            int start_idx = isAddloopFrame == true ? 0 : max(0,numPoses - 40);

            for (int i = start_idx; i < numPoses; ++i)  {
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

                PoseT gloabal_corrected_pose;
                gloabal_corrected_pose = isamCurrentEstimate.at<gtsam::Pose3>(i).matrix();
                opt_poses[i] = gloabal_corrected_pose;

            }

            map_saver.SavePoses(opt_poses);
            if(isAddloopFrame){
                ++loopNum;
                if(loopNum > 2){
                    GetloopOptimization();
                    EZLOG(INFO)<< "MAPPING Finished!"<<endl;
                    exit(1);
                }

            }
            isAddloopFrame = false;
            isAddGnssKeyFrame == false;
           // isAddOdomKeyFrame == false;

        }

    }


    //发布优化后的里程计
    void publishOdometry() {

        OdometryType current_lidar_pose_world;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(current_T_m_l);

        current_lidar_pose_world.frame = "map";
        current_lidar_pose_world.timestamp = timeLaserInfoCur;
        current_lidar_pose_world.pose.pose = Lidarodom_2_map.matrix().cast<double>();
//
        pubsub->PublishOdometry(topic_current_pose, current_lidar_pose_world);

    }

    void publishCloud(){
        if (cloudKeyPoses3D->points.empty()) return;
        pcl::PointCloud<PointType>::Ptr cloudOutCorner(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cloudOutSurf(
                new pcl::PointCloud<PointType>());

        PointTypePose thisPose6D = trans2PointTypePose(current_T_m_l);
        *cloudOutCorner += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
        *cloudOutSurf += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);

        CloudTypeXYZI corner_pub,surf_pub;
        corner_pub.frame = "map";
        corner_pub.timestamp = timeLaserInfoCur;
        corner_pub.cloud = *cloudOutCorner;

        surf_pub.frame = "map";
        surf_pub.timestamp = timeLaserInfoCur;
        surf_pub.cloud = *cloudOutSurf;
        pubsub->PublishCloud(topic_this_Corner_Keyframe, corner_pub);
        pubsub->PublishCloud(topic_this_Surf_Keyframe, surf_pub);

        pcl::PointCloud<PointType>::Ptr localMapOut(
                new pcl::PointCloud<PointType>());
        *localMapOut += *laserCloudCornerFromMapDS;
        *localMapOut += *laserCloudSurfFromMapDS;
        CloudTypeXYZI map_pub;
        map_pub.frame = "map";
        map_pub.timestamp = timeLaserInfoCur;
        map_pub.cloud = *localMapOut;
        pubsub->PublishCloud(topic_localmapOut_Keyframe, map_pub);

    }


    void DoWork(){

        while(1){

            bool isEmpty = false;
            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                isEmpty = deque_cloud.empty();
            }

            if(!isEmpty){

                CloudFeature cur_ft;
                cloud_mutex.lock();
                cur_ft = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();

                std::lock_guard<std::mutex> lock(mtx);

                static double timeLastProcessing = -1;
                if (timeLaserInfoCur - timeLastProcessing < MappingConfig::mappingProcessInterval) {
                    continue;
                }
              //  if (IsKeyFrame() == true) {//TODO keyframe starategy
                  //  EZLOG(INFO)<<"IS keyFRame"<<endl;
              //  timeLastProcessing = timeLaserInfoCur;

                TicToc t0;
                //dataprocess(cur_ft);//TODO 1118 dataprocess work
                //EZLOG(INFO)<<" dataprecess COST TIME"<<t0.toc()<<endl;
                TicToc t6;
                updateInitialGuess(cur_ft);//TODO
                //EZLOG(INFO)<<" updateInitialGuess COST TIME"<<t6.toc()<<endl;

                if (!systemInitialized)
                    continue;

                current_frame_id = cur_ft.frame_id;
                timeLaserInfoCur = cur_ft.timestamp;
                laserCloudCornerLast = cur_ft.cornerCloud;
                laserCloudSurfLast = cur_ft.surfaceCloud;

              //  if(IsKeyFrame() == true){
                    TicToc t1;
                    extractSurroundingKeyFrames();//lock

                    downsampleCurrentScan();

                    TicToc t3;
                    scan2MapOptimization();
                    //EZLOG(INFO)<<" scan2MapOptimization COST TIME"<<t3.toc()<<endl;

                    TicToc t4;
                    FactorOptAndSaveCloud();//lock
                    //EZLOG(INFO)<<" saveKeyFramesAndFactor COST TIME"<<t4.toc()<<endl;

                    TicToc t5;
                    correctPoses();//lock

                    EZLOG(INFO)<<" correctPoses COST TIME"<<t5.toc()<<endl;
                    publishOdometry();
                    publishCloud();

                  //通过建文件夹来判断

              //  }

            }else{
                sleep(0.001);
            }
        }
    }

    void AddCloudData(const CloudFeature& cloud_ft){
        std::lock_guard<std::mutex> lock(cloud_mutex);
        deque_cloud.push_back(cloud_ft);

    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        allocateMemory();
        pubsub->addPublisher(topic_this_Corner_Keyframe, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_this_Surf_Keyframe, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_current_pose,DataType::ODOMETRY,10);
        pubsub->addPublisher(topic_cloudOut_Keyframe,DataType::LIDAR,10);
        pubsub->addPublisher(topic_localmapOut_Keyframe,DataType::LIDAR,10);

        do_work_thread = new std::thread(&OPTMapping::DoWork, this);
        loop_thread =new std::thread(&OPTMapping::loopClosureThread, this);
        save_Map_thread = new std::thread(&MapSaver::do_work, &(OPTMapping::map_saver));
       // savepathThread = new std::thread(&OPTMapping::savePathThread, this);

    }
};
#endif //SEU_LIDARLOC_OPT_MAPPING_H
