//
// Created by fyy on 23-9-10.
//

#ifndef SEU_LIDARLOC_OPT_LOPC_H
#define SEU_LIDARLOC_OPT_LOPC_H

#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "gtsam/nonlinear/ISAM2Params.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>  // gtsam

//#include <std_srvs/Empty.h>

#include "GeoGraphicLibInclude/Geocentric.hpp"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "GeoGraphicLibInclude/Geoid.hpp"

class LOCMapping{
public:
    enum FRAME {
        LIDAR,    // 0
        WORLD    // 1
    };
    FRAME current_frame = WORLD;
    PubSubInterface* pubsub;
    std::function<void(const OdometryType&)> Function_AddOdometryTypeToIMUPreintegration;

    std::thread* do_work_thread;
    std::mutex cloud_mutex;
    std::mutex gnss_ins_mutex;

    std::deque<CloudFeature> deque_cloud;
    std::deque<GNSSINSType> deque_gnssins;

    std::string topic_priorMap_surf= "/priorMap_surf";
    std::string topic_priorMap_corner = "/priorMap_corner";
    std::string topic_lidar_odometry = "/lidar_odometry";

    pcl::PointCloud<PointType>::Ptr Keyframe_Poses3D;  //历史关键帧 位置
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointType>::Ptr current_corner;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_surf;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_corner_ds;  // downsampled corner feature set from
    pcl::PointCloud<PointType>::Ptr current_surf_ds;  // downsampled surf feature set from
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    pcl::PointCloud<PointType>::Ptr localMap_corner;
    pcl::PointCloud<PointType>::Ptr localMap_surf;
    pcl::PointCloud<PointType>::Ptr priorMap_corner_ds;
    pcl::PointCloud<PointType>::Ptr priorMap_surf_ds;
    pcl::PointCloud<PointType>::Ptr localMap_corner_ds;
    pcl::PointCloud<PointType>::Ptr localMap_surf_ds;
    pcl::PointCloud<PointType>::Ptr priorMap_corner;// 先验地图角点
    pcl::PointCloud<PointType>::Ptr priorMap_surf; // 先验地图面点
    pcl::PointCloud<PointTypePose>::Ptr Keyframe_Poses6D;  //历史关键帧 位姿
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_Keyframe_Poses3D;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_priorMap_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_priorMap_corner;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_corner;
    vector<pcl::PointCloud<PointType>::Ptr> Keyframe_corner_ds;
    vector<pcl::PointCloud<PointType>::Ptr> Keyframe_surf_ds;
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> localMap_corner_and_surf;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> near_keyframe_poses_ds;        // for surrounding key poses of

    std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    std::vector<bool> laserCloudOriCornerFlag;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;

    Eigen::Vector3d t_w_cur;
    Eigen::Quaterniond q_w_cur;
    Eigen::Matrix3d q_w_cur_matrix;
    double q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;
    int frame_cnt = 0;
    int flagLoadMap = 1;
    int flag_need_load_localMap = 1;

    int Keyframe_Poses3D_last_num = 3;
    int flag_scan_mode_change = 0;
    int flag_load_localMap = 0;
    int localMap_corner_ds_num = 0;
    int localMap_surf_ds_num = 0;
    int current_corner_ds_num = 0;
    int current_surf_ds_num = 0;
    int current_frameID = 0;
    int current_lidar_frameID = 0;
    int last_lidar_frameID = 0;
    int flag_use_world_localMap = 1;

    gtsam::Pose3 last_gnss_poses;
    gtsam::ISAM2 *isam;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values optimizedEstimate;
    gtsam::Values isamCurrentEstimate; // 所有关键帧位姿的优化结果
    Eigen::MatrixXd poseCovariance;
    float current_T_m_l[6]; // roll pitch yaw x y z
    double timeLaserInfoCur;
//    double timeLaserInfoStamp;
    bool isDegenerate = false;
    cv::Mat matP;

    Eigen::Vector3d originLLA;
    bool systemInitialized = false;
    bool gpsTransfromInit = false;
    GeographicLib::LocalCartesian geo_converter;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    std::mutex mtx;
    std::mutex mtxGpsInfo;
    std::mutex mtxGraph;
    int lidarScan_cnt = 0;

    void AddCloudData(const CloudFeature& cloud_ft){
        if(lidarScan_cnt > SensorConfig::lidarScanDownSample){
            cloud_mutex.lock();
            deque_cloud.push_back(cloud_ft);
            cloud_mutex.unlock();
            lidarScan_cnt = 0;
        }
        else{
            lidarScan_cnt++;
        }

    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){
        gnss_ins_mutex.lock();
//        deque_gnssins.push_back(gnss_ins_data);
        gnss_ins_mutex.unlock();
    }

    void allocateMemory() {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        Keyframe_Poses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());
        Keyframe_Poses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtree_Keyframe_Poses3D.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        current_corner.reset(
                new pcl::PointCloud<PointType>());  // corner feature set from
        // odoOptimization
        current_surf.reset(
                new pcl::PointCloud<PointType>());  // surf feature set from
        // odoOptimization
        current_corner_ds.reset(
                new pcl::PointCloud<PointType>());  // downsampled corner featuer set
        // from odoOptimization
        current_surf_ds.reset(
                new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
        // odoOptimization

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

        localMap_corner.reset(new pcl::PointCloud<PointType>());
        localMap_surf.reset(new pcl::PointCloud<PointType>());
        localMap_corner_ds.reset(new pcl::PointCloud<PointType>());
        localMap_surf_ds.reset(new pcl::PointCloud<PointType>());

        priorMap_corner.reset(new pcl::PointCloud<PointType>); // 先验地图角点
        priorMap_surf.reset(new pcl::PointCloud<PointType>);    // 先验地图面点
        kdtree_priorMap_surf.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_priorMap_corner.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_localMap_surf.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_localMap_corner.reset(new pcl::KdTreeFLANN<PointType>);

        for (int i = 0; i < 6; ++i) {
            current_T_m_l[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        last_lidar_frameID = 0;
        current_lidar_frameID = 0;
        downSizeFilterCorner.setLeafSize(MappingConfig::mappingCornerLeafSize,
                                         MappingConfig::mappingCornerLeafSize,
                                         MappingConfig::mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(MappingConfig::mappingSurfLeafSize,
                                       MappingConfig::mappingSurfLeafSize,
                                       MappingConfig::mappingSurfLeafSize);
        near_keyframe_poses_ds.setLeafSize(
                MappingConfig::surroundingKeyframeDensity,
                MappingConfig::surroundingKeyframeDensity,
                MappingConfig::surroundingKeyframeDensity);  // for surrounding key poses of

        if(flagLoadMap){
            // 从晓强接收
            Load_PriorMap_Surf(MappingConfig::prior_map_surf,priorMap_surf);
            Load_PriorMap_Corner(MappingConfig::prior_map_corner,priorMap_corner);

        }


    }
    void Load_PriorMap_Corner(std::string mapFile,pcl::PointCloud<PointType>::Ptr _priorMap_corner)
    {

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(mapFile,*_priorMap_corner) == -1)
        {
            std::cout << "Could not read Map!"<<std::endl;
            return ;
        }
//        pcl::transformPointCloud(*_priorMap_corner, *_priorMap_corner, SensorConfig::T_L_B);
//        kdtree_priorMap_corner->setInputCloud(_priorMap_corner);//kdtreeCornerFromMap
        std::cout << "Read Corner Map!"<<std::endl;

    }
    /**
 * add by sy for load Prior map Surf
 * @param mapFile
 */
    void Load_PriorMap_Surf(std::string mapFile, pcl::PointCloud<PointType>::Ptr _priorMap_surf)
    {

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(mapFile,*_priorMap_surf) == -1)
        {
            std::cout << "Could not read Map!"<<std::endl;
            return ;
        }
//        change Map from lidar axis to world axis
//        pcl::transformPointCloud(*_priorMap_surf, *_priorMap_surf, SensorConfig::T_L_B);
//        kdtree_priorMap_surf->setInputCloud(_priorMap_surf);//kdtreeSurfFromMap
        std::cout << "Read Surf Map!"<<std::endl;
    }
    void pub_CornerAndSurfFromMap()
    {
//      PriorMap surf
        CloudTypeXYZI PriorMap_surf_pub;
        PriorMap_surf_pub.frame = "map";
        PriorMap_surf_pub.timestamp = timeLaserInfoCur;
        PriorMap_surf_pub.cloud = *priorMap_surf;
        pubsub->PublishCloud(topic_priorMap_surf, PriorMap_surf_pub);
        std::cout << "Pub Surf Map!"<<std::endl;

//      PriorMap Corner
        CloudTypeXYZI PriorMap_corner_pub;
        PriorMap_corner_pub.frame = "map";
        PriorMap_corner_pub.timestamp = timeLaserInfoCur;
        PriorMap_corner_pub.cloud = *priorMap_corner;
        pubsub->PublishCloud(topic_priorMap_corner, PriorMap_corner_pub);
        std::cout << "Pub Corner Map!"<<std::endl;

    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po) {
        if(MappingConfig::scan_2_prior_map == 1){
            po->x = T_wl(0, 0) * pi->x +
                    T_wl(0, 1) * pi->y +
                    T_wl(0, 2) * pi->z +
                    T_wl(0, 3);
            po->y = T_wl(1, 0) * pi->x +
                    T_wl(1, 1) * pi->y +
                    T_wl(1, 2) * pi->z +
                    T_wl(1, 3);
            po->z = T_wl(2, 0) * pi->x +
                    T_wl(2, 1) * pi->y +
                    T_wl(2, 2) * pi->z +
                    T_wl(2, 3);
            po->intensity = pi->intensity;
        }
        else{
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
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                        PointTypePose *transformIn) {

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

    void updateInitialGuess() {
        // save current transformation before any processing
        static Eigen::Affine3f lastImuTransformation;

        // initialization the first frame
//        根据gps信息，加载地图
        if (Keyframe_Poses3D->points.empty()) {
            systemInitialized = false;
            if (SensorConfig::useGPS) {
                ;
            } else {
                current_T_m_l[0] = q_w_cur_roll;
                current_T_m_l[1] = q_w_cur_pitch;
                current_T_m_l[2] = q_w_cur_yaw;
                current_T_m_l[3] = t_w_cur[0];
                current_T_m_l[4] = t_w_cur[1];
                current_T_m_l[5] = t_w_cur[2];
                lastImuTransformation = pcl::getTransformation(0, 0, 0, 0, 0,0);
                systemInitialized = true;
                return;
            }
        }

        if (!systemInitialized) {
            EZLOG(INFO)<<"sysyem need to be initialized";
            return;
        }

//      世界系
        Eigen::Affine3f pose_guess_from_gnss = pcl::getTransformation(
                t_w_cur[0], t_w_cur[1], t_w_cur[2],q_w_cur_roll,
                q_w_cur_pitch , q_w_cur_yaw);
//      current_T_m_l -> 世界系
        pcl::getTranslationAndEulerAngles(
                pose_guess_from_gnss, current_T_m_l[3], current_T_m_l[4],
                current_T_m_l[5], current_T_m_l[0],
                current_T_m_l[1], current_T_m_l[2]);
        current_frame = WORLD;
    }
    void MODE_SCAN_2_LOCALMAP()
    {
        ;
    }
    void MODE_SCAN_2_SCAN()
    {
        if(Keyframe_Poses3D->points.empty() != true)
        {
            EZLOG(INFO) << "MODE: Scan to SCAN!";
            extractNearby();
        }
    }
    void MODE_SCAN_2_PRIORMAP()
    {
        ;
    }

/**
 * extract init points from map
 */
    void extractFromPriorMap(){
        std::cout <<"Keyframe_Poses3D->points size : "<< Keyframe_Poses3D->points.size() <<std::endl;

        TicToc extract_from_priorMap;
//        flag_scan_mode_change = 0 init || flag_scan_mode_change == 1
        if(MappingConfig::scan_2_prior_map == 1 ){ //init load local map from prior
            PointType init_point; //can be changed in gnss map
            if(Keyframe_Poses3D->points.empty() ){
                init_point.x = 0.;
                init_point.y = 0.;
                init_point.z = 0.;
                init_point.intensity = 0;
            }
            else{
                init_point.x = t_w_cur.x();
                init_point.y = t_w_cur.y();
                init_point.z = t_w_cur.z();
                init_point.intensity = current_frameID;
            }
//            EZLOG(INFO) << "MODE: Scan to Map!";
            // 添加标志位，需要下一帧再加载。现在可以先写着1.5s换一次local map
            current_lidar_frameID =  init_point.intensity;
//            flag_load_localMap = 1; // unecessary to load local map
            if(current_lidar_frameID - last_lidar_frameID < MappingConfig::LocalMap_updata_perframe && !flagLoadMap){
                EZLOG(INFO)<<"already have local map and don't need to update";

//                flag_need_load_localMap = 0; // do not need to load map
                return ; // 不需要重复加载localMap_corner_and_surf,但是如果没有加载过localMap就加载一次
            }
            flagLoadMap = 0;
//            flag_need_load_localMap = 1; // need to load map
            localMap_corner->clear();
            localMap_surf->clear();
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
//            world frame
//            EZLOG(INFO)<<"init_point is: " << init_point; // be aware of init_point's frame

            kdtree_priorMap_surf->setInputCloud(priorMap_surf);
            kdtree_priorMap_surf->radiusSearch(
                    init_point, (double) MappingConfig::localMap_searchRadius_surf,
                    pointSearchInd, pointSearchSqDis);
//            std::cout<<"surf points in local map  : "<<pointSearchInd.size()<<std::endl;
            for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
                int id = pointSearchInd[i];
                localMap_surf->push_back(priorMap_surf->points[id]);
            }
            pointSearchInd.clear();
            pointSearchSqDis.clear();
            kdtree_priorMap_corner->setInputCloud(priorMap_corner);
            kdtree_priorMap_corner->radiusSearch(
                    init_point, (double) MappingConfig::localMap_searchRadius_corner,
                    pointSearchInd, pointSearchSqDis);
            std::cout<<"corner points in local map ?  : "<<pointSearchInd.size()<<std::endl;
            for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
                int id = pointSearchInd[i];
                localMap_corner->push_back(priorMap_corner->points[id]);
            }
//            pub local map

//            kdtree_localMap_corner->setInputCloud(localMap_corner);
            std::cout<<"extract_from_priorMap is in ms: " << extract_from_priorMap.toc() <<std::endl;
            downSizeFilterCorner.setInputCloud(localMap_corner);
            downSizeFilterCorner.filter(*localMap_corner_ds);
            localMap_corner_ds_num = localMap_corner_ds->size();
            // Downsample the surrounding surf key frames (or map)
            downSizeFilterSurf.setInputCloud(localMap_surf);
            downSizeFilterSurf.filter(*localMap_surf_ds);
            localMap_surf_ds_num = localMap_surf_ds->size();
            //P_l = (T_wl)^-1 * P_w
//            如果先验地图是世界系的，需要转换成激光雷达系
            Eigen::Matrix4f T_matrix_L_B = SensorConfig::T_L_B.cast<float>();
            pcl::transformPointCloud(*localMap_surf_ds, *localMap_surf_ds, T_matrix_L_B.inverse());
            pcl::transformPointCloud(*localMap_corner_ds, *localMap_corner_ds, T_matrix_L_B.inverse());
            flag_use_world_localMap = 1;
            flag_load_localMap = 1; // load prior local map
        }
        else //else scan to scan_incremental  flag_scan_mode_change == 0
        {
            if(Keyframe_Poses3D->points.empty() != true)
            {
//                EZLOG(INFO) << "MODE: Scan to SCAN!";
                extractNearby();
            }
        }
        std::cout << " localMap_surf_ds_num is: "<<localMap_surf_ds_num
                  << " localMap_corner_ds_num is: "<<localMap_corner_ds_num<<std::endl;
        if(localMap_surf_ds_num > MappingConfig::scan_2_scan_num_surf
           ||localMap_corner_ds_num > MappingConfig::scan_2_scan_num_corner)
        {
//            EZLOG(INFO) << "MODE: Change mode ->>>>>>>>> Scan to Map!";
            flag_scan_mode_change = 1;//change loc mode to scan 2 map
        }
//        localMap_corner_and_surf[current_lidar_frameID] =
//                make_pair(*localMap_corner_ds, *localMap_surf_ds);
        last_lidar_frameID = current_lidar_frameID;
//        std::cout << "localMap_corner_and_surf size is :" << localMap_corner_and_surf.size() << std::endl;

    }
    void extractLastScan()
    {
        ;
    }
    /**
     * 根據輸入點雲構造KD树
     */
    void  extractNearby() {

        pcl::PointCloud<PointType>::Ptr near_keyframe_poses_temp(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr near_keyframe_poses_ds_temp(
                new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        ///////////////////////////////////////////////////////  need points from local map -> localCloudKeyPoses3D
        // 歷史關鍵幀xyz，用於搜索当前关键帧
        kdtree_Keyframe_Poses3D->setInputCloud(
                Keyframe_Poses3D);  // create kd-tree
        kdtree_Keyframe_Poses3D->radiusSearch(
                Keyframe_Poses3D->back(), (double) MappingConfig::surroundingKeyframeSearchRadius,//50
                pointSearchInd, pointSearchSqDis);
        std::cout << "kdtree_Keyframe_Poses3D we find : " << pointSearchInd.size()<<std::endl;
        for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
            int id = pointSearchInd[i];
            near_keyframe_poses_temp->push_back(Keyframe_Poses3D->points[id]);
        }

        near_keyframe_poses_ds.setInputCloud(near_keyframe_poses_temp);
        near_keyframe_poses_ds.filter(*near_keyframe_poses_ds_temp);
//        消除降采样的影响，对杂揉点云重新赋索引
        for (auto &pt : near_keyframe_poses_ds_temp->points) {
            kdtree_Keyframe_Poses3D->nearestKSearch(pt, 1, pointSearchInd,
                                                    pointSearchSqDis);
            pt.intensity = Keyframe_Poses3D->points[pointSearchInd[0]].intensity;
            std::cout<<"pt.intensity: "<<pt.intensity<<std::endl;
        }
        // also extract some latest key frames in case the robot rotates in one
        // position
//        10s内的位置信息存入点云
        int numPoses = Keyframe_Poses3D->size();
        std::cout<<"num of HistroyPoses: "<<numPoses<<std::endl;
        for (int i = numPoses - 1 ; i >= 0; --i) {
            if (timeLaserInfoCur - Keyframe_Poses6D->points[i].time < 10.0)
                near_keyframe_poses_ds_temp->push_back(Keyframe_Poses3D->points[i]);
            else
                break;
        }
//        surroundingKeyPosesDS这里只有激光的xyz，没有点云信息，需要把点云提取出来
        std::cout<<"num of near_keyframe_poses_ds_temp: "<<near_keyframe_poses_ds_temp->size()<<std::endl;
        extractCloud(near_keyframe_poses_ds_temp);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
        // fuse the map
//      Local Map from Prior Map.
        localMap_corner->clear();
        localMap_surf->clear();

        for (int i = 0; i < (int) cloudToExtract->size(); ++i) {
            if (pointDistance(cloudToExtract->points[i], Keyframe_Poses3D->back()) >
                MappingConfig::surroundingKeyframeSearchRadius){
//                remove far points 点不能距离关键帧太远 //5m or 30m or 50m
                continue;
            }

            int thisKeyInd = (int) cloudToExtract->points[i].intensity;
//            point found
            if (localMap_corner_and_surf.find(thisKeyInd) != localMap_corner_and_surf.end()) {
                // transformed cloud available
                *localMap_corner += localMap_corner_and_surf[thisKeyInd].first;
                *localMap_surf += localMap_corner_and_surf[thisKeyInd].second;
            } else { //not found
                // transformed cloud not available
                pcl::PointCloud<PointType> localMap_corner_temp =
                        *transformPointCloud(Keyframe_corner_ds[thisKeyInd],
                                             &Keyframe_Poses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> localMap_surf_temp =
                        *transformPointCloud(Keyframe_surf_ds[thisKeyInd],
                                             &Keyframe_Poses6D->points[thisKeyInd]);
                *localMap_corner += localMap_corner_temp;
                *localMap_surf += localMap_surf_temp;
                localMap_corner_and_surf[thisKeyInd] =
                        make_pair(localMap_corner_temp, localMap_surf_temp);
                std::cout << "localMap_corner_and_surf size is :" << localMap_corner_and_surf.size() << std::endl;

            }
        }
        downSizeFilterCorner.setInputCloud(localMap_corner);
        downSizeFilterCorner.filter(*localMap_corner_ds);
        localMap_corner_ds_num = localMap_corner_ds->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(localMap_surf);
        downSizeFilterSurf.filter(*localMap_surf_ds);
        localMap_surf_ds_num = localMap_surf_ds->size();

        if (localMap_corner_and_surf.size() > 1000) localMap_corner_and_surf.clear();
    }

    /**
     * current_corner_ds 当前帧角点降采样
     * current_surf_ds 当前帧面点降采样
     */
    void downsampleCurrentScan() {

        // Downsample cloud from current scan
        current_corner_ds->clear();
        downSizeFilterCorner.setInputCloud(current_corner);
        downSizeFilterCorner.filter(*current_corner_ds);
        current_corner_ds_num = current_corner_ds->size();

        current_surf_ds->clear();
        downSizeFilterSurf.setInputCloud(current_surf);
        downSizeFilterSurf.filter(*current_surf_ds);
        current_surf_ds_num = current_surf_ds->size();
        EZLOG(INFO) << "current_corner_ds_num: "<<current_corner_ds_num
                    << " current_surf_ds_num" <<current_surf_ds_num;

    }

    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(current_T_m_l);
    }

    void pointAssociateToWorld_noTrans(PointType const *const pi, PointType *const po){
//        accord with current_T_m_l
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

    void pointAssociateToWorld_withTrans(PointType const *const pi, PointType *const po) {
//              change current_T_m_l from lidar to world
//        T_wl -> from evo * lidar = T_wl
        po->x = T_wl(0, 0) * pi->x +
                T_wl(0, 1) * pi->y +
                T_wl(0, 2) * pi->z +
                T_wl(0, 3);
        po->y = T_wl(1, 0) * pi->x +
                T_wl(1, 1) * pi->y +
                T_wl(1, 2) * pi->z +
                T_wl(1, 3);
        po->z = T_wl(2, 0) * pi->x +
                T_wl(2, 1) * pi->y +
                T_wl(2, 2) * pi->z +
                T_wl(2, 3);
        po->intensity = pi->intensity;
    }

    void transforPoint2World() {

        Eigen::Matrix4d T_wm_temp = SensorConfig::T_L_B;
        Eigen::Matrix3d R_wm_temp = T_wm_temp.cast<double>().block<3, 3>(0, 0);
        Eigen::Vector3d t_wm_temp = T_wm_temp.cast<double>().block<3, 1>(0, 3);
        Eigen::Affine3f T_wm;
        T_wm.linear() = R_wm_temp.cast<float>();
        T_wm.translation() = t_wm_temp.cast<float>();

        Eigen::Affine3f T_ml = trans2Affine3f(current_T_m_l);
        T_wl = T_wm * T_ml; // T_wl = T_wm * T_ml
    }

    void cornerOptimization() {
        switch (current_frame) {
            case WORLD:
//                EZLOG(INFO)<<"cal in World frame";
                updatePointAssociateToMap();
                break;
            case LIDAR:
//                EZLOG(INFO)<<"cal in Lidar frame";
                transforPoint2World();
                break;
            default:
//                EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                break;
        }

//        std::cout<<" transPointAssociateToMap: "<<std::endl;
//        std::cout<<transPointAssociateToMap.matrix()<<std::endl;

//#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < current_corner_ds_num; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = current_corner_ds->points[i];
//            激光系转地图系
            switch (current_frame) {
                case WORLD:
//                    EZLOG(INFO)<<"current_T_m_l already in  world_corner";
                    pointAssociateToWorld_noTrans(&pointOri, &pointSel);
                    break;
                case LIDAR:
//                    EZLOG(INFO)<<"change current_T_m_l to world_corner";
                    pointAssociateToWorld_withTrans(&pointOri, &pointSel);
                    break;
                default:
                    EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                    break;
            }

            kdtree_localMap_corner->nearestKSearch(pointSel, 5, pointSearchInd,
                                                   pointSearchSqDis);
//            std::cout<<"pointSearchSqDis[4]  "<<pointSearchSqDis[4]<<std::endl;
            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += localMap_corner_ds->points[pointSearchInd[j]].x;
                    cy += localMap_corner_ds->points[pointSearchInd[j]].y;
                    cz += localMap_corner_ds->points[pointSearchInd[j]].z;
                }

                cx *= 0.2;
                cy *= 0.2;
                cz *= 0.2;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax =
                            localMap_corner_ds->points[pointSearchInd[j]].x - cx;
                    float ay =
                            localMap_corner_ds->points[pointSearchInd[j]].y - cy;
                    float az =
                            localMap_corner_ds->points[pointSearchInd[j]].z - cz;

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

                    float a012 =
                            sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                                 ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                                 ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                                 ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                                 ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                                 ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                                     (z1 - z2) * (z1 - z2));

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

//                    点到直线的距离
                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.15) {
//                        resize之后，指针在队尾，所以push_back的位置在末尾，而不是初始位置
//                          并且push_back效率要远低于提前分配内存之后按照索引赋值，push_back会检查内存是否足够，以及申请内存
//                        std::cout<<"laserCloudOriCornerVec size is : "<<laserCloudOriCornerVec.size() << std::endl;
//                        std::cout<<"pointOri is : "<<pointOri  << " coeff is : "<<coeff<<std::endl;
//                        laserCloudOriCornerVec.push_back(pointOri); //current_corner_ds
//                        coeffSelCornerVec.push_back(coeff);
//                        laserCloudOriCornerFlag.push_back(true);

                        laserCloudOriCornerVec[i] = pointOri;// 原始点云
                        coeffSelCornerVec[i] = coeff; // 鲁棒距离，鲁棒向量
//                        std::cout<<"laserCloudOriCornerVec i is : "<<i << std::endl;
                        laserCloudOriCornerFlag[i] = true; // 对应找到的线
                    }
                }
            }
        }
    }

    void surfOptimization() {
        switch (current_frame) {
            case WORLD:
//                EZLOG(INFO)<<"current_T_m_l already in world_surf";
                updatePointAssociateToMap();
                break;
            case LIDAR:
//                EZLOG(INFO)<<"change current_T_m_l to world_surf";
                transforPoint2World();
                break;
            default:
                EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                break;
        }

// long startTime= System.currentTimeMillis();
//#pragma omp parallel for num_threads(numberOfCores)
//        current_surf_ds_num
        for (int i = 0; i < current_surf_ds_num; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = current_surf_ds->points[i];
            switch (current_frame) {
                case WORLD:
//                    EZLOG(INFO)<<"current_T_m_l already in world_surf";
                    pointAssociateToWorld_noTrans(&pointOri, &pointSel);
                    break;
                case LIDAR:
//                    EZLOG(INFO)<<"change current_T_m_l to world_surf";
                    pointAssociateToWorld_withTrans(&pointOri, &pointSel);
                    break;
                default:
                    EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                    break;
            }
            kdtree_localMap_surf->nearestKSearch(pointSel, 5, pointSearchInd,
                                                 pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = localMap_surf_ds->points[pointSearchInd[j]].x;
                    matA0(j, 1) = localMap_surf_ds->points[pointSearchInd[j]].y;
                    matA0(j, 2) = localMap_surf_ds->points[pointSearchInd[j]].z;
                }

//                QR分解，求解平面系数
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
                    if (fabs(pa * localMap_surf_ds->points[pointSearchInd[j]].x +
                             pb * localMap_surf_ds->points[pointSearchInd[j]].y +
                             pc * localMap_surf_ds->points[pointSearchInd[j]].z +
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

                    if (s > 0.15) {
//                        laserCloudOriSurfVec.push_back(pointOri);
//                        coeffSelSurfVec.push_back(coeff);
//                        laserCloudOriSurfFlag.push_back(true);
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
//                        std::cout<<"laserCloudOriSurfVec i is : "<<i << std::endl;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs() {
        // combine corner coeffs
        std::cout<<"combineOptimizationCoeffs()!" << std::endl;
        for (int i = 0; i < current_corner_ds_num; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < current_surf_ds_num; ++i) {
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
        //由于LOAM里雷达的特殊坐标系 所以这里也转了一次
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll
        std::cout<<"LMOptimization()!" << std::endl;
        // lidar -> camera
        float srx = sin(current_T_m_l[1]);
        float crx = cos(current_T_m_l[1]);
        float sry = sin(current_T_m_l[2]);
        float cry = cos(current_T_m_l[2]);
        float srz = sin(current_T_m_l[0]);
        float crz = cos(current_T_m_l[0]);

        int laserCloudSelNum = laserCloudOri->size();

        std::cout<<"laserCloudSelNum is : " <<laserCloudSelNum <<std::endl;
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
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
                         srx * sry * pointOri.z) *
                        coeff.x +
                        (-srx * srz * pointOri.x - crz * srx * pointOri.y -
                         crx * pointOri.z) *
                        coeff.y +
                        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
                         cry * srx * pointOri.z) *
                        coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
                         (sry * srz + cry * crz * srx) * pointOri.y +
                         crx * cry * pointOri.z) *
                        coeff.x +
                        ((-cry * crz - srx * sry * srz) * pointOri.x +
                         (cry * srz - crz * srx * sry) * pointOri.y -
                         crx * sry * pointOri.z) *
                        coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                         (-cry * crz - srx * sry * srz) * pointOri.y) *
                        coeff.x +
                        (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                        ((sry * srz + cry * crz * srx) * pointOri.x +
                         (crz * sry - cry * srx * srz) * pointOri.y) *
                        coeff.z;
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

        float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.10 && deltaT < 0.10) {
            return true;  // converged
        }
        return false;  // keep optimizing
    }

    void scan2MapOptimization() {
        if (Keyframe_Poses3D->points.empty()) return;

////        当前帧特征点
//        std::cout << "current_corner_ds_num: " << current_corner_ds_num
//                  << " current_surf_ds_num: " << current_surf_ds_num << std::endl;

        if (current_corner_ds_num > MappingConfig::edgeFeatureMinValidNum &&
            current_surf_ds_num > MappingConfig::surfFeatureMinValidNum) {
//            add by prior map
            kdtree_localMap_corner->setInputCloud(localMap_corner_ds);
            kdtree_localMap_surf->setInputCloud(localMap_surf_ds);
            TicToc optimize_time;
            for (int iterCount = 0; iterCount < 30; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true) {
                    std::cout <<"iterCount: " << iterCount<<std::endl;
                    std::cout << "optimize_time in ms: "<<optimize_time.toc() <<std::endl;
                    break;
                }
            }

            transformUpdate();
        } else {
            EZLOG(INFO) <<"Not enough features! : "
                        <<current_corner_ds_num, current_surf_ds_num;
        }
    }

//    IMU 初值
    void transformUpdate() {
        std::cout <<"transformUpdate " <<std::endl;
        current_T_m_l[0] =
                constraintTransformation(current_T_m_l[0], MappingConfig::rotation_tollerance);
        current_T_m_l[1] =
                constraintTransformation(current_T_m_l[1], MappingConfig::rotation_tollerance);
        current_T_m_l[5] =
                constraintTransformation(current_T_m_l[5], MappingConfig::z_tollerance);

    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool saveFrame() {
        if (Keyframe_Poses3D->points.empty()) return true;

        Eigen::Affine3f transStart = pclPointToAffine3f(Keyframe_Poses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(
                current_T_m_l[3], current_T_m_l[4], current_T_m_l[5],
                current_T_m_l[0], current_T_m_l[1], current_T_m_l[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < MappingConfig::surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < MappingConfig::surroundingkeyframeAddingDistThreshold)
            return false;

        std::cout << "distance gap: " << sqrt(x * x + y * y) << std::endl;

        return true;
    }

    void addOdomFactor() {
        if (Keyframe_Poses3D->points.empty()) {
            gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                    .finished());  // rad*rad, meter*meter

            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(current_T_m_l),
                                              priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(current_T_m_l));
        } else {
            gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom =
                    pclPointTogtsamPose3(Keyframe_Poses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(current_T_m_l);

            mtxGraph.lock();
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    Keyframe_Poses3D->size() - 1, Keyframe_Poses3D->size(),
                    poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(Keyframe_Poses3D->size(), poseTo);
            mtxGraph.unlock();
        }
    }

    void addGPSFactor() {
//        if (gpsQueue.empty()) return;

        // wait for system initialized and settles down
        if (Keyframe_Poses3D->points.empty() || Keyframe_Poses3D->points.size() == 1)
            return;
        //    else {
        //      if (pointDistance(Keyframe_Poses3D->front(), Keyframe_Poses3D->back())
        //      < 5.0)
        //        return;
        //    }

        // pose covariance small, no need to correct
        //        if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4,
        //        4) < poseCovThreshold)
        //            return;

        // last gps position
        static PointType lastGPSPoint;
        // nav_msgs::Odometry thisGPS;
//        if (syncGPS(gpsQueue, thisGPS, timeLaserInfoCur, 1.0 / SensorConfig::gpsFrequence)) {
//            // GPS too noisy, skip
//            float noise_x = thisGPS.pose.covariance[0];
//            float noise_y = thisGPS.pose.covariance[7];
//            float noise_z = thisGPS.pose.covariance[14];
//
//            // make sure the gps data is stable encough
//            if (abs(noise_x) > SensorConfig::gpsCovThreshold || abs(noise_y) > SensorConfig::gpsCovThreshold)
//                return;
//
////            float gps_x = thisGPS.pose.pose.position.x;
////            float gps_y = thisGPS.pose.pose.position.y;
////            float gps_z = thisGPS.pose.pose.position.z;
//            double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
//            Eigen::Vector3d LLA(thisGPS.pose.covariance[1], thisGPS.pose.covariance[2], thisGPS.pose.covariance[3]);
//            geo_converter.Forward(LLA[0], LLA[1], LLA[2], gps_x, gps_y, gps_z);
//
//            if (!SensorConfig::useGpsElevation) {
//                gps_z = current_T_m_l[5];
//                noise_z = 0.01;
//            }
//            // GPS not properly initialized (0,0,0)
//            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) return;
//
//            // Add GPS every a few meters
//            PointType curGPSPoint;
//            curGPSPoint.x = gps_x;
//            curGPSPoint.y = gps_y;
//            curGPSPoint.z = gps_z;
//            if (pointDistance(curGPSPoint, lastGPSPoint) < SensorConfig::gpsDistance)
//                return;
//            else
//                lastGPSPoint = curGPSPoint;
//
//            if (SensorConfig::debugGps) {
//                ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
//                ROS_INFO("curr gps cov: %f, %f , %f", thisGPS.pose.covariance[0],
//                         thisGPS.pose.covariance[7], thisGPS.pose.covariance[14]);
//            }
//
//            gtsam::Vector Vector3(3);
//            Vector3 << noise_x, noise_y, noise_z;
//            // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
//            gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
//                    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector3);
//            gtsam::GPSFactor gps_factor(Keyframe_Poses3D->size(),
//                                        gtsam::Point3(gps_x, gps_y, gps_z),
//                                        gps_noise);
//            keyframeGPSfactor.push_back(gps_factor);
//            cloudKeyGPSPoses3D->points.push_back(curGPSPoint);
//
//            // only a trick!
//            // we need to accumulate some accurate gps points to initialize the
//            // transform between gps coordinate system and LIO coordinate system and
//            // then we can add gps points one by one into the pose graph or the whole
//            // pose graph will crashed if giving some respectively bad gps points at
//            // first.
//            if (keyframeGPSfactor.size() < 20) {
//                ROS_INFO("Accumulated gps factor: %d", keyframeGPSfactor.size());
//                return;
//            }
//
//            if (!gpsTransfromInit) {
//                ROS_INFO("Initialize GNSS transform!");
//                for (int i = 0; i < keyframeGPSfactor.size(); ++i) {
//                    gtsam::GPSFactor gpsFactor = keyframeGPSfactor.at(i);
//                    gtSAMgraph.add(gpsFactor);
//                    gpsIndexContainer[gpsFactor.key()] = i;
//                }
//                gpsTransfromInit = true;
//            } else {
//                // After the coordinate systems are aligned, in theory, the GPS z and
//                // the z estimated by the current LIO system should not be too
//                // different. Otherwise, there is a problem with the quality of the
//                // secondary GPS point.
//                //                if (abs(gps_z - Keyframe_Poses3D->back().z) > 10.0) {
//                //                    // ROS_WARN("Too large GNSS z noise %f", noise_z);
//                //                    gtsam::Vector Vector3(3);
//                //                    Vector3 << max(noise_x, 10000.0f), max(noise_y,
//                //                    10000.0f), max(noise_z, 100000.0f);
//                //                    // gps_noise =
//                //                    noiseModel::Diagonal::Variances(Vector3);
//                //                    // gps_factor =
//                //                    gtsam::GPSFactor(Keyframe_Poses3D->size(),
//                //                    gtsam::Point3(gps_x, gps_y, gps_z),
//                //                    // gps_noise);
//                //                }
//                // add loop constriant
//                mtxGraph.lock();
//                gtSAMgraph.add(gps_factor);
//                mtxGraph.unlock();
//                gpsIndexContainer[Keyframe_Poses3D->size()] =
//                        cloudKeyGPSPoses3D->size() - 1;
//            }
//        }
    }
    void addGNSSBetweenFactor()
    {
        if (Keyframe_Poses3D->points.size() < 3) {
            EZLOG(INFO) << "GNSS init start from frame 3!";
            last_gnss_poses = gtsam::Pose3(gtsam::Rot3(q_w_cur_matrix), gtsam::Point3(t_w_cur));
            Keyframe_Poses3D_last_num = Keyframe_Poses3D->points.size();
            return;
        }
        gtsam::Pose3 current_gnss_poses;
        frame_cnt++;

        if (frame_cnt < SensorConfig::gtsamGNSSBetweenFactorDistance) {
//            EZLOG(INFO) << "control GNSS factor frequency!: "<<float(frame_cnt*1.0/5)*100 <<" %";
            // void ,control frequency
        } else {
//            EZLOG(INFO) << "ADD GNSS BETWEEN FACTOR!!!";
//            EZLOG(INFO) << "last_gnss_poses:"<<last_gnss_poses;
            current_gnss_poses = gtsam::Pose3(gtsam::Rot3(q_w_cur_matrix), gtsam::Point3(t_w_cur));
//            EZLOG(INFO) << "current_gnss_poses:"<<current_gnss_poses;
//            EZLOG(INFO) << "last_gnss_poses No:"<<Keyframe_Poses3D_last_num;
//            EZLOG(INFO) << "current_gnss_poses No:"<<Keyframe_Poses3D->points.size();
            frame_cnt = 0;
            gtsam::noiseModel::Diagonal::shared_ptr GNSS_noise =
                    gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 gnss_pose_From = last_gnss_poses;
            gtsam::Pose3 gnss_pose_To = current_gnss_poses;

            mtxGraph.lock();
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                    Keyframe_Poses3D_last_num, Keyframe_Poses3D->size(),
                    gnss_pose_From.between(gnss_pose_To), GNSS_noise));
            mtxGraph.unlock();
            last_gnss_poses = current_gnss_poses;
            Keyframe_Poses3D_last_num = Keyframe_Poses3D->points.size();
        }

    }
    void saveKeyFramesAndFactor() {
        if (saveFrame() == false) return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGNSSBetweenFactor();
//        if (SensorConfig::useGPS) addGPSFactor();

        cout << "****************************************************" << endl;
        gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
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
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity =
                Keyframe_Poses3D->size();  // this can be used as index
        Keyframe_Poses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        Keyframe_Poses6D->push_back(thisPose6D);

        // save updated transform
        current_T_m_l[0] = latestEstimate.rotation().roll();
        current_T_m_l[1] = latestEstimate.rotation().pitch();
        current_T_m_l[2] = latestEstimate.rotation().yaw();
        current_T_m_l[3] = latestEstimate.translation().x();
        current_T_m_l[4] = latestEstimate.translation().y();
        current_T_m_l[5] = latestEstimate.translation().z();

    }

    void correctPoses() {
        if (Keyframe_Poses3D->points.empty()) return;

    }

    void publishOdometry() {
        // Publish odometry in world
        OdometryType current_lidar_pose_world;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(current_T_m_l);//T_ml
        Eigen::Matrix4d map_2_world = SensorConfig::T_L_B; //T_wm
        Eigen::Matrix4d Lidar_odom_world = map_2_world * Lidarodom_2_map.matrix().cast<double>();// T_wl = T_wm * T_ml

        current_lidar_pose_world.frame = "map";
        current_lidar_pose_world.timestamp = timeLaserInfoCur;
        switch (current_frame) {
            default:
                EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                break;
            case WORLD:
                current_lidar_pose_world.pose.pose = Lidarodom_2_map.matrix().cast<double>();
                break;
            case LIDAR:
                current_lidar_pose_world.pose.pose = Lidar_odom_world.matrix().cast<double>();
                break;

        }
        Function_AddOdometryTypeToIMUPreintegration(current_lidar_pose_world);
        pubsub->PublishOdometry(topic_lidar_odometry, current_lidar_pose_world);
    }

    void DoWork(){
        allocateMemory();
        while(1){
            if(deque_cloud.size()!=0){
                CloudFeature cur_ft;
                cloud_mutex.lock();
                cur_ft = deque_cloud.front();
                EZLOG(INFO)<<"cur_ft.frame_id:  "<<cur_ft.frame_id;
                deque_cloud.pop_front();
                cloud_mutex.unlock();

                //just do something
//                timeLaserInfoStamp = msgIn->header.stamp;
                EZLOG(INFO)<<"cur_ft.surfaceCloud size: "<< cur_ft.surfaceCloud->points.size();
                EZLOG(INFO)<<"cur_ft.cornerCloud size: "<<  cur_ft.cornerCloud->points.size();
                timeLaserInfoCur = cur_ft.timestamp;
                current_surf = cur_ft.surfaceCloud;
                current_corner = cur_ft.cornerCloud;
//                current_corner = &cur_corner;
//                current_surf = &cur_surf;
                t_w_cur = cur_ft.pose.GetXYZ();
                q_w_cur = cur_ft.pose.GetQ();
                q_w_cur.normalize();
                q_w_cur_matrix = q_w_cur.toRotationMatrix();
                current_frameID = cur_ft.frame_id;
                // 提取欧拉角（Z-Y-X旋转顺序）q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;
                q_w_cur_pitch = asin(-q_w_cur_matrix(2, 0)); // 计算pitch
                if (cos(q_w_cur_pitch) != 0) {
                    q_w_cur_roll = atan2(q_w_cur_matrix(2, 1), q_w_cur_matrix(2, 2)); // 计算roll
                    q_w_cur_yaw = atan2(q_w_cur_matrix(1, 0), q_w_cur_matrix(0, 0));  // 计算yaw
                } else {
                    q_w_cur_roll = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
                    q_w_cur_yaw = atan2(-q_w_cur_matrix(0, 1), q_w_cur_matrix(1, 1)); // 计算yaw
                }
                std::lock_guard<std::mutex> lock(mtx);
                static double timeLastProcessing = -1;
                if (timeLaserInfoCur - timeLastProcessing >= MappingConfig::mappingProcessInterval) {
                    timeLastProcessing = timeLaserInfoCur;

                    updateInitialGuess();
                    EZLOG(INFO)<<"------------updateInitialGuess finish---------------" <<std::endl;
                    if (systemInitialized) {
                        if(flagLoadMap){
                            pub_CornerAndSurfFromMap();

                        }
                        TicToc t_2;
                        extractFromPriorMap();
                        EZLOG(INFO)<<"extractFromPriorMap() time : "<<t_2.toc();
                        TicToc t_3;
                        downsampleCurrentScan();
                        EZLOG(INFO)<<"downsampleCurrentScan() time : "<<t_3.toc();
                        TicToc t_4;
                        scan2MapOptimization();
                        EZLOG(INFO)<<"scan2MapOptimization() time : "<<t_4.toc();
                        TicToc t_5;
                        saveKeyFramesAndFactor();
                        EZLOG(INFO)<<"saveKeyFramesAndFactor() time : "<<t_5.toc();
                        TicToc t_6;
                        publishOdometry();
                        EZLOG(INFO)<<"publishOdometry() time : "<<t_6.toc();

                    }
                }


            }
        }
    }
    /**
* add by sy for load Prior map Corner
* @param mapFile
*/
    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        pubsub->addPublisher(topic_priorMap_corner, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_priorMap_surf, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_lidar_odometry, DataType::ODOMETRY, 10);
        do_work_thread = new std::thread(&LOCMapping::DoWork, this);
    }
};
#endif //SEU_LIDARLOC_OPT_LOPC_H
