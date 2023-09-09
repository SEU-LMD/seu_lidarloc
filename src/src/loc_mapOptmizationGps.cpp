#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>  // gtsam
#include <std_srvs/Empty.h>
#include "GeoGraphicLibInclude/Geocentric.hpp"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "GeoGraphicLibInclude/Geoid.hpp"
#include <csignal>

#include "dataSaver.h"
#include "cloud_info.h"

#include "utility.h"
#include "timer.h"
#include "MapSaver.h"
#include "utm/utm_convert.h"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G;  // GPS pose
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is
 * time stamp)
 */
struct PointXYZIRPYT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZIRPYT,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                         time))

typedef PointXYZIRPYT PointTypePose;

class Loc_mapOptimization {
public:
    int flagLoadMap = 1;
    ros::NodeHandle nh;
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate; // 所有关键帧位姿的优化结果
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher publidar_odom_World;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubGPSOdometry;

    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubSurfFromMap;
    ros::Publisher pubCornerFromMap;
    sensor_msgs::PointCloud2 pcl_SurfMapMsg;
    sensor_msgs::PointCloud2 pcl_CornerMapMsg;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;

    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam_6axis::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> Keyframe_corner_ds;
    vector<pcl::PointCloud<PointType>::Ptr> Keyframe_surf_ds;

    //    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    pcl::PointCloud<PointType>::Ptr Keyframe_Poses3D;      //历史关键帧 位置
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr Keyframe_Poses6D;  //历史关键帧 位姿
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr current_corner;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_surf;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_corner_ds;  // downsampled corner feature set from
    pcl::PointCloud<PointType>::Ptr current_surf_ds;  // downsampled surf feature set from

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> localMap_corner_and_surf;
    pcl::PointCloud<PointType>::Ptr localMap_corner;
    pcl::PointCloud<PointType>::Ptr localMap_surf;
    pcl::PointCloud<PointType>::Ptr priorMap_corner_ds;
    pcl::PointCloud<PointType>::Ptr priorMap_surf_ds;
    pcl::PointCloud<PointType>::Ptr localMap_corner_ds;
    pcl::PointCloud<PointType>::Ptr localMap_surf_ds;
    pcl::PointCloud<PointType>::Ptr priorMap_corner;// 先验地图角点
    pcl::PointCloud<PointType>::Ptr priorMap_surf; // 先验地图面点

    pcl::PointCloud<PointType>::Ptr Surrounding_surf; //局部 匹配点云 面点
    pcl::PointCloud<PointType>::Ptr Surrounding_corner;//局部 匹配点云 角点

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_priorMap_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_priorMap_corner;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_corner;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_Keyframe_Poses3D;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> near_keyframe_poses_ds;        // for surrounding key poses of

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float current_T_l_m[6]; // roll pitch yaw x y z

    std::mutex mtx;
    std::mutex mtxGpsInfo;
    std::mutex mtxGraph;

    Eigen::Vector3d originLLA;
    bool systemInitialized = false;
    bool gpsTransfromInit = false;
    GeographicLib::LocalCartesian geo_converter;

    bool isDegenerate = false;
    cv::Mat matP;

    int localMap_corner_ds_num = 0;
    int localMap_surf_ds_num = 0;
    int current_corner_ds_num = 0;
    int current_surf_ds_num = 0;
    int current_lidar_frameID = 0;
    int last_lidar_frameID = 0;


    map<int, int> gpsIndexContainer;   // from new to old
    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    Eigen::Vector3d t_w_cur;
    Eigen::Quaterniond q_w_cur;
    double q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;

    /**
     * 发布：
     * 历史关键帧点云，局部关键帧特征点云，
     * 激光里程计 mapping/odometry(可视化)，mapping/odometry_incremental(IMU对roll pitch 和z  加权平均做了优化)
     * 地图保存服务
     * 回环检测中的闭环关键帧的局部地图，闭环关键帧特征点云
     * 局部map降采样，历史帧角点面点的降采样，
     * 订阅：
     * 当前帧激光点云，当前GPS，回环检测
     */
    Loc_mapOptimization() {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        priorMap_corner.reset(new pcl::PointCloud<PointType>); // 先验地图角点
        priorMap_surf.reset(new pcl::PointCloud<PointType>);    // 先验地图面点
        Surrounding_surf.reset(new pcl::PointCloud<PointType>);
        Surrounding_corner.reset(new pcl::PointCloud<PointType>);
        kdtree_priorMap_surf.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_priorMap_corner.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_localMap_surf.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_localMap_corner.reset(new pcl::KdTreeFLANN<PointType>);

        allocateMemory();

        if (SensorConfig::useGPS) {
            pubGPSOdometry = nh.advertise<sensor_msgs::NavSatFix>("lio_sam_6axis/mapping/odometry_gps", 1);
        }

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/trajectory", 1);
        pubLaserOdometryGlobal =
                nh.advertise<nav_msgs::Odometry>("lio_sam_6axis/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>(
                "lio_sam_6axis/mapping/odometry_incremental", 1);
        publidar_odom_World =
                nh.advertise<nav_msgs::Odometry>("lidar_odometry_world", 1);

        subCloud = nh.subscribe<lio_sam_6axis::cloud_info>(
                "lio_sam_6axis/feature/cloud_info", 1,
                &Loc_mapOptimization::laserCloudInfoHandler, this,
                ros::TransportHints().tcpNoDelay());

//        GPS factor
        subGPS = nh.subscribe<nav_msgs::Odometry>(
                "gps_odom", 200, &Loc_mapOptimization::gpsHandler, this,
                ros::TransportHints().tcpNoDelay());

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/map_local", 1);

        pubSurfFromMap = nh.advertise<sensor_msgs::PointCloud2>(
                "surf_fromMap", 1);
        pubCornerFromMap = nh.advertise<sensor_msgs::PointCloud2>(
                "Corner_fromMap", 1);

        pubSLAMInfo = nh.advertise<lio_sam_6axis::cloud_info>(
                "lio_sam_6axis/mapping/slam_info", 1);

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
        // scan-to-map optimization
    }

    void allocateMemory() {
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

//        laserCloudOriCornerVec.reserve(N_SCAN * Horizon_SCAN);
//        coeffSelCornerVec.reserve(N_SCAN * Horizon_SCAN);
//        laserCloudOriCornerFlag.reserve(N_SCAN * Horizon_SCAN);
//        laserCloudOriSurfVec.reserve(N_SCAN * Horizon_SCAN);
//        coeffSelSurfVec.reserve(N_SCAN * Horizon_SCAN);
//        laserCloudOriSurfFlag.reserve(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(),
                  false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(),
                  false);

        localMap_corner.reset(new pcl::PointCloud<PointType>());
        localMap_surf.reset(new pcl::PointCloud<PointType>());
        localMap_corner_ds.reset(new pcl::PointCloud<PointType>());
        localMap_surf_ds.reset(new pcl::PointCloud<PointType>());

        for (int i = 0; i < 6; ++i) {
            current_T_l_m[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        last_lidar_frameID = 0;
        current_lidar_frameID = 0;
    }


    /**
 * add by sy for load Prior map Corner
 * @param mapFile
 */
    void Load_PriorMap_Corner(std::string mapFile,pcl::PointCloud<PointType>::Ptr _priorMap_corner)
    {

        if(pcl::io::loadPCDFile<pcl::PointXYZI>(mapFile,*_priorMap_corner) == -1)
        {
            std::cout << "Could not read Map!"<<std::endl;
            return ;
        }
        pcl::transformPointCloud(*_priorMap_corner, *_priorMap_corner, SensorConfig::T_L_B);
        kdtree_priorMap_corner->setInputCloud(_priorMap_corner);//kdtreeCornerFromMap
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
        pcl::transformPointCloud(*_priorMap_surf, *_priorMap_surf, SensorConfig::T_L_B);
        kdtree_priorMap_surf->setInputCloud(_priorMap_surf);//kdtreeSurfFromMap
        std::cout << "Read Surf Map!"<<std::endl;
    }

    void pub_CornerAndSurfFromMap()
    {
        if (pubSurfFromMap.getNumSubscribers() != 0)
        {
            pcl::PointCloud<pcl::PointXYZI> pub_cloud;
            pub_cloud = *priorMap_surf;
            pcl::toROSMsg(pub_cloud, pcl_SurfMapMsg);
            pcl_SurfMapMsg.header.frame_id = "map";
            pcl_SurfMapMsg.header.stamp = ros::Time::now();
            pubSurfFromMap.publish(pcl_SurfMapMsg);
            std::cout << "Pub Surf Map!"<<std::endl;
        }

        if (pubCornerFromMap.getNumSubscribers() != 0)
        {
            pcl::PointCloud<pcl::PointXYZI> pub_cloud;
            pub_cloud = *priorMap_corner;
            pcl::toROSMsg(pub_cloud, pcl_CornerMapMsg);
            pcl_CornerMapMsg.header.frame_id = "map";
            pcl_CornerMapMsg.header.stamp = ros::Time::now();
            pubCornerFromMap.publish(pcl_CornerMapMsg);
            std::cout << "Pub Corner Map!"<<std::endl;
        }

    }

/**
 * 输入：IMU、激光原始点云、激光里程计数据，GPS
 * 输出： 因子节点位姿更新、
 * 当前帧位姿初始化
 * 订阅当前激光原始点云，特征提取，加入局部地图 ————》特征提取
 * 对当前角点、面点降采样
 * scan2map优化当前位姿 (IMU原始RPY数据，会对结果加权平均，并约束z)
 * 设置当前帧为关键帧，并因子图优化（激光、GPS、闭环，IMU积分结果作为当前帧初值） ————》 因子删减
 *
 * @param msgIn
 */
    void laserCloudInfoHandler(const lio_sam_6axis::cloud_infoConstPtr &msgIn) {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
        // extract info and feature cloud
        cloudInfo = *msgIn;
//        current_corner 当前帧瞬时点云
        pcl::fromROSMsg(msgIn->cloud_corner, *current_corner);
        pcl::fromROSMsg(msgIn->cloud_surface, *current_surf);

        t_w_cur[0]= msgIn->T_w_l_curlidar.pose.pose.position.x;
        t_w_cur[1]= msgIn->T_w_l_curlidar.pose.pose.position.y;
        t_w_cur[2]= msgIn->T_w_l_curlidar.pose.pose.position.z;
//        EZLOG(INFO)<<"t_w_cur[0]"<<t_w_cur[0]<<std::endl;
//        EZLOG(INFO)<<"t_w_cur[1]"<<t_w_cur[1]<<std::endl;
//        EZLOG(INFO)<<"t_w_cur[2]"<<t_w_cur[2]<<std::endl;

        q_w_cur.x()= msgIn->T_w_l_curlidar.pose.pose.orientation.x;
        q_w_cur.y() = msgIn->T_w_l_curlidar.pose.pose.orientation.y;
        q_w_cur.z()= msgIn->T_w_l_curlidar.pose.pose.orientation.z;
        q_w_cur.w()= msgIn->T_w_l_curlidar.pose.pose.orientation.w;
        q_w_cur.normalize();

        Eigen::Matrix3d q_w_cur_matrix = q_w_cur.toRotationMatrix();
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
//        激光点的更新频率，控制处理频率 mappingProcessInterval= 6.667 Hz , 0.15s

        if (timeLaserInfoCur - timeLastProcessing >= MappingConfig::mappingProcessInterval) {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();
            std::cout<<"updateInitialGuess finish" <<std::endl;
            if (systemInitialized) {
                if(flagLoadMap){
//                    从晓强接收
                    Load_PriorMap_Surf("/home/sy/SEU_WS/backup/seu_lidarloc_8.25/src/globalmap_lidar_feature_surf.pcd",priorMap_surf);
                    Load_PriorMap_Corner("/home/sy/SEU_WS/backup/seu_lidarloc_8.25/src/globalmap_lidar_feature_corner.pcd",priorMap_corner);

//                    PointType init_point_3D; //can be changed in gnss map
//                    init_point_3D.x = 0.;
//                    init_point_3D.y = 0.;
//                    init_point_3D.z = 0.;
//                    init_point_3D.intensity = 0;
//                    PointTypePose init_point_6D; //can be changed in gnss map
//                    init_point_6D.x = 0.;
//                    init_point_6D.y = 0.;
//                    init_point_6D.z = 0.;
//                    init_point_6D.roll = 0.;
//                    init_point_6D.pitch = 0.;
//                    init_point_6D.yaw = 0.;
//                    init_point_6D.time = timeLaserInfoCur;
//                    Keyframe_Poses3D->push_back(init_point_3D);
//                    Keyframe_Poses6D->push_back(init_point_6D);

                    flagLoadMap = 0;
                }
                pub_CornerAndSurfFromMap();

                extractFromPriorMap();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                publishOdometry();

                publishFrames();


            }
        }
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr &gpsMsg) {
        if (SensorConfig::useGPS) {
            mtxGpsInfo.lock();
            gpsQueue.push_back(*gpsMsg);
            mtxGpsInfo.unlock();
        }
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

    bool syncGPS(std::deque<nav_msgs::Odometry> &gpsBuf,
                 nav_msgs::Odometry &aligedGps, double timestamp,
                 double eps_cam) {
        bool hasGPS = false;
        while (!gpsQueue.empty()) {
            mtxGpsInfo.lock();
            if (gpsQueue.front().header.stamp.toSec() < timestamp - eps_cam) {
                // message too old
                gpsQueue.pop_front();
                mtxGpsInfo.unlock();
            } else if (gpsQueue.front().header.stamp.toSec() > timestamp + eps_cam) {
                // message too new
                mtxGpsInfo.unlock();
                break;
            } else {
                hasGPS = true;
                aligedGps = gpsQueue.front();
                gpsQueue.pop_front();
//                if (debugGps)
//                    ROS_INFO("GPS time offset %f ",
//                             aligedGps.header.stamp.toSec() - timestamp);
                mtxGpsInfo.unlock();
            }
        }

        if (hasGPS)
            return true;
        else
            return false;
    }

    void visualizeGlobalMapThread() {
        ros::Rate rate(0.2);
        while (ros::ok()) {
            rate.sleep();
//            publishGlobalMap();
        }
    }

    void updateInitialGuess() {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(current_T_l_m);
        static Eigen::Affine3f lastImuTransformation;

        // initialization the first frame
//        根据gps信息，加载地图
        if (Keyframe_Poses3D->points.empty()) {
            systemInitialized = false;
            if (SensorConfig::useGPS) {
                ROS_INFO("GPS use to init pose");
                /** when you align gnss and lidar timestamp, make sure (1.0/gpsFrequence) is small encougn
                 *  no need to care about the real gnss frquency. time alignment fail will cause
                 *  "[ERROR] [1689196991.604771416]: sysyem need to be initialized"
                 * */
                nav_msgs::Odometry alignedGPS;
//                有做gps时间同步，并控制gps频率
                if (syncGPS(gpsQueue, alignedGPS, timeLaserInfoCur,
                            1.0 / SensorConfig::gpsFrequence)) {
                    /** we store the origin wgs84 coordinate points in covariance[1]-[3] */
                    originLLA.setIdentity();
                    originLLA = Eigen::Vector3d(alignedGPS.pose.covariance[1],
                                                alignedGPS.pose.covariance[2],
                                                alignedGPS.pose.covariance[3]);
                    /** set your map origin points */
                    geo_converter.Reset(originLLA[0], originLLA[1], originLLA[2]);
                    // WGS84->ENU, must be (0,0,0)
                    Eigen::Vector3d enu;
                    geo_converter.Forward(originLLA[0], originLLA[1], originLLA[2], enu[0], enu[1], enu[2]);

                    if (SensorConfig::debugGps) {
                        double roll, pitch, yaw;
                        tf::Matrix3x3(tf::Quaternion(alignedGPS.pose.pose.orientation.x,
                                                     alignedGPS.pose.pose.orientation.y,
                                                     alignedGPS.pose.pose.orientation.z,
                                                     alignedGPS.pose.pose.orientation.w))
                                .getRPY(roll, pitch, yaw);
                        std::cout << "initial gps yaw: " << yaw << std::endl;
                        std::cout << "GPS Position: " << enu.transpose() << std::endl;
                        std::cout << "GPS LLA: " << originLLA.transpose() << std::endl;
                    }

                    /** add the first factor, we need this origin GPS point for prior map based localization,
                     * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                    PointType gnssPoint;
                    gnssPoint.x = enu[0],
                    gnssPoint.y = enu[1],
                    gnssPoint.z = enu[2];
                    float noise_x = alignedGPS.pose.covariance[0];
                    float noise_y = alignedGPS.pose.covariance[7];
                    float noise_z = alignedGPS.pose.covariance[14];

                    /** if we get reliable origin point, we adjust the weight of this gps factor */
                    if (!SensorConfig::updateOrigin) {
                        noise_x *= 1e-4;
                        noise_y *= 1e-4;
                        noise_z *= 1e-4;
                    }
                    gtsam::Vector Vector3(3);
                    Vector3 << noise_x, noise_y, noise_z;
                    noiseModel::Diagonal::shared_ptr gps_noise =
                            noiseModel::Diagonal::Variances(Vector3);
                    gtsam::GPSFactor gps_factor(
                            0, gtsam::Point3(gnssPoint.x, gnssPoint.y, gnssPoint.z),
                            gps_noise);
                    keyframeGPSfactor.push_back(gps_factor);
                    cloudKeyGPSPoses3D->points.push_back(gnssPoint);

//                    cloudInfo.T_w_l_curlidar.header.seq;id
//                    cloudInfo.T_w_l_curlidar.header.stamp;time
//                    cloudInfo.T_w_l_curlidar.pose.pose.position
//                    cloudInfo.T_w_l_curlidar.pose.pose.orientation

                    current_T_l_m[0] = cloudInfo.imuRollInit;
                    current_T_l_m[1] = cloudInfo.imuPitchInit;
                    current_T_l_m[2] = cloudInfo.imuYawInit;
                    if (!SensorConfig::useImuHeadingInitialization) current_T_l_m[2] = 0;
                    lastImuTransformation = pcl::getTransformation(
                            0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit,
                            cloudInfo.imuYawInit);
                    systemInitialized = true;
                    ROS_WARN("GPS init success");
                }
            } else {
//                std::cout << "cloudInfo.imuRollInit: " << cloudInfo.imuRollInit
//                            << " , imuPitchInit: " << cloudInfo.imuPitchInit
//                        << " , imuYawInit: " << cloudInfo.imuYawInit<<std::endl;
//              cloudInfo 回调里直接收
                current_T_l_m[0] = 0;
                current_T_l_m[1] = 0;
                current_T_l_m[2] = 0;

                if (!SensorConfig::useImuHeadingInitialization) current_T_l_m[2] = 0;

                lastImuTransformation = pcl::getTransformation(
                        0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit,
                        cloudInfo.imuYawInit);

                systemInitialized = true;
                return;
            }
        }

        if (!systemInitialized) {
            ROS_ERROR("sysyem need to be initialized");
            return;
        }

        // if not the first frame
        // use imu pre-integration estimation for pose guess
//----------------------Use Gnss for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;

        Eigen::Affine3f transBack = pcl::getTransformation(
                t_w_cur[0], t_w_cur[1], t_w_cur[2],q_w_cur_roll,
                q_w_cur_pitch , q_w_cur_yaw);

        if (lastImuPreTransAvailable == false) {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre =
                    lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(current_T_l_m);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(
                    transFinal, current_T_l_m[3], current_T_l_m[4],
                    current_T_l_m[5], current_T_l_m[0],
                    current_T_l_m[1], current_T_l_m[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(
                    0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit,
                    cloudInfo.imuYawInit);  // save imu before return;
            return;
        }


    }

/**
 * extract init points from map
 */
    void extractFromPriorMap(){
        std::cout <<"Keyframe_Poses3D->points size : "<< Keyframe_Poses3D->points.size() <<std::endl;

        TicToc extract_from_priorMap;
        if(MappingConfig::scan_2_prior_map == 1){ //init load local map from prior
            PointType init_point; //can be changed in gnss map
            if(Keyframe_Poses3D->points.empty() ){
                init_point.x = 0.;
                init_point.y = 0.;
                init_point.z = 0.;
                init_point.intensity = 0.;
            }
            else{
                init_point.x = cloudInfo.T_w_l_curlidar.pose.pose.position.x;
                init_point.y = cloudInfo.T_w_l_curlidar.pose.pose.position.y;
                init_point.z = cloudInfo.T_w_l_curlidar.pose.pose.position.z;
                init_point.intensity = cloudInfo.T_w_l_curlidar.header.seq;
            }
            // 添加标志位，需要下一帧再加载。现在可以先写着1.5s换一次local map
            current_lidar_frameID =  init_point.intensity;
            if(current_lidar_frameID - last_lidar_frameID < 15){
                return ; // 不需要重复加载localMap_corner_and_surf
            }
            localMap_corner->clear();
            localMap_surf->clear();
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            EZLOG(INFO)<<"init_point is: " << init_point;
            kdtree_priorMap_surf->radiusSearch(
                    init_point, (double) MappingConfig::localMap_searchRadius,
                    pointSearchInd, pointSearchSqDis);
            std::cout<<"surf points in local map  : "<<pointSearchInd.size()<<std::endl;
            for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
                int id = pointSearchInd[i];
                localMap_surf->push_back(priorMap_surf->points[id]);
            }

            pointSearchInd.clear();
            pointSearchSqDis.clear();

            kdtree_priorMap_corner->radiusSearch(
                    init_point, (double) MappingConfig::surroundingKeyframeSearchRadius,
                    pointSearchInd, pointSearchSqDis);
            std::cout<<"corner points in local map ?  : "<<pointSearchInd.size()<<std::endl;
            for (int i = 0; i < (int) pointSearchInd.size(); ++i) {
                int id = pointSearchInd[i];
                localMap_corner->push_back(priorMap_corner->points[id]);
            }
//            kdtree_localMap_corner->setInputCloud(localMap_corner);
            std::cout<<"extract_from_priorMap is in ms: " << extract_from_priorMap.toc() <<std::endl;

        }
        else //else scan to scan_incremental
        {
            if(Keyframe_Poses3D->points.empty() != true)
            {
                extractNearby();
            }
        }

        downSizeFilterCorner.setInputCloud(localMap_corner);
        downSizeFilterCorner.filter(*localMap_corner_ds);
        localMap_corner_ds_num = localMap_corner_ds->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(localMap_surf);
        downSizeFilterSurf.filter(*localMap_surf_ds);
        localMap_surf_ds_num = localMap_surf_ds->size();
        std::cout << " localMap_surf_ds_num is: "<<localMap_surf_ds_num
                  << " localMap_corner_ds_num is: "<<localMap_corner_ds_num<<std::endl;
//        localMap_corner_and_surf[current_lidar_frameID] =
//                make_pair(*localMap_corner_ds, *localMap_surf_ds);
//        last_lidar_frameID = current_lidar_frameID;
//        std::cout << "localMap_corner_and_surf size is :" << localMap_corner_and_surf.size() << std::endl;

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
//        for (int i = numPoses - 1 ; i >= 0; --i) {
//            if (timeLaserInfoCur - Keyframe_Poses6D->points[i].time < 10.0)
//                near_keyframe_poses_ds_temp->push_back(Keyframe_Poses3D->points[i]);
//            else
//                break;
//        }
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

    }

    void updatePointAssociateToMap() {
        transPointAssociateToMap = trans2Affine3f(current_T_l_m);
    }
    void transforPoint2World() {

        Eigen::Matrix4d T_wm_temp = SensorConfig::T_L_B;
        Eigen::Matrix3d R_wm_temp = T_wm_temp.cast<double>().block<3, 3>(0, 0);
        Eigen::Vector3d t_wm_temp = T_wm_temp.cast<double>().block<3, 1>(0, 3);
        Eigen::Affine3f T_wm;
        T_wm.linear() = R_wm_temp.cast<float>();
        T_wm.translation() = t_wm_temp.cast<float>();

        Eigen::Affine3f T_ml = trans2Affine3f(current_T_l_m);
        T_wl = T_wm * T_ml; // T_wl = T_wm * T_ml
    }

    void cornerOptimization() {
        if(MappingConfig::scan_2_prior_map == 1){
            transforPoint2World();
        }
        else{
            updatePointAssociateToMap();
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

            pointAssociateToMap(&pointOri, &pointSel);
//            std::cout<<"pointOri: "<<pointOri<<" pointSel: "<<pointSel<<std::endl;
//            std::cout<<"pointOri: "<<pointOri<<" pointSel: "<<pointSel<<std::endl;
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

                    if (s > 0.1) {
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
        std::cout<<"surfOptimization()!" << std::endl;
        if(MappingConfig::scan_2_prior_map == 1){
            transforPoint2World();
        }
        else{
            updatePointAssociateToMap();
        }

// long startTime= System.currentTimeMillis();
//#pragma omp parallel for num_threads(numberOfCores)
//        current_surf_ds_num
        for (int i = 0; i < current_surf_ds_num; i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = current_surf_ds->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
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

                    if (s > 0.1) {
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
        float srx = sin(current_T_l_m[1]);
        float crx = cos(current_T_l_m[1]);
        float sry = sin(current_T_l_m[2]);
        float cry = cos(current_T_l_m[2]);
        float srz = sin(current_T_l_m[0]);
        float crz = cos(current_T_l_m[0]);


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

        current_T_l_m[0] += matX.at<float>(0, 0);
        current_T_l_m[1] += matX.at<float>(1, 0);
        current_T_l_m[2] += matX.at<float>(2, 0);
        current_T_l_m[3] += matX.at<float>(3, 0);
        current_T_l_m[4] += matX.at<float>(4, 0);
        current_T_l_m[5] += matX.at<float>(5, 0);

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
        if (Keyframe_Poses3D->points.empty()) return;

//        当前帧特征点
        std::cout << "current_corner_ds_num: " << current_corner_ds_num
                  << " current_surf_ds_num: " << current_surf_ds_num << std::endl;

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
            ROS_WARN(
                    "Not enough features! Only %d edge and %d planar features available.",
                    current_corner_ds_num, current_surf_ds_num);
        }
    }

//    IMU 初值
    void transformUpdate() {
        std::cout <<"transformUpdate " <<std::endl;
        if (cloudInfo.imuAvailable == true) {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                double imuWeight = SensorConfig::imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(current_T_l_m[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                        .getRPY(rollMid, pitchMid, yawMid);
                current_T_l_m[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, current_T_l_m[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                        .getRPY(rollMid, pitchMid, yawMid);
                current_T_l_m[1] = pitchMid;
            }
        }

        current_T_l_m[0] =
                constraintTransformation(current_T_l_m[0], MappingConfig::rotation_tollerance);
        current_T_l_m[1] =
                constraintTransformation(current_T_l_m[1], MappingConfig::rotation_tollerance);
        current_T_l_m[5] =
                constraintTransformation(current_T_l_m[5], MappingConfig::z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(current_T_l_m);
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
                current_T_l_m[3], current_T_l_m[4], current_T_l_m[5],
                current_T_l_m[0], current_T_l_m[1], current_T_l_m[2]);
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
            noiseModel::Diagonal::shared_ptr priorNoise =
                    noiseModel::Diagonal::Variances(
                            (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                    .finished());  // rad*rad, meter*meter

            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(current_T_l_m),
                                              priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(current_T_l_m));
        } else {
            noiseModel::Diagonal::shared_ptr odometryNoise =
                    noiseModel::Diagonal::Variances(
                            (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom =
                    pclPointTogtsamPose3(Keyframe_Poses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(current_T_l_m);

            mtxGraph.lock();
            gtSAMgraph.add(BetweenFactor<Pose3>(
                    Keyframe_Poses3D->size() - 1, Keyframe_Poses3D->size(),
                    poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(Keyframe_Poses3D->size(), poseTo);
            mtxGraph.unlock();
        }
    }

    void addGPSFactor() {
        if (gpsQueue.empty()) return;

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
        nav_msgs::Odometry thisGPS;
        if (syncGPS(gpsQueue, thisGPS, timeLaserInfoCur, 1.0 / SensorConfig::gpsFrequence)) {
            // GPS too noisy, skip
            float noise_x = thisGPS.pose.covariance[0];
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];

            // make sure the gps data is stable encough
            if (abs(noise_x) > SensorConfig::gpsCovThreshold || abs(noise_y) > SensorConfig::gpsCovThreshold)
                return;

//            float gps_x = thisGPS.pose.pose.position.x;
//            float gps_y = thisGPS.pose.pose.position.y;
//            float gps_z = thisGPS.pose.pose.position.z;
            double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
            Eigen::Vector3d LLA(thisGPS.pose.covariance[1], thisGPS.pose.covariance[2], thisGPS.pose.covariance[3]);
            geo_converter.Forward(LLA[0], LLA[1], LLA[2], gps_x, gps_y, gps_z);

            if (!SensorConfig::useGpsElevation) {
                gps_z = current_T_l_m[5];
                noise_z = 0.01;
            }
            // GPS not properly initialized (0,0,0)
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6) return;

            // Add GPS every a few meters
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < SensorConfig::gpsDistance)
                return;
            else
                lastGPSPoint = curGPSPoint;

            if (SensorConfig::debugGps) {
                ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
                ROS_INFO("curr gps cov: %f, %f , %f", thisGPS.pose.covariance[0],
                         thisGPS.pose.covariance[7], thisGPS.pose.covariance[14]);
            }

            gtsam::Vector Vector3(3);
            Vector3 << noise_x, noise_y, noise_z;
            // Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            noiseModel::Diagonal::shared_ptr gps_noise =
                    noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(Keyframe_Poses3D->size(),
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
                //                if (abs(gps_z - Keyframe_Poses3D->back().z) > 10.0) {
                //                    // ROS_WARN("Too large GNSS z noise %f", noise_z);
                //                    gtsam::Vector Vector3(3);
                //                    Vector3 << max(noise_x, 10000.0f), max(noise_y,
                //                    10000.0f), max(noise_z, 100000.0f);
                //                    // gps_noise =
                //                    noiseModel::Diagonal::Variances(Vector3);
                //                    // gps_factor =
                //                    gtsam::GPSFactor(Keyframe_Poses3D->size(),
                //                    gtsam::Point3(gps_x, gps_y, gps_z),
                //                    // gps_noise);
                //                }
                // add loop constriant
                mtxGraph.lock();
                gtSAMgraph.add(gps_factor);
                mtxGraph.unlock();
                gpsIndexContainer[Keyframe_Poses3D->size()] =
                        cloudKeyGPSPoses3D->size() - 1;
            }
        }
    }

    void saveKeyFramesAndFactor() {
        if (saveFrame() == false) return;

        // odom factor
        addOdomFactor();

        // gps factor
        if (SensorConfig::useGPS) addGPSFactor();

        cout << "****************************************************" << endl;
        gtSAMgraph.print("GTSAM Graph:\n");

        // add raw odomd
        nav_msgs::Odometry laserOdometryROS;
        transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS,
                            current_T_l_m);

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate =
                isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
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

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl <<
        // endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        current_T_l_m[0] = latestEstimate.rotation().roll();
        current_T_l_m[1] = latestEstimate.rotation().pitch();
        current_T_l_m[2] = latestEstimate.rotation().yaw();
        current_T_l_m[3] = latestEstimate.translation().x();
        current_T_l_m[4] = latestEstimate.translation().y();
        current_T_l_m[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
                new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
                new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*current_corner_ds, *thisCornerKeyFrame);
        pcl::copyPointCloud(*current_surf_ds, *thisSurfKeyFrame);

        // save key frame cloud, lidar frame
        Keyframe_corner_ds.push_back(thisCornerKeyFrame);
        Keyframe_surf_ds.push_back(thisSurfKeyFrame);

    }

    void correctPoses() {
        if (Keyframe_Poses3D->points.empty()) return;

    }


    void transformEiegn2Odom(double timestamp,
                             nav_msgs::Odometry &laserOdometryROS,
                             float transform[6]) {
        laserOdometryROS.header.stamp = ros::Time().fromSec(timestamp);
        laserOdometryROS.header.frame_id = SensorConfig::odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transform[3];
        laserOdometryROS.pose.pose.position.y = transform[4];
        laserOdometryROS.pose.pose.position.z = transform[5];
        laserOdometryROS.pose.pose.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(transform[0], transform[1],
                                                        transform[2]);
    }
    void transformLidar2World(double timestamp,
                           nav_msgs::Odometry &laserOdometryROS,
                           Eigen::Matrix4f &_Lidar_odom_world) {
        Eigen::Matrix3f rotation_matrix = _Lidar_odom_world.block<3, 3>(0, 0);
        Eigen::Quaternionf quaternion(rotation_matrix);
        laserOdometryROS.header.stamp = ros::Time().fromSec(timestamp);
        laserOdometryROS.header.frame_id = "map";
        laserOdometryROS.pose.pose.position.x = _Lidar_odom_world(0,3);
        laserOdometryROS.pose.pose.position.y = _Lidar_odom_world(1,3);
        laserOdometryROS.pose.pose.position.z = _Lidar_odom_world(2,3);
        laserOdometryROS.pose.pose.orientation.x = quaternion.x();
        laserOdometryROS.pose.pose.orientation.y = quaternion.y();
        laserOdometryROS.pose.pose.orientation.z = quaternion.z();
        laserOdometryROS.pose.pose.orientation.w = quaternion.w();
    }

    void publishOdometry() {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        nav_msgs::Odometry lidar_odom_world_ros;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(current_T_l_m);//T_ml
        Eigen::Matrix4f map_2_world = SensorConfig::T_L_B.cast<float>(); //T_wm
        Eigen::Matrix4f Lidar_odom_world = map_2_world * Lidarodom_2_map.matrix();// T_wl = T_wm * T_ml
        transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS,
                            current_T_l_m);
        transformLidar2World(timeLaserInfoCur, lidar_odom_world_ros,
                          Lidar_odom_world);

        pubLaserOdometryGlobal.publish(laserOdometryROS);
        publidar_odom_World.publish(lidar_odom_world_ros);

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(
                tf::createQuaternionFromRPY(current_T_l_m[0],
                                            current_T_l_m[1],
                                            current_T_l_m[2]),
                tf::Vector3(current_T_l_m[3], current_T_l_m[4],
                            current_T_l_m[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(
                t_odom_to_lidar, timeLaserInfoStamp, SensorConfig::odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        if (SensorConfig::useGPS) {
            if (gpsTransfromInit && SensorConfig::updateOrigin) {
                /** we first update the initial GPS origin points since it may not fix here */
                Eigen::Vector3d origin_point(Keyframe_Poses6D->at(0).x,
                                             Keyframe_Poses6D->at(0).y,
                                             Keyframe_Poses6D->at(0).z);
                // ENU->LLA
                Eigen::Vector3d update_origin_lla;
                geo_converter.Reverse(origin_point[0], origin_point[1], origin_point[2], update_origin_lla[0],
                                      update_origin_lla[1],
                                      update_origin_lla[2]);
                geo_converter.Reset(update_origin_lla[0], update_origin_lla[1], update_origin_lla[2]);
                std::cout << " origin points: " << originLLA.transpose() << std::endl;
                std::cout << " update origin points: " << update_origin_lla.transpose() << std::endl;
                originLLA = update_origin_lla;
                SensorConfig::updateOrigin = false;
                ROS_WARN("UPDATE MAP ORIGIN SUCCESS!");
            }

            /** we transform the optimized ENU point to LLA point for visualization with rviz_satellite*/
            Eigen::Vector3d curr_point(Keyframe_Poses6D->back().x,
                                       Keyframe_Poses6D->back().y,
                                       Keyframe_Poses6D->back().z);
            Eigen::Vector3d curr_lla;
            // ENU->LLA
            geo_converter.Reverse(curr_point[0], curr_point[1], curr_point[2], curr_lla[0], curr_lla[1],
                                  curr_lla[2]);
            //                std::cout << std::setprecision(9)
            //                          << "CURR LLA: " << originLLA.transpose() << std::endl;
            //                std::cout << std::setprecision(9)
            //                          << "update LLA: " << curr_lla.transpose() << std::endl;
            sensor_msgs::NavSatFix fix_msgs;
            fix_msgs.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
            fix_msgs.header.frame_id = SensorConfig::odometryFrame;
            fix_msgs.latitude = curr_lla[0];
            fix_msgs.longitude = curr_lla[1];
            fix_msgs.altitude = curr_lla[2];
            pubGPSOdometry.publish(fix_msgs);
        }

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental;  // incremental odometry msg
        static Eigen::Affine3f increOdomAffine;  // incremental odometry in affine
        if (lastIncreOdomPubFlag == false) {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(current_T_l_m);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() *
                                          incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch,
                                              yaw);
            if (cloudInfo.imuAvailable == true) {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4) {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                            .getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight))
                            .getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = SensorConfig::odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation =
                    tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames() {
        if (Keyframe_Poses3D->points.empty()) return;
        // publish key poses
        publishCloud(pubKeyPoses, Keyframe_Poses3D, timeLaserInfoStamp,
                     SensorConfig::odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, localMap_surf_ds,
                     timeLaserInfoStamp, SensorConfig::odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0) {
            pcl::PointCloud<PointType>::Ptr cloudOut(
                    new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(current_T_l_m);
            *cloudOut += *transformPointCloud(current_corner_ds, &thisPose6D);
            *cloudOut += *transformPointCloud(current_surf_ds, &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp,
                         SensorConfig::odometryFrame);
        }

        // publish path
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0) {
            if (lastSLAMInfoPubSize != Keyframe_Poses6D->size()) {
                lio_sam_6axis::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(
                        new pcl::PointCloud<PointType>());
                *cloudOut += *current_corner_ds;
                *cloudOut += *current_surf_ds;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut,
                                                        timeLaserInfoStamp, SensorConfig::lidarFrame);
                slamInfo.key_frame_poses =
                        publishCloud(ros::Publisher(), Keyframe_Poses6D, timeLaserInfoStamp,
                                     SensorConfig::odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(
                        new pcl::PointCloud<PointType>());
                *localMapOut += *localMap_corner_ds;
                *localMapOut += *localMap_surf_ds;
                slamInfo.key_frame_map = publishCloud(
                        ros::Publisher(), localMapOut, timeLaserInfoStamp, SensorConfig::odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = Keyframe_Poses6D->size();
            }
        }
    }
};

int main(int argc, char **argv) {

    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif

    ros::init(argc, argv, "loc_map_opt");

    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");

    Loc_mapOptimization LMO;
    EZLOG(INFO)<<"Prior Map Loc Optimization Started!";

    std::thread visualizeMapThread(&Loc_mapOptimization::visualizeGlobalMapThread, &LMO);

    ros::spin();

    visualizeMapThread.join();

    return 0;
}
