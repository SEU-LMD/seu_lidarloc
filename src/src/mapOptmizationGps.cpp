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
//#include "GeoGraphicLibInclude/Geocentric.hpp"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
//#include "GeoGraphicLibInclude/Geoid.hpp"
#include "dataSaver.h"
#include "cloud_info.h"
#include "utility.h"
#include "timer.h"
#include "MapSaver.h"
#include "utm/utm_convert.h"
#include "ivsensorgps.h"
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

class mapOptimization {
public:
    int frame_id = -1;
    ros::NodeHandle nh;
    std::mutex timer_mutex;
    bool timer_start = false;
    TicToc timer_cloud;//用于记录接收到数据的时间
    MapSaver map_saver;
    NonlinearFactorGraph gtSAMgraph;//GTSAM总的优化图
    Values initialEstimate;//GTSAM的初始位姿
    Values optimizedEstimate;
    ISAM2 *isam;//GTSAM
    Values isamCurrentEstimate;//GTSAM优化后的结果保存优化后的所有关键帧的位姿
    Eigen::MatrixXd poseCovariance;//位姿的先验噪声


    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    //ros::Publisher pubGPSOdometry;
    ros::Publisher pubGnssOdometry;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;//发布关键帧面点的降采样点云（点云的位姿经过因子图优化）
    ros::Publisher pubRecentKeyFrame;//发布关键帧的降采样点云（点云的位姿经过因子图优化
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubCloudRaw;
    ros::Publisher pubLoopConstraintEdge;
    ros::Publisher pubGpsConstraintEdge;
    ros::Publisher publidar_odom_World;

    ros::Publisher pubSLAMInfo;

    ros::Subscriber subCloud;
    ros::Subscriber subGnss;
//  ros::Subscriber subGps;
    ros::Subscriber subLoop;


    std::deque<nav_msgs::Odometry> gpsQueue;
    lio_sam_6axis::cloud_info cloudInfo;

    vector <pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector <pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector <pcl::PointCloud<PointType>::Ptr> laserCloudRawKeyFrames;

    std::vector <Eigen::Matrix4d> keyframePosestrans;
    std::vector <nav_msgs::Odometry> keyframeRawOdom;
    std::vector<double> keyframeTimes;
    //    std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
    std::vector<sensor_msgs::PointCloud2> keyframeCloudDeskewed;
    std::vector<double> keyframeDistances;
    std::vector<gtsam::GPSFactor> keyframeGPSfactor;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr cloudKeyGPSPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses2D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr
            laserCloudCornerLast;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr
            laserCloudSurfLast;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr
            laserCloudCornerLastDS;  // downsampled corner feature set from
    // odoOptimization
    pcl::PointCloud<PointType>::Ptr
            laserCloudSurfLastDS;  // downsampled surf feature set from
    // odoOptimization

    //pcl::PointCloud<PointType>::Ptr laserCloudRaw;    // giseop
   // pcl::PointCloud<PointType>::Ptr laserCloudRawDS;  // giseop

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType>
            laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType>
            laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>>
            laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType>
            downSizeFilterSurroundingKeyPoses;        // for surrounding key poses of
    // scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterRaw;

    std::unique_ptr<DataSaver> dataSaverPtr;

    bool flg_exit = false;
    int lastLoopIndex = -1;

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    // double timeStampInitial;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;
    std::mutex mtxGpsInfo;
    std::mutex mtxGraph;

    Eigen::Vector3d originLLA;
    bool systemInitialized = false;
    bool gpsTransfromInit = false;
    GeographicLib::LocalCartesian geo_converter;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer;  // from new to old
    map<int, int> gpsIndexContainer;   // from new to old
    vector <pair<int, int>> loopIndexQueue;
    vector <gtsam::Pose3> loopPoseQueue;
    vector <gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque <std_msgs::Float64MultiArray> loopInfoVec;
    nav_msgs::Path globalPath;
    // nav_msgs::Path globalGpsPath;
    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Vector3d t_w_cur;
    Eigen::Quaterniond q_w_cur;
    float q_w_roll, q_w_pitch, q_w_yaw;

    double updateinit_cost_time;
    double exactsurrounding_key_frames_cost_time;
    double downsample_current_sacn_cost_time;
    double scantomap_cost_time;
    double addfactors_cost_time;
    double correctposes;
    double publish_odometry_cost_time;


    //  GpsTools gpsTools;
    //  Eigen::Vector3d optimized_lla;
    //  std::vector<Eigen::Vector3d> lla_vector;

    //  string savePCDDirectory;
    //  string scene_name;

    mapOptimization() {
        std::cout << "init mapOptimization function" << std::endl;
        //std::thread saveMapThread(&MapSaver::do_work, &map_saver);//comment fyy
        std::cout << "after start map thread" << std::endl;


        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        if (SensorConfig::useGPS) {
            //pubGPSOdometry = nh.advertise<sensor_msgs::NavSatFix>("lio_sam_6axis/mapping/odometry_gps", 1);
            pubGnssOdometry = nh.advertise<gps_imu::ivsensorgps>("lio_sam_6axis/mapping/odometry_gnss", 1);
        }

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/map_global", 1);
        pubLaserOdometryGlobal =
                nh.advertise<nav_msgs::Odometry>("lio_sam_6axis/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>(
                "lio_sam_6axis/mapping/odometry_incremental", 1);
//        publidar_odom_World =
//                nh.advertise<nav_msgs::Odometry>("lidar_odometry_world", 1);

        std::cout << "get in to laserCloudInfoHandler" << std::endl;
        subCloud = nh.subscribe<lio_sam_6axis::cloud_info>(
                "lio_sam_6axis/feature/cloud_info", 1,
                &mapOptimization::laserCloudInfoHandler, this,
                ros::TransportHints().tcpNoDelay());
        std::cout << "get out of the laserCloudInfoHandler" << std::endl;


        subGnss = nh.subscribe<nav_msgs::Odometry>(
                "/gnss_odom", 2000, &mapOptimization::gnssHandler, this,
                ros::TransportHints().tcpNoDelay());
//        subGps = nh.subscribe<nav_msgs::Odometry>(
//                "/gps_odom", 200, &mapOptimization::gpsHandler, this,
//                ros::TransportHints().tcpNoDelay());

        subLoop = nh.subscribe<std_msgs::Float64MultiArray>(
                "lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler,
                this, ros::TransportHints().tcpNoDelay());


        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(
                "/lio_sam_6axis/mapping/loop_closure_constraints", 1);
        pubGpsConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(
                "/lio_sam_6axis/mapping/gps_constraints", 1);
        publidar_odom_World =
                nh.advertise<nav_msgs::Odometry>("lidar_odometry_world", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(
                "lio_sam_6axis/mapping/cloud_registered_raw", 1);
        pubCloudRaw = nh.advertise<sensor_msgs::PointCloud2>("cloud_deskewed", 1);

        pubSLAMInfo = nh.advertise<lio_sam_6axis::cloud_info>(
                "lio_sam_6axis/mapping/slam_info", 1);

        downSizeFilterCorner.setLeafSize(
                MappingConfig::mappingCornerLeafSize, MappingConfig::mappingCornerLeafSize,
                MappingConfig::mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(MappingConfig::mappingSurfLeafSize, MappingConfig::mappingSurfLeafSize,
                                       MappingConfig::mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(MappingConfig::mappingSurfLeafSize, MappingConfig::mappingSurfLeafSize,
                                      MappingConfig::mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(
                MappingConfig::surroundingKeyframeDensity, MappingConfig::surroundingKeyframeDensity,
                MappingConfig::surroundingKeyframeDensity);  // for surrounding key poses of
        // scan-to-map optimization

//        const float rawMapFilterSize = 0.5;  // giseop
//        downSizeFilterRaw.setLeafSize(rawMapFilterSize, rawMapFilterSize,
//                                      rawMapFilterSize);  // giseop

        // set log dir
        dataSaverPtr = std::make_unique<DataSaver>();

        allocateMemory();
        std::cout << "******************" << MappingConfig::savePCDDirectory << std::endl;
    }

    void allocateMemory() {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyGPSPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses2D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(
                new pcl::PointCloud<PointType>());  // corner feature set from
        // odoOptimization
        laserCloudSurfLast.reset(
                new pcl::PointCloud<PointType>());  // surf feature set from
//        laserCloudPose.reset(
//                new pcl::PointCloud<PointType>()); //pose feature set from
        // odoOptimization
        laserCloudCornerLastDS.reset(
                new pcl::PointCloud<PointType>());  // downsampled corner featuer set
        // from odoOptimization
        laserCloudSurfLastDS.reset(
                new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
        // odoOptimization

        //laserCloudRaw.reset(new pcl::PointCloud<PointType>());    // giseop
        //laserCloudRawDS.reset(new pcl::PointCloud<PointType>());  // giseop

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

        for (int i = 0; i < 6; ++i) {
            transformTobeMapped[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }


    void laserCloudInfoHandler(const lio_sam_6axis::cloud_infoConstPtr &msgIn) {

        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();
      // extract info and feature cloudi

        // extract info and feature cloud
        cloudInfo = *msgIn;

        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
       // pcl::fromROSMsg(msgIn->cloud_deskewed, *laserCloudRaw);  // deskewed data


        t_w_cur[0]= msgIn->T_w_l_curlidar.pose.pose.position.x;
        t_w_cur[1]= msgIn->T_w_l_curlidar.pose.pose.position.y;
        t_w_cur[2]= msgIn->T_w_l_curlidar.pose.pose.position.z;

        q_w_cur.x()= msgIn->T_w_l_curlidar.pose.pose.orientation.x;
        q_w_cur.y()= msgIn->T_w_l_curlidar.pose.pose.orientation.y;
        q_w_cur.z()= msgIn->T_w_l_curlidar.pose.pose.orientation.z;
        q_w_cur.w()= msgIn->T_w_l_curlidar.pose.pose.orientation.w;
        q_w_cur.normalize();

//         PoseT T_w_b_lidar_start;
//         //Eigen::Vector3d t_w_lidar_start;
//         //Eigen::Quaterniond q_w_lidar_star;
//         T_w_b_lidar_start  = PoseT (t_w_lidar_start,q_w_lidar_start);
//         PoseT world_2_map =PoseT(SensorConfig::T_L_B.cast<double>().inverse());
//         PoseT T_w_lidar =PoseT( world_2_map * T_w_b_lidar_start);
//         Eigen::Quaterniond q_lidar= T_w_lidar.GetQ();
//         Eigen::Vector3d  t_lidar = T_w_lidar.GetT();

        Eigen::Matrix3d q_w_cur_matrix = q_w_cur.toRotationMatrix();
        q_w_pitch = asin(-q_w_cur_matrix(2, 0)); // 计算pitch
        if (cos(q_w_pitch) != 0) {
            q_w_roll = atan2(q_w_cur_matrix(2, 1), q_w_cur_matrix(2, 2)); // 计算roll
            q_w_yaw = atan2(q_w_cur_matrix(1, 0), q_w_cur_matrix(0, 0));  // 计算yaw
        } else {
            q_w_roll = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
            q_w_yaw = atan2(-q_w_cur_matrix(0, 1), q_w_cur_matrix(1, 1)); // 计算yaw
        }


        std::lock_guard <std::mutex> lock(mtx);

        static double timeLastProcessing = -1;

        if (timeLaserInfoCur - timeLastProcessing >= MappingConfig::mappingProcessInterval) {
            timeLastProcessing = timeLaserInfoCur;
            //cout <<"00000"<<endl;
            updateInitialGuess();

          cout<<"go out of update" <<endl;
            if (systemInitialized) {

                extractSurroundingKeyFrames();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                correctPoses();

                publishOdometry();

                std::ofstream foutC("/home/wxy/seu/seu_lidarloc--latest/src/result/time.txt",std::ios::app);
                //double turetime = odomAftMapped.header.stamp.toSec();
                foutC.setf(std::ios::fixed,std::ios::floatfield);
                for (int i = 0; i < (int) cloudKeyPoses3D->size(); i++) {
                    foutC.precision(5);
                    foutC <<i<<" "
                          <<updateinit_cost_time<<" "
                          <<exactsurrounding_key_frames_cost_time<<" "
                          <<downsample_current_sacn_cost_time<<" "
                          <<scantomap_cost_time<<" "
                          <<addfactors_cost_time<<" "
                          <<correctposes<<" "
                          <<publish_odometry_cost_time<<std::endl;
                    foutC.close();

                }


                publishFrames();

                timer_mutex.lock();
                timer_cloud.tic();
                timer_start = true;
                timer_mutex.unlock();
            }
        }
    }
//
    void gnssHandler(const nav_msgs::Odometry::ConstPtr &mapMsg) {
        if (SensorConfig::useGPS) {
            mtxGpsInfo.lock();
            gpsQueue.push_back(*mapMsg);
            mtxGpsInfo.unlock();
        }
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

        pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
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

//    void lidar2word(const float transform_lidar[6], float transform_in_world[6]){
//        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(transformTobeMapped);//T_ml
//        Eigen::Matrix4f map_2_world = SensorConfig::T_L_B.cast<float>(); //T_wm
//        Eigen::Matrix4f Lidar_odom_world = map_2_world * Lidarodom_2_map.matrix();// T_wl = T_wm * T_ml
//        Eigen::Matrix3f rotation_matrix = Lidar_odom_world.block<3, 3>(0, 0);
//        double roll_world, pitch_word, yaw_word;
//        pitch_word = asin(-Lidar_odom_world(2, 0)); // 计算pitch
//        if (cos(pitch_word) != 0) {
//            roll_world = atan2(Lidar_odom_world(2, 1), Lidar_odom_world(2, 2)); // 计算roll
//            yaw_word = atan2(Lidar_odom_world(1, 0), Lidar_odom_world(0, 0));  // 计算yaw
//        } else {
//            roll_world = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
//            yaw_word = atan2(-Lidar_odom_world(0, 1), Lidar_odom_world(1, 1)); // 计算yaw
//        }
//        transform_in_world[3] =Lidar_odom_world(0,3);
//        transform_in_world[4] =Lidar_odom_world(1,3);
//        transform_in_world[5] =Lidar_odom_world(2,3);
//        transform_in_world[0] =roll_world;
//        transform_in_world[1] =pitch_word;
//        transform_in_world[2] =yaw_word;
//    };

        void visualizeGlobalMapThread() {
//            ros::Rate rate(0.2);
//            while (ros::ok()) {
//                rate.sleep();
               publishGlobalMap();
//            }
        }

    //如果5秒内没有接收到点云数据，就会自动保存
    void savePathThread() {
//        while (1) {
//
//                //保证有了位姿数据
//                if (timer_start == true) {
//
//                    timer_mutex.lock();
//                    double delta_time = timer_cloud.toc();
//                    timer_mutex.unlock();
//                    //EZLOG(INFO) << "delta_time = " << delta_time << std::endl;
//                    if (delta_time > 5000) {
//
//
//                        if (SensorConfig::useGPS) {
//                            Eigen::Vector3d optimized_lla;
//                            Eigen::Vector3d first_point(cloudKeyPoses6D->at(0).x,
//                                                        cloudKeyPoses6D->at(0).y,
//                                                        cloudKeyPoses6D->at(0).z);
//                            // we save optimized origin gps point
//                            geo_converter.Reverse(first_point[0], first_point[1], first_point[2],
//                                                  optimized_lla[0], optimized_lla[1], optimized_lla[2]);
//
//
//                            sad::UTMCoordinate utm_coor;
//                            Eigen::Vector2d lat_lon;
//                            Eigen::Vector3d utm;
//                            //将经度和纬度给到lat_lon
//                            lat_lon << optimized_lla[0], optimized_lla[1];
//
//                            std::cout << std::setprecision(15) << "optimized_lla" << optimized_lla << std::endl;
//                            //调用utm_convert.h中的函数将精度纬度转换到UTM坐标系下
//                            sad::LatLon2UTM(lat_lon.head<2>(), utm_coor);
//                            utm_coor.z_ = optimized_lla[2];
//
//                            utm[0] = utm_coor.xy_[0];
//                            utm[1] = utm_coor.xy_[1];
//                            utm[2] = utm_coor.z_;
//
//                            dataSaverPtr->saveOriginGPS(utm);
//                            EZLOG(INFO) << " lat_lon = " << utm << endl;
//                        }
//
//                        dataSaverPtr->saveOptimizedVerticesTUM(isamCurrentEstimate);
//                        EZLOG(INFO) << "Saving poses completed! " << endl;
//                        while (1);
//                    }
//                }//end if(delta_time>5000)
//
//                sleep(0.1);
//
//
//            }
     }

        void publishGlobalMap() {
//            if (pubLaserCloudSurround.getNumSubscribers() == 0) return;
//
//            if (cloudKeyPoses3D->points.empty() == true) return;
//
//            pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(
//                    new pcl::KdTreeFLANN<PointType>());;
//            pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(
//                    new pcl::PointCloud<PointType>());
//            pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(
//                    new pcl::PointCloud<PointType>());
//            pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(
//                    new pcl::PointCloud<PointType>());
//            pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(
//                    new pcl::PointCloud<PointType>());
//
//            // kd-tree to find near key frames to visualize
//            std::vector<int> pointSearchIndGlobalMap;
//            std::vector<float> pointSearchSqDisGlobalMap;
//            // search near key frames to visualize
//            mtx.lock();
//            kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
//            kdtreeGlobalMap->radiusSearch(
//                    cloudKeyPoses3D->back(), MappingConfig::globalMapVisualizationSearchRadius,
//                    pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
//            mtx.unlock();
//
//            for (int i = 0; i < (int) pointSearchIndGlobalMap.size(); ++i)
//                globalMapKeyPoses->push_back(
//                        cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
//            // downsample near selected key frames
//            pcl::VoxelGrid <PointType>
//                    downSizeFilterGlobalMapKeyPoses;  // for global map visualization
//            downSizeFilterGlobalMapKeyPoses.setLeafSize(
//                    MappingConfig::globalMapVisualizationPoseDensity, MappingConfig::globalMapVisualizationPoseDensity,
//                    MappingConfig::globalMapVisualizationPoseDensity);  // for global map visualization
//            downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
//            downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
//            for (auto &pt: globalMapKeyPosesDS->points) {
//                kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap,
//                                                pointSearchSqDisGlobalMap);
//                pt.intensity =
//                        cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
//            }
//
//            // extract visualized and downsampled key frames
//            for (int i = 0; i < (int) globalMapKeyPosesDS->size(); ++i) {
//                if (pointDistance(globalMapKeyPosesDS->points[i],
//                                  cloudKeyPoses3D->back()) >
//                    MappingConfig::globalMapVisualizationSearchRadius)
//                    continue;
//                int thisKeyInd = (int) globalMapKeyPosesDS->points[i].intensity;
//                *globalMapKeyFrames +=
//                        *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],
//                                             &cloudKeyPoses6D->points[thisKeyInd]);
//                *globalMapKeyFrames += *transformPointCloud(
//                        surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
//            }
//            // downsample visualized points
//            pcl::VoxelGrid <PointType>
//                    downSizeFilterGlobalMapKeyFrames;  // for global map visualization
//            downSizeFilterGlobalMapKeyFrames.setLeafSize(
//                    MappingConfig::globalMapVisualizationLeafSize, MappingConfig::globalMapVisualizationLeafSize,
//                    MappingConfig::globalMapVisualizationLeafSize);  // for global map visualization
//            downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
//            downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
//            publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS,
//                         timeLaserInfoStamp, SensorConfig::odometryFrame);
        }
       //回环检测线程
        void loopClosureThread() {
            if (MappingConfig::loopClosureEnableFlag == false) return;
           //设置回环检测的频率 loopClosureFrequency默认为 1hz
            ros::Rate rate(MappingConfig::loopClosureFrequency);
            while (ros::ok()) {
                ros::spinOnce();

                performLoopClosure();
                visualizeLoopClosure();

                if (SensorConfig::useGPS) visualGPSConstraint();

                rate.sleep();
            }
        }

        void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg) {
            std::lock_guard <std::mutex> lock(mtxLoopInfo);
            if (loopMsg->data.size() != 2) return;

            loopInfoVec.push_back(*loopMsg);

            while (loopInfoVec.size() > 5) loopInfoVec.pop_front();
        }
      //
        void performLoopClosure() {
            if (cloudKeyPoses3D->points.empty() == true) return;

            mtx.lock();
            //把存储关键帧额位姿的点云copy出来，避免线程冲突
            *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
            *copy_cloudKeyPoses2D = *cloudKeyPoses3D;
            *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
            mtx.unlock();

            // find keys
            int loopKeyCur;
            int loopKeyPre;

            //外部通知的回环信息
            //if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
                //在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
                if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false) return;

            // extract cloud
            //检测回环存在后则可以计算检测出这两帧的位姿变换
            pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(
                    new pcl::PointCloud<PointType>());//声明当前关键帧的点云
            pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(
                    new pcl::PointCloud<PointType>());//和历史回环帧周围的点云（局部地图）
            {
                 //回环帧把自己提取出来
                loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
                //回环帧把自己周围一些点云取出来，也就是构成一个帧局部地图的一个匹配问题
                loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre,
                                      MappingConfig::historyKeyframeSearchNum);
                if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                    return;
                //把局部地图发布出来供rviz可视化使用
                if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                    publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp,
                                 SensorConfig::odometryFrame);
            }
           //现在有了当前关键帧投到地图坐标系下的点云和历史回环帧投到地图坐标系下的局部地图，那么接下来就可以进行两者的icp位姿变换求解
            // ICP Settings
            static pcl::IterativeClosestPoint <PointType, PointType> icp;//使用简单的icp来进行帧到局部地图的配准
            //设置最大相关距离 15m
            icp.setMaxCorrespondenceDistance(MappingConfig::historyKeyframeSearchRadius * 2);
            //最大优化次数
            icp.setMaximumIterations(100);
            //单次变换范围
            icp.setTransformationEpsilon(1e-6);
            //残差设置
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            //设置两个点云
            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            //执行配准
            pcl::PointCloud<PointType>::Ptr unused_result(
                    new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
           //检测icp是否收敛 且 得分是否满足要求
            if (icp.hasConverged() == false ||
                icp.getFitnessScore() > MappingConfig::historyKeyframeFitnessScore)
                return;

            // publish corrected cloud
            //把修正后的当前点云发布供可视化使用
            if (pubIcpKeyFrames.getNumSubscribers() != 0) {
                pcl::PointCloud<PointType>::Ptr closed_cloud(
                        new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud,
                                         icp.getFinalTransformation());
                publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp,
                             SensorConfig::odometryFrame);
            }

            // Get pose transformation
            float x_lidar, y_lidar, z_lidar, roll_lidar, yaw_lidar, pitch_lidar;
            Eigen::Affine3f correctionLidarFrame;
            //获得两个点云的变换矩阵结果，
            correctionLidarFrame = icp.getFinalTransformation();
            // transform from world origin to wrong pose
            //取出当前帧的位姿
            Eigen::Affine3f tWrong =
                    pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
            // transform from world origin to corrected pose
            //将icp结果补偿过去，就是当前帧的更为准确的位姿结果
            Eigen::Affine3f tCorrect =
                    correctionLidarFrame *
                    tWrong;  // pre-multiplying -> successive rotation about a fixed frame
//          Eigen::Matrix4f map_2_world = SensorConfig::T_L_B.cast<float>(); //T_wm
//          Eigen::Matrix4f tCorrect_world = map_2_world * tCorrect.matrix();// T_wl = T_wm * T_ml
//          Eigen::Transform<float, 3, Eigen::Affine> a3f_transform (tCorrect_world);

            pcl::getTranslationAndEulerAngles(tCorrect, x_lidar, y_lidar, z_lidar,roll_lidar,pitch_lidar,yaw_lidar);

          //将当前帧补偿后的位姿 转换成 gtsam的形式
            //From 和 To相当于帧间约束的因子，To是历史回环帧的位姿
            gtsam::Pose3 poseFrom =
                    Pose3(Rot3::RzRyRx(roll_lidar, pitch_lidar, yaw_lidar), Point3(x_lidar, y_lidar, z_lidar));

//            float loop_world[6];
//            float loop_lidar2world[6];
//            loop_world[3] = copy_cloudKeyPoses6D->points[loopKeyPre].x;
//            loop_world[4] = copy_cloudKeyPoses6D->points[loopKeyPre].y;
//            loop_world[5] = copy_cloudKeyPoses6D->points[loopKeyPre].z;
//            loop_world[0] = copy_cloudKeyPoses6D->points[loopKeyPre].roll;
//            loop_world[1] = copy_cloudKeyPoses6D->points[loopKeyPre].pitch;
//            loop_world[2] = copy_cloudKeyPoses6D->points[loopKeyPre].yaw;

           // lidar2word(loop_world,loop_lidar2world);


//            gtsam::Pose3 poseTo =
//                    pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
          gtsam::Pose3 poseTo =
                     pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);

            //使用icp的得分作为他们的约束噪声项
            gtsam::Vector Vector6(6);
            float noiseScore = icp.getFitnessScore();
            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
                    noiseScore;
            noiseModel::Diagonal::shared_ptr constraintNoise =
                    noiseModel::Diagonal::Variances(Vector6);

            // Add pose constraint
            //将两帧索引，两帧相对位姿和噪声作为回环约束 送入对列
            mtx.lock();
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));//两帧索引
            loopPoseQueue.push_back(poseFrom.between(poseTo));//当前帧与历史回环帧的相对位姿
            loopNoiseQueue.push_back(constraintNoise);//
            mtx.unlock();

            // add loop constriant
            //保存已经存在的约束对
            loopIndexContainer[loopKeyCur] = loopKeyPre;
            lastLoopIndex = loopKeyCur;
        }

        bool detectLoopClosureDistance(int *latestID, int *closestID) {
        //检测最新帧是否和其它帧形成回环
            int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
            int loopKeyPre = -1;

            // check loop constraint added before
            //检查一下较晚帧是否和别的形成了回环，如果有就算了
            //因为当前帧刚刚出现，不会和其它帧形成回环，所以基本不会触发
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
            //b不需要关注z值，将其设置为0
            for (int i = 0; i < copy_cloudKeyPoses2D->size(); ++i) {
                copy_cloudKeyPoses2D->at(i).z = 0;
            }

            // find the closest history key frame
            std::vector<int> pointSearchIndLoop;
            std::vector<float> pointSearchSqDisLoop;

            kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses2D);
            //根据最后一个关键帧的平移信息，寻找离他一定距离内的其它关键帧，搜索范围15米
            kdtreeHistoryKeyPoses->radiusSearch(
                    copy_cloudKeyPoses2D->back(), MappingConfig::historyKeyframeSearchRadius,
                    pointSearchIndLoop, pointSearchSqDisLoop, 0);
            //
            for (int i = 0; i < (int) pointSearchIndLoop.size(); ++i) {
                int id = pointSearchIndLoop[i];
                //历史帧，必须比当前帧间隔30s以上
                if (abs(copy_cloudKeyPoses6D->points[id].time- timeLaserInfoCur) >
                    MappingConfig::historyKeyframeSearchTimeDiff) {
                    loopKeyPre = id;
                    break;
                }
            }
           //如果没有找到回环或者回环找到自己身上去了，就认为是本次回环寻找失败
            if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;

            // we also need to care about the accumulated distance between keyframe;
            // LOOPs that are too close together have no meaning and may reduce
            // accuracy. For example, the lidar starts to move after being stationary
            // for 30s in a certain place. At this time, the IMU should be trusted more
            // than the lidar.
            //关键帧间的距离应该大于12
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
             //找到了当前关键帧和历史回环帧，赋值当前帧和历史回环帧的id;
            *latestID = loopKeyCur;
            *closestID = loopKeyPre;

            return true;
        }
       //提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
        void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes,
                                   const int &key, const int &searchNum) {
            // extract near keyframes
            nearKeyframes->clear();
            int cloudSize = copy_cloudKeyPoses6D->size();
            //通过-searchNum 到 +searchNum遍历帧的搜索范围
            for (int i = -searchNum; i <= searchNum; ++i) {
                int keyNear = key + i;
                if (keyNear < 0 || keyNear >= cloudSize) continue;
                //把对应角点和面点的点云转到世界坐标系下去
                *nearKeyframes +=
                        *transformPointCloud(cornerCloudKeyFrames[keyNear],
                                             &copy_cloudKeyPoses6D->points[keyNear]);
                *nearKeyframes += *transformPointCloud(
                        surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            }

            if (nearKeyframes->empty()) return;

            // downsample near keyframes
            //把点云下采样，存放到cloud_temp
            pcl::PointCloud<PointType>::Ptr cloud_temp(
                    new pcl::PointCloud<PointType>());
            downSizeFilterICP.setInputCloud(nearKeyframes);
            downSizeFilterICP.filter(*cloud_temp);
            *nearKeyframes = *cloud_temp;
        }
       //可视化回环， no need
        void visualizeLoopClosure() {
//            if (loopIndexContainer.empty()) return;
//
//            visualization_msgs::MarkerArray markerArray;
//            // loop nodes
//            visualization_msgs::Marker markerNode;
//            markerNode.header.frame_id = SensorConfig::odometryFrame;
//            markerNode.header.stamp = timeLaserInfoStamp;
//            markerNode.action = visualization_msgs::Marker::ADD;
//            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
//            markerNode.ns = "loop_nodes";
//            markerNode.id = 0;
//            markerNode.pose.orientation.w = 1;
//            markerNode.scale.x = 0.15;
//            markerNode.scale.y = 0.15;
//            markerNode.scale.z = 0.15;
//            markerNode.color.r = 0;
//            markerNode.color.g = 0.8;
//            markerNode.color.b = 1;
//            markerNode.color.a = 1;
//            // loop edges
//            visualization_msgs::Marker markerEdge;
//            markerEdge.header.frame_id = SensorConfig::odometryFrame;
//            markerEdge.header.stamp = timeLaserInfoStamp;
//            markerEdge.action = visualization_msgs::Marker::ADD;
//            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
//            markerEdge.ns = "loop_edges";
//            markerEdge.id = 1;
//            markerEdge.pose.orientation.w = 1;
//            markerEdge.scale.x = 0.1;
//            markerEdge.color.r = 0.9;
//            markerEdge.color.g = 0.9;
//            markerEdge.color.b = 0;
//            markerEdge.color.a = 1;
//
//            for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end();
//                 ++it) {
//                int key_cur = it->first;
//                int key_pre = it->second;
//                geometry_msgs::Point p;
//                p.x = copy_cloudKeyPoses6D->points[key_cur].x;
//                p.y = copy_cloudKeyPoses6D->points[key_cur].y;
//                p.z = copy_cloudKeyPoses6D->points[key_cur].z;
//                markerNode.points.push_back(p);
//                markerEdge.points.push_back(p);
//                p.x = copy_cloudKeyPoses6D->points[key_pre].x;
//                p.y = copy_cloudKeyPoses6D->points[key_pre].y;
//                p.z = copy_cloudKeyPoses6D->points[key_pre].z;
//                markerNode.points.push_back(p);
//                markerEdge.points.push_back(p);
//            }
//
//            markerArray.markers.push_back(markerNode);
//            markerArray.markers.push_back(markerEdge);
//            pubLoopConstraintEdge.publish(markerArray);
        }
        //可视化GPS约束 不需要
        void visualGPSConstraint() {
//            if (gpsIndexContainer.empty()) return;
//           //visualization_msgs
//            visualization_msgs::MarkerArray markerArray;
//            // gps nodes
//            visualization_msgs::Marker markerNode;
//            markerNode.header.frame_id = SensorConfig::odometryFrame;
//            markerNode.header.stamp = timeLaserInfoStamp;
//            markerNode.action = visualization_msgs::Marker::ADD;
//            markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
//            markerNode.ns = "gps_nodes";
//            markerNode.id = 0;
//            markerNode.pose.orientation.w = 1;
//            markerNode.scale.x = 0.3;
//            markerNode.scale.y = 0.3;
//            markerNode.scale.z = 0.3;
//            markerNode.color.r = 0.8;
//            markerNode.color.g = 0;
//            markerNode.color.b = 1;
//            markerNode.color.a = 1;
//
//            // loop edges
//            visualization_msgs::Marker markerEdge;
//            markerEdge.header.frame_id = SensorConfig::odometryFrame;
//            markerEdge.header.stamp = timeLaserInfoStamp;
//            markerEdge.action = visualization_msgs::Marker::ADD;
//            markerEdge.type = visualization_msgs::Marker::LINE_LIST;
//            markerEdge.ns = "gps_edges";
//            markerEdge.id = 1;
//            markerEdge.pose.orientation.w = 1;
//            markerEdge.scale.x = 0.2;
//            markerEdge.color.r = 0.9;
//            markerEdge.color.g = 0;
//            markerEdge.color.b = 0.1;
//            markerEdge.color.a = 1;
//
//            for (auto it = gpsIndexContainer.begin(); it != gpsIndexContainer.end();
//                 ++it) {
//                int key_cur = it->first;
//                int key_pre = it->second;
//
//                geometry_msgs::Point p;
//                p.x = copy_cloudKeyPoses6D->points[key_cur].x;
//                p.y = copy_cloudKeyPoses6D->points[key_cur].y;
//                p.z = copy_cloudKeyPoses6D->points[key_cur].z;
//                markerNode.points.push_back(p);
//                markerEdge.points.push_back(p);
//
//                p.x = cloudKeyGPSPoses3D->points[key_pre].x;
//                p.y = cloudKeyGPSPoses3D->points[key_pre].y;
//                p.z = cloudKeyGPSPoses3D->points[key_pre].z;
//                markerNode.points.push_back(p);
//                markerEdge.points.push_back(p);
//            }
//
//            markerArray.markers.push_back(markerNode);
//            markerArray.markers.push_back(markerEdge);
//            pubGpsConstraintEdge.publish(markerArray);
        }


//
void updateInitialGuess() {
        //每帧激光初位姿,将四元数转换到欧拉角
        TicToc timer;
        timer.tic();

        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastGnssTransformation;
        // initialization the first frame
        if (cloudKeyPoses3D->points.empty()) {
            systemInitialized = false;
            if (SensorConfig::useGPS) {
                ROS_INFO("GPS use to init pose");
                //拿到局部坐标系下GNSS的原点位置
                Eigen::Vector3d enu;
                enu[0] =gpsQueue.front().pose.pose.position.x;
                enu[1] = gpsQueue.front().pose.pose.position.y;
                enu[2] = gpsQueue.front().pose.pose.position.z;

//                  enu[0] = t_w_cur[0];
//                  enu[1] = t_w_cur[1];
//                  enu[2] = t_w_cur[2];
                double q_w_yaw_cur = q_w_yaw * (180.0 / M_PI);

                if (SensorConfig::debugGps) {

                    std::cout << "initial gps yaw: " << q_w_yaw_cur << std::endl;
                    std::cout << "GPS Position: " << enu.transpose() << std::endl;
                    //std::cout << "GPS LLA: " << originLLA.transpose() << std::endl;
                    ROS_WARN("GPS init success");
                }

                /** add the first factor, we need this origin GPS point for prior map based localization,
                    * but we need to optimize its value by pose graph if the origin gps RTK status is not fixed.*/
                PointType gnssPoint;
                gnssPoint.x = enu[0],
                gnssPoint.y = enu[1],
                gnssPoint.z = enu[2];
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
                noiseModel::Diagonal::shared_ptr gps_noise =
                        noiseModel::Diagonal::Variances(Vector3);
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

        //static bool lastGnssPreTransAvailable = false;
       // static Eigen::Affine3f lastGnssPreTransformation;
        // if (cloudInfo.odomAvailable == true) {
        /***
         * transback 可以直接给值
         */

        Eigen::Affine3f transBack = pcl::getTransformation(
         t_w_cur[0], t_w_cur[1], t_w_cur[2], q_w_roll, q_w_pitch,
                               q_w_yaw);
//            transformTobeMapped[0] = q_w_roll;
//            transformTobeMapped[1] = q_w_pitch;
//            transformTobeMapped[2] = q_w_yaw;
//
//            transformTobeMapped[3] = t_w_cur[0];
//            transformTobeMapped[4] = t_w_cur[1];
//            transformTobeMapped[5] = t_w_cur[2];


//
//        if (lastGnssPreTransAvailable = false) {
//            lastGnssPreTransformation = transBack;
//            lastGnssPreTransAvailable = true;
//        } else {
//            Eigen::Affine3f transIncre =
//                    lastGnssPreTransformation.inverse() * transBack;
//            //将当前激光里程计的状态转变为矩阵形式
//            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
//            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(
                    transBack, transformTobeMapped[3], transformTobeMapped[4],
                    transformTobeMapped[5], transformTobeMapped[0],
                    transformTobeMapped[1], transformTobeMapped[2]);

         updateinit_cost_time = timer.toc();
        //EZLOG(INFO)<<"updateinit_cost_time"<<updateinit_cost_time<<endl;
            return;

        //}
    }

        void extractForLoopClosure() {
//            pcl::PointCloud<PointType>::Ptr cloudToExtract(
//                    new pcl::PointCloud<PointType>());
//            int numPoses = cloudKeyPoses3D->size();
//            for (int i = numPoses - 1; i >= 0; --i) {
//                if (cloudToExtract->size() <= MappingConfig::surroundingKeyframeSize)
//                    cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
//                else
//                    break;
//            }
//            extractCloud(cloudToExtract);
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
        /*
         * 将相邻关键帧集合对应的角点和面点加入到局部map中,作为scan-to-map匹配的局部点云地图
         */

        void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract) {
            //将存储角点和面点的局部地图清空
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
                    *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                    *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
                } else {
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
            TicToc timer;
            if (cloudKeyPoses3D->points.empty() == true) return;

            // if (loopClosureEnableFlag == true)
            // {
            //     extractForLoopClosure();
            // } else {
            //     extractNearby();
            // }

            extractNearby();
             exactsurrounding_key_frames_cost_time = timer.toc();
            //EZLOG(INFO)<<"exactsurrounding_key_frames_cost_time"<<exactsurrounding_key_frames_cost_time<<endl;
        }

        void downsampleCurrentScan() {
            TicToc timer;
            //laserCloudRawDS->clear();
            //对当前帧点云降采样  刚刚完成了周围关键帧的降采样 ,为了使点云稀疏化,加快匹配以及实时性要求
           // downSizeFilterRaw.setInputCloud(laserCloudRaw);
           // downSizeFilterRaw.filter(*laserCloudRawDS);

            // Downsample cloud from current scan
            laserCloudCornerLastDS->clear();
            downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
            downSizeFilterCorner.filter(*laserCloudCornerLastDS);
            laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

            EZLOG(INFO)<<"laserCloudCornerLastDSNum"<<laserCloudCornerLastDSNum<<endl;

            laserCloudSurfLastDS->clear();
            downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
            downSizeFilterSurf.filter(*laserCloudSurfLastDS);
            laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();

            EZLOG(INFO)<<"laserCloudCornerLastDSNum"<<laserCloudSurfLastDSNum<<endl;
             downsample_current_sacn_cost_time = timer.toc();
            //EZLOG(INFO)<<"downsample_current_sacn_cost_time"<<downsample_current_sacn_cost_time<<endl;
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

        /***
         * 1.根据现有地图与最新点云数据进行配准从而更新机器人精确位姿与融合建图，\
           2.分为角点优化、平面点优化、配准与更新等部分。
           3.优化的过程与里程计的计算类似，是通过计算点到直线或平面的距离，构建优化公式再用LM法求解。
         */
        void scan2MapOptimization() {
            TicToc timer;
            EZLOG(INFO) << "current_corner_ds_num: " << laserCloudCornerLastDSNum<<endl;
            EZLOG(INFO) << " current_surf_ds_num: " << laserCloudSurfLastDSNum <<endl;
            if (cloudKeyPoses3D->points.empty()) return;
           //判断当前帧的角点数和面点数是否足够,角点要求10,面点要求100
            if (laserCloudCornerLastDSNum > MappingConfig::edgeFeatureMinValidNum &&
                laserCloudSurfLastDSNum > MappingConfig::surfFeatureMinValidNum) {
               //分别把角点和面点 局部地图构建 kdtree
                kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
                kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

                //迭代求解，迭代30次,里面是手写的优化器了用的LM优化方法
                for (int iterCount = 0; iterCount < 15; iterCount++) {
                    laserCloudOri->clear();
                    coeffSel->clear();

                    cornerOptimization();
                    surfOptimization();

                    combineOptimizationCoeffs();

                    if (LMOptimization(iterCount) == true) break;
                }
                 //把优化后的结果和imu进行一次加权融合；
               scantomap_cost_time = timer.toc();
                //EZLOG(INFO)<<"scantomap_cost_time"<<scantomap_cost_time<<endl;
               // transformUpdate();
            } else {
                ROS_WARN(
                        "Not enough features! Only %d edge and %d planar features available.",
                        laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
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
        //计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
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
            keyframeDistances.push_back(sqrt(x * x + y * y));

            return true;
        }

        void addOdomFactor() {
            TicToc timer;
           // EZLOG(I)
            //对于第一帧关键帧，则将置信度设置差一点。尤其是平移和yaw角
            //将Lidar转到世界坐标系

//            float current_T_w[6];
//             lidar2word(current_T_w_l,current_T_w);

            if (cloudKeyPoses3D->points.empty()) {
                noiseModel::Diagonal::shared_ptr priorNoise =
                        noiseModel::Diagonal::Variances(
                                (Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                        .finished());  // rad*rad, meter*meter
               //增加先验约束 ， 对第 0 个节点增加约束
                gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped),
                                                  priorNoise));
                //加入节点信息 初始值
                initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
            } else {
                //不是第一帧增加帧间约束，明显观察到约束置信度设置较高；
                noiseModel::Diagonal::shared_ptr odometryNoise =
                        noiseModel::Diagonal::Variances(
                                (Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
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
                gtSAMgraph.add(BetweenFactor<Pose3>(
                        cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(),
                        poseFrom.between(poseTo), odometryNoise));
                //加入节点信息 先验位姿
                initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
                mtxGraph.unlock();
                double add_odom_cost_time = timer.toc();
                EZLOG(INFO)<<"add_odom_cost_time"<<add_odom_cost_time<<endl;
            }
        }

        void addGPSFactor() {
            TicToc timer;

            timer.tic();
            if (gpsQueue.empty()) return;

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

                double gps_x = gpsQueue.front().pose.pose.position.x;
                double gps_y = gpsQueue.front().pose.pose.position.y;
                double gps_z = gpsQueue.front().pose.pose.position.z;

//                  double gps_x = t_w_cur[0];
//                  double gps_y = t_w_cur[1];
//                  double gps_z = t_w_cur[2];



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

                if (pointDistance(curGPSPoint, lastGPSPoint) < SensorConfig::gpsDistance)
                    return;
                else
                    lastGPSPoint = curGPSPoint;

                if (SensorConfig::debugGps) {
                    ROS_INFO("curr gps pose: %f, %f , %f", gps_x, gps_y, gps_z);
//                    ROS_INFO("curr gps cov: %f, %f , %f", thisGPS.pose.covariance[0],
//                             thisGPS.pose.covariance[7], thisGPS.pose.covariance[14]);
                }
               //gps 的 置信度，标准差设置成最小1m，也就是不会特别信任gps信息
               /*
                * noise_x noise_y noise_z当作gps在xyz的标准差
                */
               gtsam::Vector Vector3(3);
                Vector3 << noise_x, noise_y, noise_z;
                 Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise =
                        noiseModel::Diagonal::Variances(Vector3);
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
             double add_gps_cost_time = timer.toc();
             EZLOG(INFO)<<add_gps_cost_time<<"add_gps_cost_time"<<endl;
        }

        void addLoopFactor() {
            TicToc timer;
           timer.tic();
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
                        BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
                mtxGraph.unlock();
            }
            //清空回环相关队列
            loopIndexQueue.clear();
            loopPoseQueue.clear();
            loopNoiseQueue.clear();
            aLoopIsClosed = true;
            double add_loop_cost_time = timer.toc();
            EZLOG(INFO)<<"add_loop_cost_time"<<add_loop_cost_time<<endl;
        }

        void saveKeyFramesAndFactor() {
            TicToc timer;
            timer.tic();
            //通过旋转和平移增量来判断是否是关键帧，不是则不会添加factor;
            if (saveFrame() == false) return;

            // odom factor
            addOdomFactor();
            // gps factor
            if (SensorConfig::useGPS)
                addGPSFactor();

            // loop factor
            addLoopFactor();


            // cout << "****************************************************" << endl;
            // gtSAMgraph.print("GTSAM Graph:\n");

            // add raw odom
            //关键帧原始里程计的信息
            nav_msgs::Odometry laserOdometryROS;
            transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS,
                                transformTobeMapped);
            keyframeRawOdom.push_back(laserOdometryROS);

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
            Pose3 latestEstimate;
           //通过接口获得所以变量的状态，获得优化结果
            isamCurrentEstimate = isam->calculateEstimate();
            //取出优化后的最新关键帧位姿
            latestEstimate =
                    isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
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
//
//            EZLOG(INFO)<<"transformTobeMapped roll "<<transformTobeMapped[0]<<endl;
//            EZLOG(INFO)<<"transformTobeMapped pitch "<<transformTobeMapped[1]<<endl;
//            EZLOG(INFO)<<"transformTobeMapped yaw "<<transformTobeMapped[2]<<endl;
//            EZLOG(INFO)<<"transformTobeMapped x "<<transformTobeMapped[3]<<endl;
//            EZLOG(INFO)<<"transformTobeMapped y "<<transformTobeMapped[4]<<endl;
//            EZLOG(INFO)<<"transformTobeMapped z "<<transformTobeMapped[5]<<endl;

            // save all the received edge and surf points
            //当前帧的点云的角点和面点和原始点云分别拷贝一下
            pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr thislaserCloudRawKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
            pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
           // pcl::copyPointCloud(*laserCloudRawDS, *thislaserCloudRawKeyFrame);
           // pcl::copyPointCloud(*laserCloudRaw, *thislaserCloudRawKeyFrame);

            // save key frame cloud
            //保存关键帧的点云
            cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
            surfCloudKeyFrames.push_back(thisSurfKeyFrame);

            //add by fyy
//            std::cout << "start push cloud to map saver" << std::endl;
//            CloudInfo cloud_info;
//            cloud_info.frame_id = ++frame_id;
//            cloud_info.corner_cloud = thisCornerKeyFrame;
//            cloud_info.surf_cloud = thisSurfKeyFrame;
//            map_saver.AddCloudToSave(cloud_info);

            // if you want to save raw cloud
//        laserCloudRawKeyFrames.push_back(thislaserCloudRawKeyFrame);

            keyframeCloudDeskewed.push_back(cloudInfo.cloud_deskewed);
            keyframeTimes.push_back(timeLaserInfoStamp.toSec());


            pcl::PointCloud<PointType>::Ptr globalCornerCloud(
                    new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(
                    new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloud(
                    new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(
                    new pcl::PointCloud<PointType>());

            for (int i = 0; i < (int) cloudKeyPoses3D->size(); i++) {
                *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],
                                                           &cloudKeyPoses6D->points[i]);
                *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i],
                                                         &cloudKeyPoses6D->points[i]);
            }

            downSizeFilterCorner.setInputCloud(globalCornerCloud);
            downSizeFilterCorner.setLeafSize(MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize);
            downSizeFilterCorner.filter(*globalCornerCloudDS);
           // dataSaverPtr->saveCornerClouMap(*globalCornerCloudDS);

            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize, MappingConfig::globalMapLeafSize);
            downSizeFilterSurf.filter(*globalSurfCloudDS);

            std::cout << "start push cloud to map saver" << std::endl;
            CloudInfo cloud_info;
            cloud_info.frame_id = ++frame_id;
            cloud_info.corner_cloud = globalCornerCloudDS;
            cloud_info.surf_cloud = globalSurfCloudDS;
            map_saver.AddCloudToSave(cloud_info);

            addfactors_cost_time = timer.toc();
            //EZLOG(INFO)<<"addfactors_cost_time"<<addfactors_cost_time<<endl;
//            dataSaverPtr->saveSurfClouMap(*globalSurfCloudDS);
//             save global point cloud map
//            *globalMapCloud += *globalCornerCloudDS;
//            *globalMapCloud += *globalSurfCloudDS;
            //根据当前最新位姿更新rviz可视化
            updatePath(thisPose6D);
        }


       //调整全局轨迹
        void correctPoses() {
           TicToc timer;
           timer.tic();
            if (cloudKeyPoses3D->points.empty()) return;
            //只有回环以及gps信息这些会触发全局调整信息才会触发
            if (aLoopIsClosed == true) {
                // clear map cache
                //存放关键帧的位姿和点云的容器清空
                laserCloudMapContainer.clear();
                // clear path
                //清空path
                globalPath.poses.clear();

                // update key poses
                int numPoses = isamCurrentEstimate.size();
                for (int i = 0; i < numPoses; ++i) {
                    //更新所有关键帧的位姿
                    cloudKeyPoses3D->points[i].x =
                            isamCurrentEstimate.at<Pose3>(i).translation().x();
                    cloudKeyPoses3D->points[i].y =
                            isamCurrentEstimate.at<Pose3>(i).translation().y();
                    cloudKeyPoses3D->points[i].z =
                            isamCurrentEstimate.at<Pose3>(i).translation().z();

                    cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                    cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                    cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                    cloudKeyPoses6D->points[i].roll =
                            isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                    cloudKeyPoses6D->points[i].pitch =
                            isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                    cloudKeyPoses6D->points[i].yaw =
                            isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                    //更新path
                    updatePath(cloudKeyPoses6D->points[i]);
                }

                aLoopIsClosed = false;
                 correctposes = timer.toc();
               // EZLOG(INFO)<<"dorrectposes cost time "<< correctposes<<endl;
            }
        }

        //just for show
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

            globalPath.poses.push_back(pose_stamped);
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


         void  transformLidar2World(double timestamp,
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


        //发布优化后的里程计
        void publishOdometry() {
            TicToc timer;
            timer.tic();
            // Publish odometry for ROS (global)
            //发布优化后的里程计
            nav_msgs::Odometry laserOdometryROS;

            Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(transformTobeMapped);
             transformEiegn2Odom(timeLaserInfoCur, laserOdometryROS,
                                         transformTobeMapped);

                    laserOdometryROS.header.stamp = timeLaserInfoStamp;
                    laserOdometryROS.header.frame_id = "map";
                    laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
                    laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
                    laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
//                    laserOdometryROS.pose.pose.orientation.x =q_publish.x();
//                    laserOdometryROS.pose.pose.orientation.y =q_publish.y();
//                    laserOdometryROS.pose.pose.orientation.z =q_publish.z();
//                    laserOdometryROS.pose.pose.orientation.w =q_publish.w();

                    laserOdometryROS.pose.pose.orientation =
                            tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0],
                            transformTobeMapped[1],transformTobeMapped[2]);

            pubLaserOdometryGlobal.publish(laserOdometryROS);

            if (SensorConfig::useGPS) {
                if (gpsTransfromInit && SensorConfig::updateOrigin) {
                    /** we first update the initial GPS origin points since it may not fix here */
                    Eigen::Vector3d origin_point(cloudKeyPoses6D->at(0).x,
                                                 cloudKeyPoses6D->at(0).y,
                                                 cloudKeyPoses6D->at(0).z);
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
                Eigen::Vector3d curr_point(cloudKeyPoses6D->back().x,
                                           cloudKeyPoses6D->back().y,
                                           cloudKeyPoses6D->back().z);
                Eigen::Vector3d curr_lla;
                // ENU->LLA
                geo_converter.Reverse(curr_point[0], curr_point[1], curr_point[2], curr_lla[0], curr_lla[1],
                                      curr_lla[2]);
                //                std::cout << std::setprecision(9)
                //                          << "CURR LLA: " << originLLA.transpose() << std::endl;
                //                std::cout << std::setprecision(9)
                //                          << "update LLA: " << curr_lla.transpose() << std::endl;
                gps_imu::ivsensorgps fix_msgs;
                //sensor_msgs::NavSatFix fix_msgs;

                fix_msgs.header.stamp = ros::Time().fromSec(timeLaserInfoCur);
                fix_msgs.header.frame_id = SensorConfig::odometryFrame;
                fix_msgs.lat = curr_lla[0];
                fix_msgs.lon = curr_lla[1];
                fix_msgs.height = curr_lla[2];
                //pubGPSOdometry.publish(fix_msgs);
                pubGnssOdometry.publish(fix_msgs);
                publish_odometry_cost_time = timer.toc();
               // EZLOG(INFO)<< "publish_odometry_cost_time"<<publish_odometry_cost_time<<endl;
            }
            // Publish odometry for ROS (incremental)
            static bool lastIncreOdomPubFlag = false;
            static nav_msgs::Odometry laserOdomIncremental;  // incremental odometry msg
            static Eigen::Affine3f increOdomAffine;  // incremental odometry in affine
            if (lastIncreOdomPubFlag == false) {
                lastIncreOdomPubFlag = true;
                laserOdomIncremental = laserOdometryROS;
                increOdomAffine = trans2Affine3f(transformTobeMapped);
            } else {
                Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() *
                                              incrementalOdometryAffineBack;
                increOdomAffine = increOdomAffine * affineIncre;
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch,
                                                  yaw);
                //q_incre.normalize();
                laserOdomIncremental.header.stamp = timeLaserInfoStamp;
                laserOdomIncremental.header.frame_id = SensorConfig::odometryFrame;
                laserOdomIncremental.child_frame_id = "odom_mapping";
                laserOdomIncremental.pose.pose.position.x = x;
                laserOdomIncremental.pose.pose.position.y = y;
                laserOdomIncremental.pose.pose.position.z = z;
//                laserOdomIncremental.pose.pose.orientation.x = q_incre.x();
//                laserOdomIncremental.pose.pose.orientation.y = q_incre.y();
//                laserOdomIncremental.pose.pose.orientation.z = q_incre.z();
//                laserOdomIncremental.pose.pose.orientation.w = q_incre.w();
                laserOdomIncremental.pose.pose.orientation =
                        tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
//                if (isDegenerate)
//                    laserOdomIncremental.pose.covariance[0] = 1;
//                else
//                    laserOdomIncremental.pose.covariance[0] = 0;
            }
            pubLaserOdometryIncremental.publish(laserOdomIncremental);
        }

        void publishFrames() {
//            if (cloudKeyPoses3D->points.empty()) return;
//            // publish key poses
//            //发布历史关键帧位姿集合
//            publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp,
//                         SensorConfig::odometryFrame);
//            // Publish surrounding key frames
//            //发布 局部map的将采样平面点集合
//            publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS,
//                         timeLaserInfoStamp, SensorConfig::odometryFrame);
//            // publish registered key frame
//            //发布当前帧的角点、平面点降采样集合
//            if (pubRecentKeyFrame.getNumSubscribers() != 0) {
//                pcl::PointCloud<PointType>::Ptr cloudOut(
//                        new pcl::PointCloud<PointType>());
//                PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
//                *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
//                *cloudOut += *transformPointCloud(laserCloudSurfLastDS, &thisPose6D);
//                publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp,
//                             SensorConfig::odometryFrame);
//            }
//            // publish registered high-res raw cloud
//            //发布当前帧原始点云配准之后的点云
//            if (pubCloudRegisteredRaw.getNumSubscribers() != 0) {
//                pcl::PointCloud<PointType>::Ptr cloudOut(
//                        new pcl::PointCloud<PointType>());
//                pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
//                PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
//                // PointTypePose thisPose6D = trans2PointTypePose(current_T_w_l);
//                *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
//                publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp,
//                             SensorConfig::odometryFrame);
//            }
//            if (pubCloudRaw.getNumSubscribers() != 0) {
//                pcl::PointCloud<PointType>::Ptr cloudOut(
//                        new pcl::PointCloud<PointType>());
//                pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
//                publishCloud(pubCloudRaw, cloudOut, timeLaserInfoStamp, SensorConfig::lidarFrame);
//            }
//
//            // publish path
//            //发布里程计轨迹
//            if (pubPath.getNumSubscribers() != 0) {
//                globalPath.header.stamp = timeLaserInfoStamp;
//                globalPath.header.frame_id = SensorConfig::odometryFrame;
//                pubPath.publish(globalPath);
//            }
            // publish SLAM infomation for 3rd-party usage

//            static int lastSLAMInfoPubSize = -1;
//            if (pubSLAMInfo.getNumSubscribers() != 0) {
//                if (lastSLAMInfoPubSize != cloudKeyPoses6D->size()) {
//                    lio_sam_6axis::cloud_info slamInfo;
//                    slamInfo.header.stamp = timeLaserInfoStamp;
//                    pcl::PointCloud<PointType>::Ptr cloudOut(
//                            new pcl::PointCloud<PointType>());
//                    *cloudOut += *laserCloudCornerLastDS;
//                    *cloudOut += *laserCloudSurfLastDS;
//                    slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut,
//                                                            timeLaserInfoStamp, SensorConfig::lidarFrame);
//                    slamInfo.key_frame_poses =
//                            publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp,
//                                         SensorConfig::odometryFrame);
//                    pcl::PointCloud<PointType>::Ptr localMapOut(
//                            new pcl::PointCloud<PointType>());
//                    *localMapOut += *laserCloudCornerFromMapDS;
//                    *localMapOut += *laserCloudSurfFromMapDS;
//                    slamInfo.key_frame_map = publishCloud(
//                            ros::Publisher(), localMapOut, timeLaserInfoStamp, SensorConfig::odometryFrame);
//                    pubSLAMInfo.publish(slamInfo);
//                    lastSLAMInfoPubSize = cloudKeyPoses6D->size();
//                }
//            }
        }
   };

    int main(int argc, char **argv) {

        el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
        EZLOG(INFO) << "easylogging++ thread safe!";
#else
        EZLOG(INFO) << "easylogging++ thread unsafe";
#endif

        ros::init(argc, argv, "map_opt");

        Load_Sensor_YAML("./config/sensor.yaml");
        Load_Mapping_YAML("./config/mapping.yaml");

        mapOptimization MO;
        EZLOG(INFO) << "Map Optimization Started";

        std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
        //std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
        std::thread savepathThread(&mapOptimization::savePathThread, &MO);
        std::thread saveMapThread(&MapSaver::do_work, &(MO.map_saver));//comment fyy
        //启动每帧点云数据线程
        ros::spin();

        loopthread.join();
        //visualizeMapThread.join();

        return 0;
    }


//
// Created by wxy on 23-9-3.
//
