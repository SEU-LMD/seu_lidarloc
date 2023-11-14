//
// Created by fyy on 23-9-10.
//

#ifndef SEU_LIDARLOC_OPT_LOPC_H
#define SEU_LIDARLOC_OPT_LOPC_H

#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/filter.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/uniform_sampling.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/range_image/range_image.h"
#include "pcl/registration/icp.h"
#include "pcl/conversions.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/ISAM2Params.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"
#include "gtsam/slam/dataset.h"  // gtsam


#include "GeoGraphicLibInclude/Geocentric.hpp"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"
#include "GeoGraphicLibInclude/Geoid.hpp"


#include "map_loader.h"


class LOCMapping{

public:
    enum FRAME {
        LIDAR,    // 0
        WORLD    // 1
    };
    FRAME current_frame = WORLD;
    PubSubInterface* pubsub;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToFuse;
    std::shared_ptr<UDP_THREAD> udp_thread;//udp

    std::thread* do_work_thread;
    std::mutex cloud_mutex;
    MapManager* map_manager_ptr;
    std::deque<CloudFeature> deque_cloud;
    std::string topic_priorMap_surf= "/priorMap_surf";
    std::string topic_priorMap_corner = "/priorMap_corner";
    std::string topic_current_lidarPcl= "/lidar_current_pcl";
    std::string topic_lidar_origin_odometry = "/lidar_origin_odometry";

    pcl::PointCloud<PointType>::Ptr Keyframe_Poses3D;  //历史关键帧 位置
    pcl::PointCloud<PointType>::Ptr current_corner;  // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_surf;  // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr current_corner_ds;  // downsampled corner feature set from
    pcl::PointCloud<PointType>::Ptr current_surf_ds;  // downsampled surf feature set from
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    pcl::PointCloud<PointType>::Ptr localMap_corner;
    pcl::PointCloud<PointType>::Ptr localMap_surf;
    pcl::PointCloud<PointType>::Ptr localMap_corner_ds;
    pcl::PointCloud<PointType>::Ptr localMap_surf_ds;
    pcl::PointCloud<PointType>::Ptr priorMap_corner;// 先验地图角点
    pcl::PointCloud<PointType>::Ptr priorMap_surf; // 先验地图面点
    pcl::PointCloud<PointTypePose>::Ptr Keyframe_Poses6D;  //历史关键帧 位姿
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_surf;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_localMap_corner;
    pcl::UniformSampling<PointType> downSizeFilterCorner_US;
    pcl::UniformSampling<PointType> downSizeFilterSurf_US;

    std::vector<PointType> laserCloudOriCornerVec;  // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<PointType> laserCloudOriSurfVec;  // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;
    std::vector<bool> laserCloudOriCornerFlag;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f T_wl;

    Eigen::Matrix4d T_wb_pub;

    float current_T_m_l[6]; // roll pitch yaw x y z
    double timeLaserInfoCur;

    bool isDegenerate = false;
    cv::Mat matP;

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

    void allocateMemory() {

        Keyframe_Poses3D.reset(new pcl::PointCloud<PointType>());
        Keyframe_Poses6D.reset(new pcl::PointCloud<PointTypePose>());

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

        kdtree_localMap_surf.reset(new pcl::KdTreeFLANN<PointType>);
        kdtree_localMap_corner.reset(new pcl::KdTreeFLANN<PointType>);

        for (int i = 0; i < 6; ++i) {
            current_T_m_l[i] = 0;
        }
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
        downSizeFilterCorner_US.setRadiusSearch(MappingConfig::mappingCornerRadiusSize_US);
        downSizeFilterSurf_US.setRadiusSearch(MappingConfig::mappingSurfRadiusSize_US);
        EZLOG(INFO)<<"opt Loc init success!"<<std::endl;
    }

    void pub_CornerAndSurfFromMap(pcl::PointCloud<PointType>::Ptr &_corner_map, pcl::PointCloud<PointType>::Ptr &_surf_map)
    {
//      PriorMap surf
        CloudTypeXYZI PriorMap_surf_pub;
        PriorMap_surf_pub.frame = "map";
        PriorMap_surf_pub.timestamp = timeLaserInfoCur;
        PriorMap_surf_pub.cloud = *_surf_map;
        pubsub->PublishCloud(topic_priorMap_surf, PriorMap_surf_pub);
        std::cout << "Pub Surf Map!"<<std::endl;

//      PriorMap Corner
        CloudTypeXYZI PriorMap_corner_pub;
        PriorMap_corner_pub.frame = "map";
        PriorMap_corner_pub.timestamp = timeLaserInfoCur;
        PriorMap_corner_pub.cloud = *_corner_map;
        pubsub->PublishCloud(topic_priorMap_corner, PriorMap_corner_pub);
        std::cout << "Pub Corner Map!"<<std::endl;

    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn,
                                                        PointTypePose *transformIn) {

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
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

    void updateInitialGuess(CloudFeature &cur_ft,PointType &_init_point) {

        Eigen::Vector3d t_w_cur;
        Eigen::Quaterniond q_w_cur;
        Eigen::Matrix3d q_w_cur_matrix;
        double q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;

        // use DR
        t_w_cur = cur_ft.DRPose.GetXYZ();
        q_w_cur = cur_ft.DRPose.GetR();
        q_w_cur.normalize();
        q_w_cur_matrix = q_w_cur.toRotationMatrix();
        T_wb_pub.block<3,3>(0, 0) = q_w_cur_matrix;
        T_wb_pub.block<3,1>(0, 3) = t_w_cur;

        // 提取欧拉角（Z-Y-X旋转顺序）q_w_cur_roll,q_w_cur_pitch,q_w_cur_yaw;
        q_w_cur_pitch = asin(-q_w_cur_matrix(2, 0)); // 计算pitch
        if (cos(q_w_cur_pitch) != 0) {
            q_w_cur_roll = atan2(q_w_cur_matrix(2, 1), q_w_cur_matrix(2, 2)); // 计算roll
            q_w_cur_yaw = atan2(q_w_cur_matrix(1, 0), q_w_cur_matrix(0, 0));  // 计算yaw
        } else {
            q_w_cur_roll = 0; // 如果pitch为正90度或负90度，则roll和yaw无法唯一确定
            q_w_cur_yaw = atan2(-q_w_cur_matrix(0, 1), q_w_cur_matrix(1, 1)); // 计算yaw
        }

        // initialization the  frame

        //TODO GNSS init
        current_T_m_l[0] = q_w_cur_roll;
        current_T_m_l[1] = q_w_cur_pitch;
        current_T_m_l[2] = q_w_cur_yaw;
        current_T_m_l[3] = t_w_cur[0];
        current_T_m_l[4] = t_w_cur[1];
        current_T_m_l[5] = t_w_cur[2];

        current_frame = WORLD;
        _init_point.x = t_w_cur.x();
        _init_point.y = t_w_cur.y();
        _init_point.z = t_w_cur.z();
        _init_point.intensity = cur_ft.frame_id;


    }//end funtion updateInitialGuess

/**
 * extract init points from map
 */
    void extractFromPriorMap(const PointType& _init_point,
                             pcl::PointCloud<PointType>::Ptr &_priorMap_corner,
                             pcl::PointCloud<PointType>::Ptr &_priorMap_surf,
                             pcl::PointCloud<PointType>::Ptr &_localMap_corner_ds,
                             pcl::PointCloud<PointType>::Ptr &_localMap_surf_ds){
        int localMap_corner_ds_num = 0;
        int localMap_surf_ds_num = 0;

        //TODO 1030 not used any std::cout, use EZLOG(INFO) << instead-----Done
        EZLOG(INFO) <<"Keyframe_Poses3D->points size : "<< Keyframe_Poses3D->points.size();

        TicToc extract_from_priorMap;

        //TODO 1030 add xiaoqiang map manager interface!!!!!!!!!!---Done
        downSizeFilterCorner_US.setInputCloud(_priorMap_corner);
        downSizeFilterCorner_US.filter(*_localMap_corner_ds);
        downSizeFilterSurf_US.setInputCloud(_priorMap_surf);
        downSizeFilterSurf_US.filter(*_localMap_surf_ds);

        localMap_corner_ds_num = _localMap_corner_ds->size();
        localMap_surf_ds_num = _localMap_surf_ds->size();
        //P_l = (T_wl)^-1 * P_w
//            如果先验地图是世界系的，需要转换成激光雷达系,但是不需要平移向量了
        Eigen::Matrix4f T_matrix_L_B = SensorConfig::T_L_B.cast<float>();
        T_matrix_L_B(0,3) = 0.0;
        T_matrix_L_B(1,3) = 0.0;
        T_matrix_L_B(2,3) = 0.0;
        //TODO ?
        pcl::transformPointCloud(*_localMap_surf_ds, *_localMap_surf_ds, T_matrix_L_B.inverse());
        pcl::transformPointCloud(*_localMap_corner_ds, *_localMap_corner_ds, T_matrix_L_B.inverse());

        //TODO 1030
        EZLOG(INFO) << " localMap_surf_ds_num is: "  << localMap_surf_ds_num
                    << " localMap_corner_ds_num is: "<< localMap_corner_ds_num;
    }//end fucntion extractFromPriorMap

    /**
     * current_corner_ds 当前帧角点降采样
     * current_surf_ds 当前帧面点降采样
     */
    void downsampleCurrentScan( pcl::PointCloud<PointType>::Ptr &_current_corner,
                                pcl::PointCloud<PointType>::Ptr &_current_surf,
                                pcl::PointCloud<PointType>::Ptr &_current_corner_ds,
                                pcl::PointCloud<PointType>::Ptr &_current_surf_ds) {

        _current_corner_ds->clear();
        _current_surf_ds->clear();
        // Downsample cloud from current scan
        downSizeFilterCorner_US.setInputCloud(_current_corner);
        downSizeFilterCorner_US.filter(*_current_corner_ds);
        downSizeFilterSurf_US.setInputCloud(_current_surf);
        downSizeFilterSurf_US.filter(*_current_surf_ds);

        CloudTypeXYZI currentlidar_surf_pub;
        currentlidar_surf_pub.frame = "map";
        currentlidar_surf_pub.timestamp = timeLaserInfoCur;
        pcl::transformPointCloud(*_current_corner_ds, currentlidar_surf_pub.cloud, (T_wb_pub).cast<float>());
        pubsub->PublishCloud(topic_current_lidarPcl, currentlidar_surf_pub);


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

    void cornerOptimization(const pcl::PointCloud<PointType>::Ptr &_current_corner_ds,
                            const pcl::PointCloud<PointType>::Ptr &_localMap_corner_ds) {
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

        for (int i = 0; i < _current_corner_ds->size(); i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = _current_corner_ds->points[i];
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
                    cx += _localMap_corner_ds->points[pointSearchInd[j]].x;
                    cy += _localMap_corner_ds->points[pointSearchInd[j]].y;
                    cz += _localMap_corner_ds->points[pointSearchInd[j]].z;
                }

                cx *= 0.2;
                cy *= 0.2;
                cz *= 0.2;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax =
                            _localMap_corner_ds->points[pointSearchInd[j]].x - cx;
                    float ay =
                            _localMap_corner_ds->points[pointSearchInd[j]].y - cy;
                    float az =
                            _localMap_corner_ds->points[pointSearchInd[j]].z - cz;

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

                    float l12_a = (x1 - x2);
                    float l12_b = (y1 - y2);
                    float l12_c = (z1 - z2);
                    float l12 = sqrt(l12_a * l12_a + l12_b * l12_b +
                                     l12_c * l12_c);

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

                    if (s > 0.10) {
                        laserCloudOriCornerVec[i] = pointOri;// 原始点云
                        coeffSelCornerVec[i] = coeff; // 鲁棒距离，鲁棒向量
                        laserCloudOriCornerFlag[i] = true; // 对应找到的线
                    }
                }
            }
        }
    }

    void surfOptimization(const pcl::PointCloud<PointType>::Ptr &_current_surf_ds,
                          const pcl::PointCloud<PointType>::Ptr &_localMap_surf_ds) {
        switch (current_frame) {
            case WORLD:
                updatePointAssociateToMap();
                break;
            case LIDAR:
                transforPoint2World();
                break;
            default:
                EZLOG(INFO)<<"ERROR! Wrong current_frame Type!!!";
                break;
        }

        for (int i = 0; i < _current_surf_ds->size(); i++) {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = _current_surf_ds->points[i];
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
                    matA0(j, 0) = _localMap_surf_ds->points[pointSearchInd[j]].x;
                    matA0(j, 1) = _localMap_surf_ds->points[pointSearchInd[j]].y;
                    matA0(j, 2) = _localMap_surf_ds->points[pointSearchInd[j]].z;
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
                    if (fabs( pa * _localMap_surf_ds->points[pointSearchInd[j]].x +
                              pb * _localMap_surf_ds->points[pointSearchInd[j]].y +
                              pc * _localMap_surf_ds->points[pointSearchInd[j]].z +
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

                    if (s > 0.10) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs(int _current_corner_ds_num, int _current_surf_ds_num) {
        // combine corner coeffs
//        std::cout<<"combineOptimizationCoeffs()!" << std::endl;
        for (int i = 0; i < _current_corner_ds_num; ++i) {
            if (laserCloudOriCornerFlag[i] == true) {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < _current_surf_ds_num; ++i) {
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
//        std::cout<<"LMOptimization()!" << std::endl;
        // lidar -> camera
        float srx = sin(current_T_m_l[1]);
        float crx = cos(current_T_m_l[1]);
        float sry = sin(current_T_m_l[2]);
        float cry = cos(current_T_m_l[2]);
        float srz = sin(current_T_m_l[0]);
        float crz = cos(current_T_m_l[0]);

        int laserCloudSelNum = laserCloudOri->size();

//        std::cout<<"laserCloudSelNum is : " <<laserCloudSelNum <<std::endl;
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

    void scan2MapOptimization(pcl::PointCloud<PointType>::Ptr &_current_corner_ds,
                              pcl::PointCloud<PointType>::Ptr &_current_surf_ds,
                              pcl::PointCloud<PointType>::Ptr &_localMap_corner_ds,
                              pcl::PointCloud<PointType>::Ptr &_localMap_surf_ds) {
        if (Keyframe_Poses3D->points.empty()) { return; }
        const int maxIters = LocConfig::maxIters;
        if (_current_corner_ds->size() > LocConfig::edgeFeatureMinValidNum &&
            _current_surf_ds->size() > LocConfig::surfFeatureMinValidNum) {
//            add by prior map
            kdtree_localMap_corner->setInputCloud(_localMap_corner_ds);
            kdtree_localMap_surf->setInputCloud(_localMap_surf_ds);
            TicToc optimize_time;
            for (int iterCount = 0; iterCount < maxIters; iterCount++) {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization(_current_corner_ds,_localMap_corner_ds);
                surfOptimization(_current_surf_ds,_localMap_surf_ds);

                combineOptimizationCoeffs(_current_corner_ds->size(),_current_surf_ds->size());

                if(iterCount == SensorConfig::LMOpt_Cnt){
                    EZLOG(INFO)<<"LMOptimization FAILED!!"<<endl;
                    break;
                }

                if (LMOptimization(iterCount) == true) {
                    EZLOG(INFO)<<"LMOptimization Success!!"<<endl;
                    EZLOG(INFO)<<"iterCount: " << iterCount<<std::endl;
                    EZLOG(INFO)<< "optimize_time in ms: "<<optimize_time.toc() <<std::endl;
                    break;
                }

            }

            transformUpdate();
        } else {
            EZLOG(INFO) <<"Not enough features! : ";
        }
    }

//    IMU 初值
    void transformUpdate() {
        std::cout <<"transformUpdate " <<std::endl;
        current_T_m_l[0] =
                constraintTransformation(current_T_m_l[0], LocConfig::rotation_tollerance);
        current_T_m_l[1] =
                constraintTransformation(current_T_m_l[1], LocConfig::rotation_tollerance);
        current_T_m_l[5] =
                constraintTransformation(current_T_m_l[5],  LocConfig::z_tollerance);

    }

    float constraintTransformation(float value, float limit) {
        if (value < -limit) value = -limit;
        if (value > limit) value = limit;

        return value;
    }

    bool WhetherThisFrameIsKeyFrame() {
        if (Keyframe_Poses3D->points.empty()) {
            return true;
        }
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
            sqrt(x * x + y * y + z * z) < MappingConfig::surroundingkeyframeAddingDistThreshold){
            return false;
        }

        std::cout << "distance gap: " << sqrt(x * x + y * y) << std::endl;

        return true;
    }

    void ResultsAndPub2Fuse() {

        OdometryType current_Lidar_pose;
        Eigen::Affine3f Lidarodom_2_map = trans2Affine3f(current_T_m_l);//T_ml
        Eigen::Matrix4d map_2_world = SensorConfig::T_L_B; //T_wm
        Eigen::Matrix4d Lidar_odom_world = map_2_world * Lidarodom_2_map.matrix().cast<double>();// T_wl = T_wm * T_ml

        current_Lidar_pose.frame = "map";
        current_Lidar_pose.timestamp = timeLaserInfoCur;
        current_Lidar_pose.pose.pose = Lidarodom_2_map.matrix().cast<double>();

        Function_AddLidarOdometryTypeToFuse(current_Lidar_pose);
        EZLOG(INFO)<<"4 Loc send to Fuse! lidar result begin: ";
        EZLOG(INFO)<<current_Lidar_pose.pose.pose;
        EZLOG(INFO)<<"lidar result end";

        pubsub->PublishOdometry(topic_lidar_origin_odometry, current_Lidar_pose);

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;

        thisPose3D.x = current_Lidar_pose.pose.GetXYZ().x();
        thisPose3D.y = current_Lidar_pose.pose.GetXYZ().y();
        thisPose3D.z = current_Lidar_pose.pose.GetXYZ().z();
        thisPose3D.intensity = Keyframe_Poses3D->size();  // this can be used as index
        Keyframe_Poses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
        thisPose6D.roll = current_T_m_l[0];
        thisPose6D.pitch = current_T_m_l[1];
        thisPose6D.yaw = current_T_m_l[2];
        thisPose6D.time = timeLaserInfoCur;
        Keyframe_Poses6D->push_back(thisPose6D);

    }

    void DoWork(){

        while(1){
            if(deque_cloud.size()!=0){
                //TODO change position
                CloudFeature cur_ft;
                PointType init_point;
                cloud_mutex.lock();
                cur_ft = deque_cloud.back(); //TODO 1030---Done
                deque_cloud.clear();//TODO 1030---Done
                cloud_mutex.unlock();

                timeLaserInfoCur = cur_ft.timestamp;
                current_surf = cur_ft.surfaceCloud;
                current_corner = cur_ft.cornerCloud;

                static double timeLastProcessing = -1;
                if(timeLaserInfoCur - timeLastProcessing < LocConfig::mappingProcessInterval){
                    continue;
                }
                timeLastProcessing = timeLaserInfoCur;
                EZLOG(INFO) <<"Recive current_surf size: "<<current_surf->size()
                            <<" current_corner size: "<<current_corner->size();
                //1. init pose
                updateInitialGuess(cur_ft,init_point);

                //2. Keyframe //TOOD 1030 add keyframe selection strategy!!!------Done
                if (WhetherThisFrameIsKeyFrame() == false) {
                    EZLOG(INFO)<<"Loc: this Frame is not KeyFrame, just Drop!";
                    continue;
                }

                TicToc t_2;
                //3.load PriorMap
                map_manager_ptr->SafeLockCloud();
                map_manager_ptr->GetCurMapCloud(priorMap_corner,priorMap_surf);
                EZLOG(INFO)<<"GetCurMapCloud() time : "<<t_2.toc();
                {
                    pub_CornerAndSurfFromMap(priorMap_corner,priorMap_surf); // debug use
                }

                //4.downSample current scan TODO1111
                TicToc t_3;
                downsampleCurrentScan(current_corner,current_surf,current_corner_ds,current_surf_ds);
                EZLOG(INFO)<<"downsampleCurrentScan() time : "<<t_3.toc();

                //5.extract prior map
                TicToc t_7;
                extractFromPriorMap(init_point,priorMap_corner,priorMap_surf,localMap_corner_ds,localMap_surf_ds);
                EZLOG(INFO)<<"extractFromPriorMap() time : "<<t_7.toc();

                //6.scan 2 map
                TicToc t_4;
                scan2MapOptimization(current_corner,current_surf,localMap_corner_ds,localMap_surf_ds);
                EZLOG(INFO)<<"scan2MapOptimization() time : "<<t_4.toc();
                map_manager_ptr->SafeUnlockCloud();//very important fucniton to protect map manager memory!!!!!!!

                TicToc t_5;
                ResultsAndPub2Fuse();//TODO 1030 change function name------Done
                EZLOG(INFO)<<"ResultsAndPub2Fuse() time : "<<t_5.toc();

            }
            else{
                sleep(0.01);
            }
        }
    }//end fucntion DoWork

    /**
* add by sy for load Prior map Corner
* @param mapFile
*/
    void Init(PubSubInterface* pubsub_, MapManager* map_manager_ptr_, std::shared_ptr<UDP_THREAD> udp_thread_){
        pubsub = pubsub_;
        map_manager_ptr = map_manager_ptr_;
        udp_thread = udp_thread_;
        allocateMemory();
        pubsub->addPublisher(topic_priorMap_corner, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_priorMap_surf, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_current_lidarPcl, DataType::LIDAR, 10);
        pubsub->addPublisher(topic_lidar_origin_odometry, DataType::ODOMETRY, 10);
        do_work_thread = new std::thread(&LOCMapping::DoWork, this);
    }
};
#endif //SEU_LIDARLOC_OPT_LOPC_H
