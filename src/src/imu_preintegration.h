
// Use the Velodyne point format as a common representation
#ifndef SEU_LIDARLOC_IMUPREINTEGRATION_H
#define SEU_LIDARLOC_IMUPREINTEGRATION_H
#include <mutex>
#include <thread>
#include <iostream>
#include "gtsam/geometry/Pose3.h"
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

#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "featureExtraction.h"
#include "utils/MapSaver.h"
#include "utils/timer.h"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz) /Pose3(x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class IMUPreintegration  {
public:
    PubSubInterface* pubsub;
    std::mutex odom_mutex;
    std::mutex gnssins_mutex;
    std::mutex lidarodom_mutex;

    std::thread* do_lidar_thread;

//    std::deque<OdometryTypePtr> lidar_poseQueue;
    std::deque<OdometryType> lidar_poseQueue;
    std::deque<GNSSINSType> deque_gnssins;

    std::string topic_imu_raw_odom = "/imu_odom_raw";

    bool systemInitialized = false;
    // 噪声协方差
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<IMURawDataPtr> imuQueOpt;
//    std::deque<IMURawDataPtr> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(-SensorConfig::extrinsicTrans.x(), -SensorConfig::extrinsicTrans.y(), -SensorConfig::extrinsicTrans.z()));
    gtsam::Pose3 lidar2Imu =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(SensorConfig::extrinsicTrans.x(), SensorConfig::extrinsicTrans.y(), SensorConfig::extrinsicTrans.z()));

    void AddOdomData(const OdometryType &data){

//        OdometryTypePtr odom_ptr(new OdometryType());
//        *odom_ptr = data;//深拷贝
        lidarodom_mutex.lock();
        lidar_poseQueue.push_back(data);
//        EZLOG(INFO)<<"lidar_poseQueue.size() "<<lidar_poseQueue.size()<<std::endl;
        lidarodom_mutex.unlock();

    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){
          TicToc timer;
//        gnssins_mutex.lock();
//        deque_gnssins.push_back(gnss_ins_data);
//        EZLOG(INFO)<<"deque_gnssins.size() =  "<<deque_gnssins.size()<<std::endl;
//        gnssins_mutex.unlock();
//        GNSSINSType gnss_ins_data;
//        odom_mutex.lock();
//        gnss_ins_data = deque_gnssins.front();
//        deque_gnssins.pop_front();
//        odom_mutex.unlock();

        IMURawDataPtr imu_raw (new IMURawData);
        imu_raw->imu_angular_v = gnss_ins_data.imu_angular_v * 3.1415926535 / 180.0; //转弧度值
        imu_raw->imu_linear_v = gnss_ins_data.imu_linear_acc;
        imu_raw->timestamp = gnss_ins_data.timestamp;
        Eigen::Quaterniond orientation_quaternion;
        orientation_quaternion = Eigen::AngleAxisd(gnss_ins_data.roll * 3.1415926535 / 180.0, Eigen::Vector3d::UnitX())
                                 *Eigen::AngleAxisd(gnss_ins_data.pitch * 3.1415926535 / 180.0, Eigen::Vector3d::UnitY())
                                 *Eigen::AngleAxisd(gnss_ins_data.yaw * 3.1415926535 / 180.0, Eigen::Vector3d::UnitZ());
        imu_raw->orientation = orientation_quaternion;

        double imuTime = imu_raw->timestamp;

        gnssins_mutex.lock();
        imuQueOpt.push_back(imu_raw);
        gnssins_mutex.unlock();

//        EZLOG(INFO) << "imuQueOpt size: "<<imuQueOpt.size() << " imuQueImu size: "<<imuQueImu.size()
//                    <<" doneFirstOpt: "<<doneFirstOpt;

        if (doneFirstOpt == false) return;

        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // imu预积分器添加一帧imu数据，注：这个预积分器的起始时刻是上一帧激光里程计时刻
        //insert measurement data into imu_pre
        imuIntegratorImu_->integrateMeasurement(imu_raw->imu_linear_v,
                                                imu_raw->imu_angular_v,
                                                dt);
        // predict odometry
        // 用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);//both are input parameters

        // publish odometry
        //for debug use
        {
            // transform imu pose to ldiar
            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Pose3 lidar_preditct_pose_gtsam = imuPose.compose(imu2Lidar);
            PoseT lidar_preditct_pose(lidar_preditct_pose_gtsam.translation().vector(),
                                      lidar_preditct_pose_gtsam.rotation().matrix());

            OdometryType Odometry_imuPredict_pub;
            Odometry_imuPredict_pub.timestamp = imu_raw->timestamp;
            Odometry_imuPredict_pub.frame = "map";
            Odometry_imuPredict_pub.pose = lidar_preditct_pose;
            pubsub->PublishOdometry(topic_imu_raw_odom, Odometry_imuPredict_pub);
        }
        EZLOG(INFO)<<"imu_pre addgnssdata cost time(ms) = "<<timer.toc()<<std::endl;
    }

    //
    void SetIMUPreParamter(){

        boost::shared_ptr<gtsam::PreintegrationParams> p =  gtsam::PreintegrationParams::MakeSharedU(SensorConfig::imuGravity);
        p->accelerometerCovariance =  gtsam::Matrix33::Identity(3, 3) *  pow(SensorConfig::imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance =      gtsam::Matrix33::Identity(3, 3) *  pow(SensorConfig::imuGyrNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance =    gtsam::Matrix33::Identity(3, 3) *  pow(1e-4,2);  // error committed in integrating position from velocities
        //TODO!!!!!!!!
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());  // assume zero initial bias

        //used for predict
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements( p,prior_imu_bias);  // setting up the IMU integration for IMU message
        //used for opt bias
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements( p, prior_imu_bias);  // setting up the IMU integration for optimization

        //TODO!!! what finihsed use for???
        //below parameters are used for opt bias
        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1) .finished());  // rad,rad,rad,m, m, m
        //TODO correct!!!!!
        noiseModelBetweenBias = (gtsam::Vector(6) << SensorConfig::imuAccBiasN,
                                                        SensorConfig::imuAccBiasN,
                                                        SensorConfig::imuAccBiasN,
                                                        SensorConfig::imuGyrBiasN,
                                                        SensorConfig::imuGyrBiasN,
                                                        SensorConfig::imuGyrBiasN).finished();

    }



    void ClearFactorGraph() {
//        gtsam::ISAM2Params optParameters;
//        optParameters.relinearizeThreshold = 0.1;
//        optParameters.relinearizeSkip = 1;
//        optimizer = gtsam::ISAM2(optParameters);

//        gtsam::NonlinearFactorGraph newGraphFactors;
//        graphFactors = newGraphFactors;
        graphFactors.resize(0);
//        gtsam::Values NewGraphValues;
//        graphValues = NewGraphValues;
        graphValues.clear();
    }

    //
    bool failureDetection(const gtsam::Vector3 &velCur,
                          const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
//            EZLOG(INFO)<<"Large velocity, reset IMU-preintegration!";
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
//            EZLOG(INFO) <<"Large bias, reset IMU-preintegration!";
            return true;
        }

        return false;
    }

    void resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void DoLidar(){
//
        while(1){
            if(lidar_poseQueue.size()!=0){

                EZLOG(INFO)<< "DoLidar!! "<<std::endl;

                OdometryType cur_lidar_odom;
                lidarodom_mutex.lock();
                cur_lidar_odom = lidar_poseQueue.front();
                lidar_poseQueue.pop_front();
                lidarodom_mutex.unlock();

                double currentCorrectionTime = cur_lidar_odom.timestamp;
                // make sure we have imu data to integrate
                if (imuQueOpt.empty())
                {
                    continue;
                }

                bool degenerate = 0;
                gtsam::Pose3 lidarPose =
                        gtsam::Pose3(gtsam::Rot3::Quaternion(cur_lidar_odom.pose.GetQ().w(),
                                                                 cur_lidar_odom.pose.GetQ().x(),
                                                                 cur_lidar_odom.pose.GetQ().y(),
                                                                 cur_lidar_odom.pose.GetQ().z()),
                                     gtsam::Point3(cur_lidar_odom.pose.GetXYZ()));

//                EZLOG(INFO)<< "lidarPose: "<< lidarPose.matrix();

                // 0. initialize system
                if (systemInitialized == false) {
                    ClearFactorGraph();
                    // pop old IMU message
                    gnssins_mutex.lock();
                    while (!imuQueOpt.empty()) {
//                        EZLOG(INFO)<<"imuQueOpt.size()"<<imuQueOpt.size()<<std::endl;
                        if (imuQueOpt.front()->timestamp < currentCorrectionTime - delta_t) {
//                            EZLOG(INFO)<<"imuQueOpt.front()->timestamp - currentCorrectionTime = "<<imuQueOpt.front()->timestamp - currentCorrectionTime<<std::endl;
                            lastImuT_opt = imuQueOpt.front()->timestamp;
                            imuQueOpt.pop_front();
//                            EZLOG(INFO)<<"imuQueOpt.size()"<<imuQueOpt.size()<<std::endl;
                        } else
                            break;
                    }
                    gnssins_mutex.unlock();

                    // initial pose
                    prevPose_ = lidarPose.compose(lidar2Imu);
                    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,priorPoseNoise);
                    graphFactors.add(priorPose);
                    // initial velocity
                    prevVel_ = gtsam::Vector3(0, 0, 0);
                    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                    graphFactors.add(priorVel);
                    // initial bias
                    prevBias_ = gtsam::imuBias::ConstantBias();
                    EZLOG(INFO)<<""<<std::endl;
                    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                    graphFactors.add(priorBias);
                    // add values
                    graphValues.insert(X(0), prevPose_);
                    graphValues.insert(V(0), prevVel_);
                    graphValues.insert(B(0), prevBias_);
                    // optimize once
                    optimizer.update(graphFactors, graphValues);
                    graphFactors.resize(0);
                    graphValues.clear();

                    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
                    imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
                     EZLOG(INFO)<<""<<std::endl;
                    key = 1;
                    systemInitialized = true;
                    continue;
                }
                // reset graph for speed
                if (key == 50) {
                    // get updated noise before reset
                    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise =
                            gtsam::noiseModel::Gaussian::Covariance(
                                    optimizer.marginalCovariance(X(key - 1)));
                    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise =
                            gtsam::noiseModel::Gaussian::Covariance(
                                    optimizer.marginalCovariance(V(key - 1)));
                    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise =
                            gtsam::noiseModel::Gaussian::Covariance(
                                    optimizer.marginalCovariance(B(key - 1)));
                    // reset graph
                    ClearFactorGraph();
                    // add pose
                    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                               updatedPoseNoise);
                    graphFactors.add(priorPose);
                    // add velocity
                    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                                updatedVelNoise);
                    graphFactors.add(priorVel);
                    // add bias
                    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                            B(0), prevBias_, updatedBiasNoise);
                    graphFactors.add(priorBias);
                    // add values
                    graphValues.insert(X(0), prevPose_);
                    graphValues.insert(V(0), prevVel_);
                    graphValues.insert(B(0), prevBias_);
                    // optimize once
                    optimizer.update(graphFactors, graphValues);
                    graphFactors.resize(0);
                    graphValues.clear();

                    key = 1;
                }
                // 1. integrate imu data and optimize
                while (!imuQueOpt.empty()) {
                    // pop and integrate imu data that is between two optimizations
                    IMURawData thisImu = *imuQueOpt.front();
                    double imuTime = thisImu.timestamp;
                    // std::cout << " delta_t: " << imuTime -lastImuT_opt << std::endl;
                    if (imuTime < currentCorrectionTime - delta_t) {
                        double dt =
                                (lastImuT_opt < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_opt);
                        //        double dt = (lastImuT_opt < 0) ? (1.0 / imuFrequence) :
                        //        (imuTime - lastImuT_opt);
                        imuIntegratorOpt_->integrateMeasurement(
                                gtsam::Vector3(thisImu.imu_linear_v),
                                gtsam::Vector3(thisImu.imu_angular_v),
                                dt);
                        lastImuT_opt = imuTime;
                        imuQueOpt.pop_front();
                    } else
                        break;
                }
                // add imu factor to graph
                const gtsam::PreintegratedImuMeasurements &preint_imu =
                        dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(
                                *imuIntegratorOpt_);
                gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key),
                                            B(key - 1), preint_imu);
                graphFactors.add(imu_factor);
                // add imu bias between factor
                graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                        B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                        gtsam::noiseModel::Diagonal::Sigmas(
                                sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
                // add pose factor
                gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
                gtsam::PriorFactor<gtsam::Pose3> pose_factor(
                        X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
                graphFactors.add(pose_factor);
                // insert predicted values
                gtsam::NavState propState_ =
                        imuIntegratorOpt_->predict(prevState_, prevBias_);
                graphValues.insert(X(key), propState_.pose());
                graphValues.insert(V(key), propState_.v());
                graphValues.insert(B(key), prevBias_);
                // optimize
                optimizer.update(graphFactors, graphValues);
                optimizer.update();
                graphFactors.resize(0);
                graphValues.clear();
                // Overwrite the beginning of the preintegration for the next step.
                gtsam::Values result = optimizer.calculateEstimate();
                prevPose_ = result.at<gtsam::Pose3>(X(key));
                prevVel_ = result.at<gtsam::Vector3>(V(key));
                prevState_ = gtsam::NavState(prevPose_, prevVel_);
                prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
                // Reset the optimization preintegration object.
                imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
                // check optimization
                if (failureDetection(prevVel_, prevBias_)) {
                    EZLOG(INFO)<<"failureDetection triggered"<<std::endl;
                    resetParams();
                    continue;
                }

                // 2. after optiization, re-propagate imu odometry preintegration
                prevStateOdom = prevState_;
                prevBiasOdom = prevBias_;
                // first pop imu message older than current correction data
                double lastImuQT = -1;
//                while (!imuQueImu.empty() &&
//                       *(&imuQueImu.front()->timestamp) < currentCorrectionTime - delta_t) {
//                    lastImuQT = *(&imuQueImu.front()->timestamp);
//                    imuQueImu.pop_front();
//                }
                lastImuQT = lastImuT_opt;
                // repropogate
                if (!imuQueOpt.empty()) {
                    // reset bias use the newly optimized bias
                    imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
                    // integrate imu message from the beginning of this optimization
                    for (int i = 0; i < (int) imuQueOpt.size(); ++i) {
                        IMURawData thisImu = *imuQueOpt[i];
                        double imuTime = thisImu.timestamp;
                        double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                        //        double dt = (lastImuQT < 0) ? (1.0 / imuFrequence) : (imuTime
                        //        - lastImuQT);

                        imuIntegratorImu_->integrateMeasurement(
                                gtsam::Vector3(thisImu.imu_linear_v),
                                gtsam::Vector3(thisImu.imu_angular_v),
                                dt);
                        lastImuQT = imuTime;
                    }
                }

                ++key;
                doneFirstOpt = true;
            }//end if(deque_cloud.size()!=0){
            else{
                sleep(0.005);
            }
        }
    }//end function DoLidar

    void Init(PubSubInterface* pubsub_){

        //设置imu的参数
        SetIMUPreParamter();
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);
        pubsub = pubsub_;
        pubsub->addPublisher(topic_imu_raw_odom, DataType::ODOMETRY, 10);
        do_lidar_thread = new std::thread(&IMUPreintegration::DoLidar, this);
    }
};

#endif //SEU_LIDARLOC_IMUPREINTEGRATION_H

