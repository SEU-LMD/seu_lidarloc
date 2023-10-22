
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

class IMU_DR  {
public:
    PubSubInterface* pubsub;
    std::mutex odom_mutex;
    std::mutex gnssins_mutex;
    std::mutex lidarodom_mutex;
    std::function<void(const OdometryType&)> Function_AddOdometryTypeToImageProjection;
    std::function<void(const IMUOdometryType&)> Function_AddIMUOdometryTypeToFuse;
    std::ofstream outfile;

    std::thread* do_lidar_thread;

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

    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<IMURawDataPtr> imuQueOpt;
    std::deque<IMURawDataPtr> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prior_imu_bias;

    double lastImuT_imu = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    int key = 1;

    gtsam::Pose3 imu2Lidar =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(-SensorConfig::extrinsicTrans.x(), -SensorConfig::extrinsicTrans.y(), -SensorConfig::extrinsicTrans.z()));
    gtsam::Pose3 lidar2Imu =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(SensorConfig::extrinsicTrans.x(), SensorConfig::extrinsicTrans.y(), SensorConfig::extrinsicTrans.z()));
    OdometryType first_lidar_odom;
    gtsam::NavState firstLidarPose;
    gtsam::NavState currentState;
    IMURawDataPtr cur_imu;

    Eigen::Matrix3d rotVecToMat(const Eigen::Vector3d &revc){
        return Eigen::AngleAxisd(revc.norm(),revc.normalized()).toRotationMatrix();
    }

    Eigen::Matrix3d deltaRotMat(const Eigen::Vector3d &delta_rot_vec, int flag ) {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();

        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.vec() = 0.5 * delta_rot_vec;
        delta_R = delta_q.toRotationMatrix();

        return delta_R;
    }

    Eigen::Matrix3d rotationUpdate(const Eigen::Matrix3d &Rwi,
                                          const Eigen::Matrix3d &delta_rot_mat) {
        Eigen::Matrix3d updatedR = Eigen::Matrix3d::Identity();
        updatedR = Rwi * delta_rot_mat;
        //updatedR = delta_rot_mat * Rwi;
        return updatedR;
    }

    Eigen::Matrix3d skewMatrix(const Eigen::Vector3d &v){
        Eigen::Matrix3d w;
        w << 0. ,-v(2), v(1), v(2) , 0. , -v(1) , v(0), 0.;
        return w;
    }

    void AddOdomData(const OdometryType &data){

//      not use lidar
        lidarodom_mutex.lock();
        if(lidar_poseQueue.empty()){
            lidar_poseQueue.push_back(data);
            EZLOG(INFO)<<"lidar_poseQueue.size() "<<lidar_poseQueue.size()<<std::endl;
            first_lidar_odom = lidar_poseQueue.front();
//            firstLidarPose.pose();
            lidarodom_mutex.unlock();
        }
        lidarodom_mutex.unlock();
    }
    gtsam::NavState IMUWheel_predict(IMURawWheelDataPtr curr_imu){
        double imuTime = _imuWheel_raw->timestamp;
        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
        StateData state_;
        state_.timestamp = curr_imu->timestamp;

        const Eigen::Vector3d gyr_unbias =  curr_imu->imu_angular_v;  // gyro not unbias for now
        const auto &dR = deltaRotMat(gyr_unbias * dt);
        state_.Rwi_ = rotationUpdate(state_.Rwi_ , dR);
        state_.v_wi_ = (0, curr_imu->velocity,0);
        state_.p_wi_ = last_state->p_wi_ + (last_state->v_wi_ + state_->v_wi_)/2 *dt;//position

//        double imuTime = _imuWheel_raw->timestamp;
//        StateData last_state = state_;
//        state_.timestamp = curr_imu->timestamp;
//        const double dt = curr_imu->timestamp - last_imu->timestamp;
//
//        const Eigen::Vector3d gyr_unbias =  0.5*(last_imu->imu_angular_v + curr_imu->imu_angular_v);  // gyro not unbias for now
//        const auto &dR = deltaRotMat(gyr_unbias * dt);
//        state_.Rwi_ = rotationUpdate(last_state.Rwi_ , dR);
//        state_.v_wi_ = (0, curr_imu->velocity,0);
//        state_.p_wi_ = last_state->p_wi_ + (last_state->v_wi_ + state_->v_wi_)/2 *dt;//position

    }
    gtsam::NavState IMU_predict(IMURawDataPtr _imu_raw){
        double imuTime = _imu_raw->timestamp;
        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
        //        double dt = (lastImuQT < 0) ? (1.0 / imuFrequence) : (imuTime
        //        - lastImuQT);
        _imu_raw->imu_angular_v *= SensorConfig::imu_angular_v_gain;
//                        EZLOG(INFO)<<"dt: "<<dt; dt: 0.0199661 dt: 0.00914884
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(_imu_raw->imu_linear_acc),
                gtsam::Vector3(_imu_raw->imu_angular_v),
                dt);
        lastImuT_imu = imuTime;

        EZLOG(INFO)<<"firstLidarPose: "<<firstLidarPose;
        gtsam::NavState _currentState = imuIntegratorImu_->predict(firstLidarPose, prior_imu_bias);//both are input parameters
        EZLOG(INFO) <<" imu_raw->imu_linear_acc: "<<_imu_raw->imu_linear_acc.transpose()
                    <<" imu_raw->imu_angular_v: "<<_imu_raw->imu_angular_v.transpose()
                    <<" dt: "<< dt
                    <<" prevBiasOdom: "<<prior_imu_bias;
        EZLOG(INFO)<<" currentState: "<<_currentState;

        return _currentState;
    }
    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){

        TicToc time1;
        double currentTime = gnss_ins_data.timestamp;
//        static int imu_cnt = 0;
//          for debug
//        outfile.precision(6);
//        outfile << imu_raw->imu_angular_v.x()<<" "<< imu_raw->imu_angular_v.y()<<" "<<imu_raw->imu_angular_v.z()<<std::endl;

//        gnssins_mutex.lock();
//        imuQueImu.push_back(imu_raw);
//        gnssins_mutex.unlock();
        if(SensorConfig::if_use_Wheel_DR == 1){
            IMURawWheelDataPtr imuWheel_raw (new IMURawWheelData);
            imuWheel_raw->imu_angular_v = gnss_ins_data.imu_angular_v * 3.1415926535 / 180.0; //转弧度值
            imuWheel_raw->imu_linear_acc = gnss_ins_data.imu_linear_acc;
            imuWheel_raw->timestamp = gnss_ins_data.timestamp;
            imuWheel_raw->velocity = gnss_ins_data.velocity;
            currentState = IMUWheel_predict(imuWheel_raw);
        }
        else{
            IMURawDataPtr imu_raw (new IMURawData);
            imu_raw->imu_angular_v = gnss_ins_data.imu_angular_v * 3.1415926535 / 180.0; //转弧度值
            imu_raw->imu_linear_acc = gnss_ins_data.imu_linear_acc;
            imu_raw->timestamp = gnss_ins_data.timestamp;
            currentState = IMU_predict(imu_raw);
        }

        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
//        gtsam::Pose3 lidar_preditct_pose_gtsam = imuPose.compose(lidar2Imu);

        PoseT lidar_preditct_pose(imuPose.translation().vector(),
                                  imuPose.rotation().matrix());

        //          for debug
        OdometryType Odometry_imuPredict_pub;
        Odometry_imuPredict_pub.timestamp = currentTime;
        Odometry_imuPredict_pub.frame = "map";
        Odometry_imuPredict_pub.pose = lidar_preditct_pose;
        //for debug use
        pubsub->PublishOdometry(topic_imu_raw_odom, Odometry_imuPredict_pub);
        EZLOG(INFO)<<" time in ms: "<<time1.toc();
//        imu_cnt++;
//        EZLOG(INFO)<<"imu_cnt: "<<imu_cnt;

    }

    //
    void SetIMUPreParamter(){

        boost::shared_ptr<gtsam::PreintegrationParams> p =  gtsam::PreintegrationParams::MakeSharedU(SensorConfig::imuGravity);
        // Realistic MEMS white noise characteristics. Angular and velocity random walk
        // expressed in degrees respectively m/s per sqrt(hr). 弧度制,米
//   example:
//          kGyroSigma = radians(0.5) / 60;     // 0.5 degree ARW,
//          kAccelSigma = 0.1 / 60;             // 10 cm VRW
        p->accelerometerCovariance =  gtsam::Matrix33::Identity(3, 3) *  pow(SensorConfig::imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance =      gtsam::Matrix33::Identity(3, 3) *  pow(SensorConfig::imuGyrNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance =    gtsam::Matrix33::Identity(3, 3) *  pow(1e-4,2);  // error committed in integrating position from velocities
//        EZLOG(INFO)<<"p->getGravity(): "<<p->getGravity();
        //TODO!!!!!!!!
        ///why not imuAccBiasN and imuGyrBiasN???
        prior_imu_bias = gtsam::imuBias::ConstantBias((gtsam::Vector(6) <<
                                        SensorConfig::imuConstBias_acc,
                                        SensorConfig::imuConstBias_acc,
                                        SensorConfig::imuConstBias_acc,
                                        SensorConfig::imuConstBias_gyro,
                                        SensorConfig::imuConstBias_gyro,
                                        SensorConfig::imuConstBias_gyro).finished());  // assume zero initial bias

        //used for predict
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements( p,prior_imu_bias);  // setting up the IMU integration for IMU message

        lastImuT_imu = -1;
        gtsam::Pose3 firstLidarPose_pose3 =
                gtsam::Pose3(gtsam::Rot3(  0,-1,0,
                                                 1,0,0,
                                                 0,0,1),
                             gtsam::Point3(0, 0, 0));
        gtsam::Vector3 firstLidarVel =
                gtsam::Vector3(0,0,0);
        firstLidarPose = gtsam::NavState(firstLidarPose_pose3,firstLidarVel);
    }

    void DoPredict(){
        while(1){
            if(!imuQueImu.empty()){
//                TicToc time1;
//                gnssins_mutex.lock();
//                IMURawDataPtr thisImu = imuQueImu.front();
//                imuQueImu.pop_front();
//                EZLOG(INFO)<<"imuQueImu size: "<<imuQueImu.size();
//                gnssins_mutex.unlock();
//                double imuTime = thisImu->timestamp;
//                double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
//                //        double dt = (lastImuQT < 0) ? (1.0 / imuFrequence) : (imuTime
//                //        - lastImuQT);
//                thisImu->imu_angular_v *= SensorConfig::imu_angular_v_gain;
////                        EZLOG(INFO)<<"dt: "<<dt; dt: 0.0199661 dt: 0.00914884
//                imuIntegratorImu_->integrateMeasurement(
//                        gtsam::Vector3(thisImu->imu_linear_acc),
//                        gtsam::Vector3(thisImu->imu_angular_v),
//                        dt);
//                lastImuT_imu = imuTime;
////                 predict odometry
////                         用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
//                EZLOG(INFO)<<"firstLidarPose: "<<firstLidarPose;
//                gtsam::NavState currentState = imuIntegratorImu_->predict(firstLidarPose, prior_imu_bias);//both are input parameters
//                EZLOG(INFO) <<" imu_raw->imu_linear_acc: "<<thisImu->imu_linear_acc.transpose()
//                            <<" imu_raw->imu_angular_v: "<<thisImu->imu_angular_v.transpose()
//                            <<" dt: "<< dt
//                            <<" prevBiasOdom: "<<prior_imu_bias;
//                EZLOG(INFO)<<" currentState: "<<currentState;
//                EZLOG(INFO)<<" time in ms: "<<time1.toc();
//
////                 publish odometry
////                 transform imu pose to ldiar
//                gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
////        gtsam::Pose3 lidar_preditct_pose_gtsam = imuPose.compose(lidar2Imu);
//
//                PoseT lidar_preditct_pose(imuPose.translation().vector(),
//                                          imuPose.rotation().matrix());
//
//                OdometryType Odometry_imuPredict_pub;
//                Odometry_imuPredict_pub.timestamp = thisImu->timestamp;
//                Odometry_imuPredict_pub.frame = "map";
//                Odometry_imuPredict_pub.pose = lidar_preditct_pose;
//                //for debug use
//                pubsub->PublishOdometry(topic_imu_raw_odom, Odometry_imuPredict_pub);
            }//end if(imuQueImu.size()!=0){
            else{
                sleep(0.001);
            }
        }
    }//end function DoLidar

    void Init(PubSubInterface* pubsub_){

        //设置imu的参数
        SetIMUPreParamter();
        //debug
        outfile.open("imu_gyro.txt",std::ios::app);
        if (!outfile.is_open()) {
            std::cerr << "无法打开输出文件 " << std::endl;
        }
        outfile << "gyro.x "<<"gyro.y "<<"gyro.z"<<std::endl;

        pubsub = pubsub_;
        pubsub->addPublisher(topic_imu_raw_odom, DataType::ODOMETRY, 10);
        do_lidar_thread = new std::thread(&IMU_DR::DoPredict, this);
    }
};

#endif //SEU_LIDARLOC_IMUPREINTEGRATION_H

