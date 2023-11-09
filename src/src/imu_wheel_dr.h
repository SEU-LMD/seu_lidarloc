
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
#include "utils/MapSaver.h"
#include "utils/timer.h"

//#include "udp_seralize.h"
#include "utils/udp_thread.h"
#include "GeoGraphicLibInclude/LocalCartesian.hpp"

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz) /Pose3(x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

class imu_wheel_dr  {
public:
    PubSubInterface* pubsub;
    std::mutex odom_mutex;
    std::mutex gnssins_mutex;
    std::mutex lidarodom_mutex;
    std::function<void(const OdometryType&)> Function_AddDROdometryTypeToImageProjection;
    std::function<void(const DROdometryType&)> Function_AddDROdometryTypeToFuse;
    GeographicLib::LocalCartesian geoConverter;
    std::ofstream outfile;

    std::thread* do_lidar_thread;

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
    Eigen::Vector3d prior_acc_bias;
    Eigen::Vector3d prior_gyro_bias;

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
    gtsam::NavState currentState,lastState;
    StateData state,last_state;
    Eigen::Matrix3d R_w_b;
    double current_V;
    IMURawDataPtr cur_imu;
    gtsam::NonlinearFactorGraph* graph;

    std::shared_ptr<UDP_THREAD> udp_thread;
    bool flag_firstGNSSPoint = 0;


    Eigen::Matrix3d rotVecToMat(const Eigen::Vector3d &rvec){
        return Eigen::AngleAxisd(rvec.norm(),rvec.normalized()).toRotationMatrix();
    }

    Eigen::Matrix3d deltaRotMat(const Eigen::Vector3d &delta_rot_vec) {
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.vec() = 0.5 * delta_rot_vec;
        delta_R = delta_q.toRotationMatrix();

        return delta_R;
    }

    Eigen::Matrix3d rotationUpdate(const Eigen::Matrix3d &Rwi,
                                   const Eigen::Matrix3d &delta_rot_mat) {
//        need debug
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
    void AddFirstGNSSPoint(const GNSSOdometryType &data){
        state.Rwb_ = data.pose.GetR();
        state.p_wb_ = data.pose.GetXYZ();
        state.timestamp = data.timestamp;

        last_state = state;
        flag_firstGNSSPoint =1;
    }


    void IMUWheel_predict(IMURawWheelDataPtr curr_imu,
                          const Eigen::Matrix3d &_R_w_b,
                          double* _t_w_b){
        double imuTime = curr_imu->timestamp;
        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
//        state_.timestamp = curr_imu->timestamp;
//        const Eigen::Vector3d gyr_unbias =  curr_imu->imu_angular_v - prior_gyro_bias;  // gyro not unbias for now
        const Eigen::Vector3d gyr_unbias =  curr_imu->imu_angular_v;  // no bias
        const auto &dR = deltaRotMat(gyr_unbias * dt);
        Eigen::Vector3d state_v_b = Eigen::Vector3d (0, curr_imu->velocity,0);
//        state.v_wb_ = SensorConfig::T  state_v_b
        static bool flag_firstGnss = false;
        if(flag_firstGnss == false){
            state.Rwb_ = _R_w_b;
            state.p_wb_[0] = _t_w_b[0];
            state.p_wb_[1] = _t_w_b[1];
            state.p_wb_[2] = _t_w_b[2];
            last_state = state;
            flag_firstGnss = true;

        }
        if(SensorConfig::if_DR_use_Euler == 1){
            state.Rwb_ = _R_w_b;
            state.Rwb_ = rotationUpdate(state.Rwb_ , dR);
            Eigen::Vector3d state_v_w = state.Rwb_ * state_v_b;
            state.v_w_ = state_v_w;
            state.p_wb_ = last_state.p_wb_ + (last_state.v_w_ + state.v_w_)/2 *dt;//position
        }
        else{
            //pure DR
            //state.Rwb_ = last_state.Rwb_;
            state.Rwb_ = rotationUpdate(last_state.Rwb_ , dR);
            Eigen::Vector3d state_v_w = state.Rwb_ * state_v_b;
            state.v_w_ = state_v_w;
            state.p_wb_ = last_state.p_wb_ + (last_state.v_w_ + state.v_w_)/2 *dt;//position
        }

//        EZLOG(INFO)<<"gyr_unbias : "<<gyr_unbias.transpose();
//        EZLOG(INFO)<<"dR :"<<dR;
//        EZLOG(INFO)<<"state Rwb_: ";
//        std::cout<<state.Rwb_<< std::endl;
//        EZLOG(INFO)<<"state p_wb_: "<<state.p_wb_.transpose();
//        EZLOG(INFO)<<"state v_wb_: "<<state.v_w_.transpose();
//        EZLOG(INFO)<<"last_state Rwb_: ";
//        std::cout<<last_state.Rwb_<< std::endl;
//        EZLOG(INFO)<<"last_state p_wb_: "<<last_state.p_wb_.transpose();
//        EZLOG(INFO)<<"last_state v_wb_: "<<last_state.v_w_.transpose();
        last_state = state;


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

       // EZLOG(INFO)<<"firstLidarPose: "<<firstLidarPose;
        gtsam::NavState _currentState = imuIntegratorImu_->predict(firstLidarPose, prior_imu_bias);//both are input parameters
      //  EZLOG(INFO) <<" imu_raw->imu_linear_acc: "<<_imu_raw->imu_linear_acc.transpose()
      //              <<" imu_raw->imu_angular_v: "<<_imu_raw->imu_angular_v.transpose()
      //              <<" dt: "<< dt
     //               <<" prevBiasOdom: "<<prior_imu_bias;
       // EZLOG(INFO)<<" currentState: "<<_currentState;

        return _currentState;
    }


//    void Udp_OdomPub(const PoseT& data){
//        Vis_Odometry odom_out;
//        std::string fu_str;
//        odom_out.type = "dr";
//        odom_out.t[0]= data.GetXYZ().x();
//        odom_out.t[1]= data.GetXYZ().y();
//        odom_out.t[2]= data.GetXYZ().z();
//
//
//
//        odom_out.q.x() = data.GetQ().x();
//        odom_out.q.y() = data.GetQ().y();
//        odom_out.q.z() = data.GetQ().z();
//        odom_out.q.w() = data.GetQ().w();
//
//        fu_str = odom_out.ToString();
//        udp_thread -> SendUdpMSg(fu_str);
//    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){

        TicToc time1;
        double currentTime = gnss_ins_data.timestamp;
        //calculate Quaternion
        Eigen::Matrix3d z_matrix;
        Eigen::Matrix3d x_matrix;
        Eigen::Matrix3d y_matrix;
        double heading_Z = -gnss_ins_data.yaw * 3.1415926535 / 180.0;
        double pitch_Y = gnss_ins_data.pitch * 3.1415926535 / 180.0;
        double roll_X = gnss_ins_data.roll * 3.1415926535 / 180.0;
        z_matrix << cos(heading_Z), -sin(heading_Z), 0,
                sin(heading_Z), cos(heading_Z),  0,
                0,                 0,            1;

        x_matrix << 1,                 0,              0,
                0,            cos(pitch_Y),     -sin(pitch_Y),
                0,            sin(pitch_Y),    cos(pitch_Y);

        y_matrix << cos(roll_X) ,     0,        sin(roll_X),
                0,                 1,               0 ,
                -sin(roll_X),      0,         cos(roll_X);

        R_w_b =  (z_matrix * x_matrix * y_matrix);   // Pw = Twb * Pb, 右前上，zyx前左上

        double t_enu[3];
        geoConverter.Reset(gnss_ins_data.lla[0], gnss_ins_data.lla[1], gnss_ins_data.lla[2]);
        geoConverter.Forward(gnss_ins_data.lla[0], gnss_ins_data.lla[1], gnss_ins_data.lla[2],
                             t_enu[0], t_enu[1], t_enu[2]);//t_enu = enu coordiate
//        static int imu_cnt = 0;
//          for debug
//        outfile.precision(6);
//        outfile << imu_raw->imu_angular_v.x()<<" "<< imu_raw->imu_angular_v.y()<<" "<<imu_raw->imu_angular_v.z()<<std::endl;

        //
//        if(flag_firstGNSSPoint == 0 ){
//            return ;
//        }

        if(SensorConfig::if_use_Wheel_DR == 1){
            IMURawWheelDataPtr imuWheel_raw (new IMURawWheelData);
            imuWheel_raw->imu_angular_v = gnss_ins_data.imu_angular_v * 3.1415926535 / 180.0; //转弧度值
            imuWheel_raw->imu_linear_acc = gnss_ins_data.imu_linear_acc;
            imuWheel_raw->timestamp = gnss_ins_data.timestamp;
            imuWheel_raw->velocity = gnss_ins_data.velocity;
            IMUWheel_predict(imuWheel_raw,R_w_b,t_enu);
        }
        else{
            IMURawDataPtr imu_raw (new IMURawData);
            imu_raw->imu_angular_v = gnss_ins_data.imu_angular_v * 3.1415926535 / 180.0; //转弧度值
            imu_raw->imu_linear_acc = gnss_ins_data.imu_linear_acc;
            imu_raw->timestamp = gnss_ins_data.timestamp;
            gnssins_mutex.lock();
            imuQueImu.push_back(imu_raw);
            gnssins_mutex.unlock();
            currentState = IMU_predict(imu_raw);
        }

        OdometryType Odometry_imuPredict_pub;
        DROdometryType DR_pose;
        if(SensorConfig::if_use_Wheel_DR == 1){
            Eigen::Matrix3d R_b_l;
            Eigen::Vector3d lidar_preditct_pose_p_wb_;
            Eigen::Matrix3d lidar_preditct_pose_Rwl_;
            R_b_l = SensorConfig::T_L_DR.block<3,3>(0, 0);
            lidar_preditct_pose_Rwl_ = state.Rwb_ * R_b_l;// Rwl = Rwb * Rbl
            //lidar_preditct_pose_p_wb_ = R_b_l.inverse() * state.p_wb_; // 地面存在高程误差——外参？ 建图？
            lidar_preditct_pose_p_wb_ =  state.Rwb_ * (SensorConfig::T_L_B.inverse().block<3,1>(0,3)) + state.p_wb_; // 地面存在高程误差——外参？ 建图？
            //lidar_preditct_pose_p_wb_ =  lidar_preditct_pose_p_wb_ + SensorConfig::T_L_DR.block<3,1>(0,3);

            PoseT lidar_preditct_pose(lidar_preditct_pose_p_wb_, lidar_preditct_pose_Rwl_);
            //PoseT lidar_predict_world_pose = PoseT(lidar_preditct_pose.pose * SensorConfig::T_L_B);
            Odometry_imuPredict_pub.pose = lidar_preditct_pose;
            DR_pose.pose = lidar_preditct_pose;
        }
        else{
            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
//        gtsam::Pose3 lidar_preditct_pose_gtsam = imuPose.compose(lidar2Imu);

            PoseT lidar_preditct_pose(imuPose.translation().vector(),
                                      imuPose.rotation().matrix());
            Odometry_imuPredict_pub.pose = lidar_preditct_pose;
            DR_pose.pose = lidar_preditct_pose;
        }

        Odometry_imuPredict_pub.timestamp = currentTime;
        Odometry_imuPredict_pub.frame = "map";
        if(MappingConfig::use_DR_or_fuse_in_loc == 1){
            Function_AddDROdometryTypeToImageProjection(Odometry_imuPredict_pub);
            EZLOG(INFO)<<"1 of 2, DR send to data_preprocess. And Send Pose begin: ";
            EZLOG(INFO)<<Odometry_imuPredict_pub.pose.pose;
            EZLOG(INFO)<<"1 of 2, DR send to data_preprocess. And Send Pose end!";
        }

        //for debug use
        DR_pose.timestamp = currentTime;
        DR_pose.frame = "map";
        if(MappingConfig::slam_mode_switch == 1){
            Function_AddDROdometryTypeToFuse(DR_pose);
            EZLOG(INFO)<<"2 of 2, DR send to fuse. And Send Pose begin: ";
            EZLOG(INFO)<<DR_pose.pose.pose;
            EZLOG(INFO)<<"2 of 2, DR send to fuse. And Send Pose end!";
        }

        //Udp_OdomPub(Odometry_imuPredict_pub.pose);
        pubsub->PublishOdometry(topic_imu_raw_odom, Odometry_imuPredict_pub);

   //     EZLOG(INFO)<<" time in ms: "<<time1.toc();
//        imu_cnt++;
//        EZLOG(INFO)<<"imu_cnt: "<<imu_cnt;

    }

    //
    void SetIMUPreParamter(){

        //TODO!!!!!!!!
        ///why not imuAccBiasN and imuGyrBiasN???
        lastImuT_imu = -1;
        gtsam::Pose3 firstLidarPose_pose3 =
                gtsam::Pose3(gtsam::Rot3(  1,0,0,
                                                 0,1,0,
                                                 0,0,1),
                             gtsam::Point3(0, 0, 0));
        gtsam::Vector3 firstLidarVel =
                gtsam::Vector3(0,0,0);
        firstLidarPose = gtsam::NavState(firstLidarPose_pose3,firstLidarVel);
        state.Rwb_ <<1,0,0,
                    0,1,0,
                    0,0,1;
        state.p_wb_ << 0,0,0;
        state.v_w_ << 0,0,0;

        graph = new gtsam::NonlinearFactorGraph();

        last_state = state;

        if(SensorConfig::if_use_Wheel_DR == 0){
            key = 0;
            gtsam::Values initial_values;
            priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
            priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
            priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
            correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());

            prior_imu_bias = gtsam::imuBias::ConstantBias((gtsam::Vector(6) <<
                                                                            SensorConfig::imuConstBias_acc,
                    SensorConfig::imuConstBias_acc,
                    SensorConfig::imuConstBias_acc,
                    SensorConfig::imuConstBias_gyro,
                    SensorConfig::imuConstBias_gyro,
                    SensorConfig::imuConstBias_gyro).finished());  // assume zero initial bias

            prior_acc_bias = Eigen::Vector3d (SensorConfig::imuConstBias_acc,
                                              SensorConfig::imuConstBias_acc,
                                              SensorConfig::imuConstBias_acc);
            prior_gyro_bias = Eigen::Vector3d (SensorConfig::imuConstBias_gyro,
                                               SensorConfig::imuConstBias_gyro,
                                               SensorConfig::imuConstBias_gyro);


            initial_values.insert(X(key), firstLidarPose_pose3);
            initial_values.insert(V(key), firstLidarVel);
            initial_values.insert(B(key), prior_imu_bias);

            graph->addPrior(X(key), firstLidarPose_pose3, priorPoseNoise);
            graph->addPrior(V(key), firstLidarVel, priorVelNoise);
            graph->addPrior(B(key), prior_imu_bias, priorBiasNoise);

            boost::shared_ptr<gtsam::PreintegrationParams> p =  gtsam::PreintegrationParams::MakeSharedU(SensorConfig::imuGravity);

            // Realistic MEMS white noise characteristics. Angular and velocity random walk
            // expressed in degrees respectively m/s per sqrt(hr). 弧度制,米
            //   example:
            //          kGyroSigma = radians(0.5) / 60;     // 0.5 degree ARW,
            //          kAccelSigma = 0.1 / 60;             // 10 cm VRW

            double accel_noise_sigma = 0.0003924;
            double gyro_noise_sigma = 0.000205689024915;
            gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(accel_noise_sigma, 2);
            gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(gyro_noise_sigma, 2);
            gtsam::Matrix33 integration_error_cov = gtsam::I_3x3 * 1e-8;  // error committed in integrating position from velocities

            // PreintegrationBase params:
            p->accelerometerCovariance = measured_acc_cov;  // acc white noise in continuous
            p->integrationCovariance = integration_error_cov;  // integration uncertainty continuous
            // should be using 2nd order integration
            // PreintegratedRotation params:
            p->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous
            // PreintegrationCombinedMeasurements params:

            //used for predict
            std::shared_ptr<gtsam::PreintegrationType> imuIntegratorImu_ = nullptr;
            imuIntegratorImu_ = std::make_shared<gtsam::PreintegratedImuMeasurements>( p,prior_imu_bias);  // setting up the IMU integration for IMU message
            assert(imuIntegratorImu_);
            imuIntegratorImu_->resetIntegrationAndSetBias(prior_imu_bias);

            lastState = gtsam::NavState(firstLidarPose_pose3,firstLidarVel);

        }

    }

    void ClearFactorGraph() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.factorization = gtsam::ISAM2Params::CHOLESKY;
        optParameters.relinearizeSkip = 10;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

//    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_){
//
//        //设置imu的参数
//        SetIMUPreParamter();
//        //debug
////        outfile.open("imu_gyro.txt",std::ios::app);
////        if (!outfile.is_open()) {
////            std::cerr << "无法打开输出文件 " << std::endl;
////        }
////        outfile << "gyro.x "<<"gyro.y "<<"gyro.z"<<std::endl;
//
//        pubsub = pubsub_;
//        udp_thread = udp_thread_;
//        pubsub->addPublisher(topic_imu_raw_odom, DataType::ODOMETRY, 10);
//    }
    void Init(PubSubInterface* pubsub_){

        //设置imu的参数
        SetIMUPreParamter();
        //debug
//        outfile.open("imu_gyro.txt",std::ios::app);
//        if (!outfile.is_open()) {
//            std::cerr << "无法打开输出文件 " << std::endl;
//        }
//        outfile << "gyro.x "<<"gyro.y "<<"gyro.z"<<std::endl;

        pubsub = pubsub_;
        pubsub->addPublisher(topic_imu_raw_odom, DataType::ODOMETRY, 10);
    }
};

#endif //SEU_LIDARLOC_IMUPREINTEGRATION_H

