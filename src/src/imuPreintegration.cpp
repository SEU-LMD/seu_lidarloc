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
#include "config_helper.h"
#include <iostream>

// #include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "utility.h"
#include "timer.h"

#include "easylogging++.h"
#include "ivsensorgps.h"
INITIALIZE_EASYLOGGINGPP
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz) /Pose3(x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

//订阅激光里程计（来自MapOptimization）和IMU里程计，
//根据前一时刻激光里程计，和该时刻到当前时刻的IMU里程计变换增量，
//计算当前时刻IMU里程计；
//rviz展示IMU里程计轨迹（局部）

class TransformFusion  {
public:
    ros::NodeHandle nh;
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion() {
        //subscribe LO,from mapOptimization
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
                "lidar_odometry_world", 5,
                &TransformFusion::lidarOdometryHandler, this,
                ros::TransportHints().tcpNoDelay());
        //subscribe IMU odometry,from IMUPreintegration
        //topic name: odometry/imu_incremental,是增量内容，即两帧激光里程计之间的预积分内容,（加上开始的激光里程计本身有的位姿）
        //imuIntegratorImu_本身是个积分器，只有两帧之间的预积分，但是发布的时候发布的实际是结合了前述里程计本身有的位姿
        //currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);
        subImuOdometry = nh.subscribe<nav_msgs::Odometry>(
                SensorConfig::odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler,
                this, ros::TransportHints().tcpNoDelay());
        //for rviz use
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(SensorConfig::odomTopic, 2000);
        pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam_6axis/imu/path", 1);
    }

    //calculate transformation according to odometry
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    //LO,from mapOptimization
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        // find T and time
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    //sub imu odometry,from IMUPreintegration
    //1、以最近一帧激光里程计位姿为基础，计算该时刻与当前时刻间imu里程计增量位姿变换，相乘得到当前时刻imu里程计位姿
    //2、发布当前时刻里程计位姿，用于rviz展示；发布imu里程计路径，注：只是最近一帧激光里程计时刻与当前时刻之间的一段
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        // static tf,map与odom系设为同一个系
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(
                tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(
                map_to_odom, odomMsg->header.stamp, SensorConfig::mapFrame, SensorConfig::odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);
        // 添加imu里程计到队列，注：imu里程计由本cpp中的另一个类imuPreintegration来发布
        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        //从imu里程计队列中删除当前（最近的一帧）激光里程计时刻之前的数据
        // lidarOdomTime初始化为-1，在收到lidar里程计数据后，在回调函数lidarOdometryHandler中被赋值时间戳
        if (lidarOdomTime == -1) return;
        while (!imuOdomQueue.empty()) {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        //imu里程计增量位姿变换
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre =
                imuOdomAffineFront.inverse() * imuOdomAffineBack;
        //当前时刻imu里程计位姿=最近的一帧激光里程计位姿 * imu里程计增量位姿变换
        //lidarOdomAffine在本类的lidarOdometryHandler回调函数中被赋值，消息来源于mapOptimization.cpp发布,是激光里程计
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch,
                                          yaw);

        // publish latest odometry
        //发布名为"odometry/imu"的话题
        //也就是说，这个函数先监听了类IMUPreintegration中发布的odometry/imu_incremental，
        //然后计算imu二者之间的增量，然后在激光的基础上加上增量，重新发布
        //把x，y，z，roll，pitch，yaw都经过了激光里程计的修正，才发布当前时刻里程计位姿.发布名称为"odometry/imu"
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation =tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        //publish tf mark!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //发布tf，当前时刻odom与baselink系变换关系
        //由于之前把map和odom坐标系固定了，因此这里我认为发布的就是真正的最终位姿关系
        //map优化提供激光，预积分提供imu，imu之间变换再乘以激光里程计得到各个时刻精确位姿
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if (SensorConfig::lidarFrame != SensorConfig::baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(
                tCur, odomMsg->header.stamp, SensorConfig::odometryFrame,SensorConfig::baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path mark!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        // 发布imu里程计路径，注：只是最近一帧激光里程计时刻与当前时刻之间的一段
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        // 每隔0.1s添加一个
        if (imuTime - last_path_time > 0.1) {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = SensorConfig::odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            // 删除最近一帧激光里程计时刻之前的imu里程计
            while (!imuPath.poses.empty() &&
                   imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0) {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = SensorConfig::odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration  {
public:
    ros::NodeHandle nh;

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

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

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

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

    IMUPreintegration() {
        subImu = nh.subscribe<gps_imu::ivsensorgps>(
                SensorConfig::imuTopic, 2000, &IMUPreintegration::imuHandler, this,
                ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>(
                "lidar_odometry_world", 5,
                &IMUPreintegration::odometryHandler, this,
                ros::TransportHints().tcpNoDelay());

        pubImuOdometry =
                nh.advertise<nav_msgs::Odometry>(SensorConfig::odomTopic + "_incremental", 2000);

        // imu预积分的噪声协方差
        boost::shared_ptr<gtsam::PreintegrationParams> p =
                gtsam::PreintegrationParams::MakeSharedU(SensorConfig::imuGravity);
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) *
                pow(SensorConfig::imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) *
                pow(SensorConfig::imuGyrNoise, 2);  // gyro white noise in continuous
        p->integrationCovariance =
                gtsam::Matrix33::Identity(3, 3) *
                pow(1e-4,
                    2);  // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());;  // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2)
                        .finished());  // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(
                6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1)
                        .finished());  // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1, 1, 1, 1, 1, 1)
                        .finished());  // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << SensorConfig::imuAccBiasN, SensorConfig::imuAccBiasN,
                SensorConfig::imuAccBiasN, SensorConfig::imuGyrBiasN, SensorConfig::imuGyrBiasN, SensorConfig::imuGyrBiasN)
                .finished();

        noiseModelBetweenBias =
                (gtsam::Vector(6) << SensorConfig::imuAccBiasN, SensorConfig::imuAccBiasN, SensorConfig::imuAccBiasN,
                        SensorConfig::imuGyrBiasN, SensorConfig::imuGyrBiasN, SensorConfig::imuGyrBiasN)
                        .finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(
                p,
                prior_imu_bias);  // setting up the IMU integration for IMU message
//        // thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(
                p, prior_imu_bias);  // setting up the IMU integration for optimization
    }

    void resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty()) return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int) odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose =
                gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                             gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false) {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty()) {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                } else
                    break;
            }
            // initial pose
//            change Lidar to IMU
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                       priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
//            速度初值可以给组合导航的输出
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_,
                                                        priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(
                    B(0), prevBias_, priorBiasNoise);
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

            key = 1;
            systemInitialized = true;
            return;
        }

        // reset graph for speed
        if (key == 100) {
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
            resetOptimization();
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
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            // std::cout << " delta_t: " << imuTime -lastImuT_opt << std::endl;
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt =
                        (lastImuT_opt < 0) ? (1.0 / 100.0) : (imuTime - lastImuT_opt); // 100Hz
                //        double dt = (lastImuT_opt < 0) ? (1.0 / imuFrequence) :
                //        (imuTime - lastImuT_opt);
//                积分器
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x,
                                       thisImu->linear_acceleration.y,
                                       thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,
                                       thisImu->angular_velocity.y,
                                       thisImu->angular_velocity.z),
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
            resetParams();
            return;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() &&
               ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }

        //注意，这里是要“删除”当前帧“之前”的imu数据，是想根据当前帧“之后”的累积递推。
        //而前面imuIntegratorOpt_做的事情是，“提取”当前帧“之前”的imu数据，用两帧之间的imu数据进行积分。处理过的就弹出来。
        //因此，新到一帧激光帧里程计数据时，imuQueOpt队列变化如下：
        //当前帧之前的数据被提出来做积分，用一个删一个（这样下一帧到达后，队列中就不会有现在这帧之前的数据了）
        //那么在更新完以后，imuQueOpt队列不再变化，剩下的原始imu数据用作下一次优化时的数据。
        //而imuQueImu队列则是把当前帧之前的imu数据都给直接剔除掉，仅保留当前帧之后的imu数据，

        // repropogate
        if (!imuQueImu.empty()) {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int) imuQueImu.size(); ++i) {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
                //        double dt = (lastImuQT < 0) ? (1.0 / imuFrequence) : (imuTime
                //        - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x,
                                       thisImu->linear_acceleration.y,
                                       thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,
                                       thisImu->angular_velocity.y,
                                       thisImu->angular_velocity.z),
                        dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3 &velCur,
                          const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(),
                           biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(),
                           biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }


    /**
     * 订阅imu原始数据
     * 1、用上一帧激光里程计时刻对应的状态、偏置，
     * 施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态，也就是imu里程计
     * 2、imu里程计位姿转到lidar系，发布里程计
    */

    void imuHandler(const gps_imu::ivsensorgpsConstPtr &gnssImu_raw) {
        std::lock_guard<std::mutex> lock(mtx);

//        in world Frame
        sensor_msgs::Imu::Ptr imu_raw(new sensor_msgs::Imu());
        imu_raw->header = gnssImu_raw->header;
        imu_raw->linear_acceleration.x = gnssImu_raw->accx;
        imu_raw->linear_acceleration.y = gnssImu_raw->accy;
        imu_raw->linear_acceleration.z = gnssImu_raw->accz;

        imu_raw->angular_velocity.x = gnssImu_raw->angx * 3.1415926535 / 180.0;
        imu_raw->angular_velocity.y = gnssImu_raw->angy * 3.1415926535 / 180.0;
        imu_raw->angular_velocity.z = gnssImu_raw->yaw * 3.1415926535 / 180.0;

        Eigen::Quaterniond orientation_quaternion;
        orientation_quaternion = Eigen::AngleAxisd(gnssImu_raw->roll * 3.1415926535 / 180.0, Eigen::Vector3d::UnitX())
                                 *Eigen::AngleAxisd(gnssImu_raw->pitch* 3.1415926535 / 180.0, Eigen::Vector3d::UnitY())
                                 *Eigen::AngleAxisd(gnssImu_raw->heading* 3.1415926535 / 180.0, Eigen::Vector3d::UnitZ());
        imu_raw->orientation.x = orientation_quaternion.x();
        imu_raw->orientation.y = orientation_quaternion.y();
        imu_raw->orientation.z = orientation_quaternion.z();
        imu_raw->orientation.w = orientation_quaternion.w();

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false) return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 100.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        //    std::cout << "dt: " << dt << std::endl;

        // integrate this single imu message
        // imu预积分器添加一帧imu数据，注：这个预积分器的起始时刻是上一帧激光里程计时刻
//        由GTSAM计算IMU位姿
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x,
                               thisImu.linear_acceleration.y,
                               thisImu.linear_acceleration.z),
                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y,
                               thisImu.angular_velocity.z),
                dt);

        // predict odometry
        // 用上一帧激光里程计时刻对应的状态、偏置，施加从该时刻开始到当前时刻的imu预计分量，得到当前时刻的状态
        gtsam::NavState currentState =
                imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = SensorConfig::odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose =
                gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x =
                thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y =
                thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z =
                thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
    }


};


int main(int argc, char **argv) {
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif

    ros::init(argc, argv, "imu_pre");

    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");

    IMUPreintegration ImuP;

    TransformFusion TF;

    EZLOG(INFO)<<"IMU Preintegration Started!";
    ros::spin();
//    ros::MultiThreadedSpinner spinner(4);
//    spinner.spin();

    return 0;
}
