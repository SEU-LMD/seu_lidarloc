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
// #include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "utility.h"
#include "ivsensorgps.h"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

std::ofstream fout_gpsimu_raw_data("/home/sy/dataSet/ZhongKeXingChi/output/1403/gpsimu.txt", std::ios::app);
int fileflag=0;

class TransformFusion : public ParamServer {
public:
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

    /**
     * 根据IMU预积分量，基于最近一帧lidar的位姿，得到当前帧位姿
     */
    TransformFusion() {
//        IMU and lidar frame are different
//        if (lidarFrame != baselinkFrame) {
//            try {
//                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0),
//                                            ros::Duration(10.0));
//                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0),
//                                           lidar2Baselink);
//            } catch (tf::TransformException ex) {
//                ROS_ERROR("%s", ex.what());
//            }
//        }
//        tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0),
//                                            ros::Duration(3.0));
//        tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0),
//                                   lidar2Baselink);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
                "lio_sam_6axis/mapping/odometry", 5,
                &TransformFusion::lidarOdometryHandler, this,
                ros::TransportHints().tcpNoDelay());
        subImuOdometry = nh.subscribe<nav_msgs::Odometry>(
                odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler,
                this, ros::TransportHints().tcpNoDelay());
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam_6axis/imu/path", 1);
    }

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

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    /**
     *订阅imu里程计，来自IMUPreintegration
     * 1、以最近一帧激光里程计位姿为基础，计算该时刻与当前时刻间imu里程计增量位姿变换，相乘得到当前时刻imu里程计位姿
     * 2、发布当前时刻里程计位姿，用于rviz展示；发布imu里程计路径，注：只是最近一帧激光里程计时刻与当前时刻之间的一段
     * @param odomMsg
     */
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(
                tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(
                map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1) return;
        while (!imuOdomQueue.empty()) {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
//        上一帧激光时刻的IMU位姿
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
//        当前IMU位姿
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());

        //  当前时刻imu里程计位姿=最近的一帧激光里程计位姿 * imu里程计增量位姿变换
        Eigen::Affine3f imuOdomAffineIncre =
                imuOdomAffineFront.inverse() * imuOdomAffineBack;
//        IMU的增量变化
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch,
                                          yaw);

        // publish latest odometry
//        odometry/imu = odometry/imu_incremental(激光修正) + odometry/imu
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
//        if (lidarFrame != baselinkFrame) tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(
                tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1) {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() &&
                   imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0) {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer {
public:
    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    //imuIntegratorOpt_负责预积分两个激光里程计之间的imu数据，作为约束加入因子图，并且优化出bias
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    //imuIntegratorImu_用来根据新的激光里程计到达后已经优化好的bias，预测从当前帧开始，下一帧激光里程计到达之前的imu里程计增量
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

    // imu-lidar位姿变换
    //这点要注意，tixiaoshan这里命名的很垃圾，这只是一个平移变换，
    //同样头文件的imuConverter中，也只有一个旋转变换。这里绝对不可以理解为把imu数据转到lidar下的变换矩阵。
    //事实上，作者后续是把imu数据先用imuConverter旋转到雷达系下（但其实还差了个平移）。
    //作者真正是把雷达数据又根据lidar2Imu反向平移了一下，和转换以后差了个平移的imu数据在“中间系”对齐，
    //之后算完又从中间系通过imu2Lidar挪回了雷达系进行publish。
    gtsam::Pose3 imu2Lidar =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration() {
        // 接受IMU原始数据，发布两帧间激光系下的里程计，pubImuOdometry
        subImu = nh.subscribe<gps_imu::ivsensorgps>(
                imuTopic, 2000, &IMUPreintegration::imuHandler, this,
                ros::TransportHints().tcpNoDelay());
        //        Lidar odometry
//        来自mapOptimization
        subOdometry = nh.subscribe<nav_msgs::Odometry>(
                "lio_sam_6axis/mapping/odometry_incremental", 5,
                &IMUPreintegration::odometryHandler, this,
                ros::TransportHints().tcpNoDelay());

        //        odometry/imu_incremental
        pubImuOdometry =
                nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p =
                gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) *
                pow(imuAccNoise, 2);  // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) *
                pow(imuGyrNoise, 2);  // gyro white noise in continuous
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
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN,
                imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN)
                .finished();

//    if (sensor == SensorType::HESAI) {
//      //      p->n_gravity = gtsam::Vector3(imuGravity_N[0], imuGravity_N[1],
//      //      imuGravity_N[2]);
//      noiseModelBetweenBias =
//          (gtsam::Vector(6) << imuAccBias_N[0], imuAccBias_N[1],
//           imuAccBias_N[2], imuGyrBias_N[0], imuGyrBias_N[1], imuGyrBias_N[2])
//              .finished();
//    } else {
        noiseModelBetweenBias =
                (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN,
                        imuGyrBiasN, imuGyrBiasN, imuGyrBiasN)
                        .finished();
//    }

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(
                p,
                prior_imu_bias);  // setting up the IMU integration for IMU message
        // thread
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

/**
 * 订阅激光里程计 /mapping/odometry_incremental,相对世界坐标系的位姿，有IMU优化的结果
 * (/mapping/odometry,是前端激光里程计直接优化得到的位姿）
 * (/mapping/）
 * @param odomMsg
 */
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
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_,
                                                       priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
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
//        接受来自mapOptimization的当前帧位姿，因子图优化更新当前帧位姿
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            // std::cout << " delta_t: " << imuTime -lastImuT_opt << std::endl;
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt =
                        (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                //        double dt = (lastImuT_opt < 0) ? (1.0 / imuFrequence) :
                //        (imuTime - lastImuT_opt);
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
        //利用两帧之间的IMU数据完成了预积分后增加imu因子到因子图中,
        //注意后面容易被遮挡，imuIntegratorOpt_的值经过格式转换被传入preint_imu，
        //因此可以推测imuIntegratorOpt_中的integrateMeasurement函数应该就是一个简单的积分轮子，
        //传入数据和dt，得到一个积分量,数据会被存放在imuIntegratorOpt_中
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
 * 发布距离最近的激光数据的IMU里程计的结果，转到激光里程计
 * odometry/imu
 * @param imu_raw
 */
    void imuHandler(const gps_imu::ivsensorgpsConstPtr &gnssImu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

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

//        std::cout << "gnssImu_raw.orientation_quaternion w: " << orientation_quaternion.w()
//                    <<" , x:  "<<orientation_quaternion.x()
//                    << ", y: " << orientation_quaternion.y()
//                    <<" , z: "<< orientation_quaternion.z() <<std::endl;

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false) return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        //    std::cout << "dt: " << dt << std::endl;

        // integrate this single imu message
//        从上一时刻激光雷达的数据开始
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x,
                               thisImu.linear_acceleration.y,
                               thisImu.linear_acceleration.z),
                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y,
                               thisImu.angular_velocity.z),
                dt);

        // predict odometry
//        得到相对于上一帧激光雷达的当前姿态，imu系
        gtsam::NavState currentState =
                imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
//        轉到激光雷达系
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
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

    ros::init(argc, argv, "roboat_loam");

    IMUPreintegration ImuP;

    TransformFusion TF;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
