//
// Created by fyy on 23-9-11.
//

#ifndef SEU_LIDARLOC_IMUPREINTEGRATION_H
#define SEU_LIDARLOC_IMUPREINTEGRATION_H
#include <mutex>
#include <thread>

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include <gtsam/inference/Symbol.h"
#include <gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/navigation/GPSFactor.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"

#include "utils/timer.h"
#include "pubsub/pubusb.h"

class DeadReckoning{
public:
    PubSubInterface* pubsub;
    std::thread* do_work_thread;
    std::mutex odom_mutex;
    std::mutex imu_mutex;
    std::deque<OdometryType> deque_odom;
    std::deque<GNSSINSType> deque_imu;

    bool doneFirstOpt = false;

    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;
    gtsam::Pose3 imu2Lidar =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(-SensorConfig::extrinsicTrans.x(), -SensorConfig::extrinsicTrans.y(), -SensorConfig::extrinsicTrans.z()));
    gtsam::Pose3 lidar2Imu =
            gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                         gtsam::Point3(SensorConfig::extrinsicTrans.x(), SensorConfig::extrinsicTrans.y(), SensorConfig::extrinsicTrans.z()));



    void DoWork(){
        while(1){
            if(deque_odom.size()!=0){
                OdometryType cur_imu;
                odom_mutex.lock();
                cur_imu = deque_odom.front();
                deque_odom.pop_front();
                odom_mutex.unlock();

                //do something to correct bias


            }
        }
    }

    GNSSINSType ConvertIMUCoordinate(const GNSSINSType& gnssins_data){
        GNSSINSType res = gnssins_data;
        res.imu_linear_acc = SensorConfig::T_L_B.block<3,3>(0, 0) * res.imu_linear_acc;
        res.imu_angular_v = SensorConfig::T_L_B.block<3,3>(0, 0) * res.imu_angular_v;
        return res;
    }

    void AddGNSSINSSData(const GNSSINSType& gnssins_data){
        TicToc timer;
        //directly preintegrate
        GNSSINSType imu_data_lidar = ConvertIMUCoordinate(gnssins_data);
//        sensor_msgs::Imu::Ptr imu_raw(new sensor_msgs::Imu());
//        imu_raw->header = gnssImu_raw->header;
//        imu_raw->linear_acceleration.x = gnssImu_raw->accx;
//        imu_raw->linear_acceleration.y = gnssImu_raw->accy;
//        imu_raw->linear_acceleration.z = gnssImu_raw->accz;
//
//        imu_raw->angular_velocity.x = gnssImu_raw->angx;
//        imu_raw->angular_velocity.y = gnssImu_raw->angy;
//        imu_raw->angular_velocity.z = gnssImu_raw->yaw;
//
//        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false) return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        //    std::cout << "dt: " << dt << std::endl;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(imu_data_lidar.imu_linear_acc[0],
                               imu_data_lidar.imu_linear_acc[1],
                               imu_data_lidar.imu_linear_acc[2]),
                gtsam::Vector3(imu_data_lidar.imu_angular_v[0],
                                            imu_data_lidar.imu_angular_v[1],
                                            imu_data_lidar.imu_angular_v[2]),
                dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

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
        EZLOG(INFO)<<"pre-integration cost time(ms) = "<<timer.toc()<<std::endl;
    }

    void AddOdometryData(const OdometryType& odo_data){
        odom_mutex.lock();
        deque_odom.push_back(odo_data);
        odom_mutex.unlock();
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        do_work_thread = new std::thread(&DeadReckoning::DoWork, this);
    }
};
#endif //SEU_LIDARLOC_IMUPREINTEGRATION_H
