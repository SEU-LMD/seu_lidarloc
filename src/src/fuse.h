//
// Created by fyy on 23-10-12.
//

#ifndef SEU_LIDARLOC_FUSE_H
#define SEU_LIDARLOC_FUSE_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"

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

class Fuse{
public:
    PubSubInterface* pubsub;
    std::thread* HighFrequencyLoc_thread;
    int data_deque_max_size = 200;
    int front_index = 0;
    std::deque<std::shared_ptr<BaseType>> data_deque;
    std::deque<std::shared_ptr<BaseType>> imu_data_deque;
    std::deque<std::shared_ptr<BaseType>> lidar_data_deque;
    std::deque<std::shared_ptr<BaseType>> gnss_data_deque;
    std::mutex mutex_data;
    gtsam::PreintegratedImuMeasurements *imuIntegrator;//just use imu to predict
    gtsam::imuBias::ConstantBias prior_imu_bias;
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::Vector noiseModelBetweenBias;
    bool firstLidarFlag;
    int key = 1;
    double lastImuT_imu = -1;


    std::string topic_highFequency_odom = "/loc_result";


    //this fucntion needs to be binded by lidar loc node
    void AddLidarLocToFuse(const OdometryType &lidar_loc_res){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<OdometryType>(lidar_loc_res);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }


    void AddGNSSToFuse(const GNSSOdometryType &gnss_odom){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<GNSSOdometryType>(gnss_odom);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

    void AddIMUToFuse(const IMUOdometryType &imu_odom){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<IMUOdometryType>(imu_odom);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

//    std::vector<GNSS_INSType> = FindAfterTimestampIMU(front_data.time_stamp){
//
//    }

    void DR_predict(const std::deque<std::shared_ptr<BaseType>> _imu_data_deque){
        IMUOdometryTypePtr cur_imu_odom;
        cur_imu_odom = std::static_pointer_cast<IMUOdometryType>(std::move(_imu_data_deque.front()));
        double imuTime = cur_imu_odom->timestamp;

        double dt = (lastImuT_imu < 0) ? (1.0 / SensorConfig::imuHZ) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
//        imuIntegrator->integrateMeasurement(cur_imu_odom->pose.,
//                                            cur_imu_odom->imu_angular_v,
//                                                dt);

    }

    void GNSS_StatusCheck(std::deque<std::shared_ptr<BaseType>> _gnss_data_deque){
        static int cnt_test = 0;
        if(cnt_test > 100){
            EZLOG(INFO)<<"GNSS: "<< cnt_test;
            cnt_test = 0;
        }
        cnt_test++;
    }

    void fuseInitialized(){
        prior_imu_bias = gtsam::imuBias::ConstantBias((gtsam::Vector(6) <<
                                                        SensorConfig::imuConstBias_acc,
                                                        SensorConfig::imuConstBias_acc,
                                                        SensorConfig::imuConstBias_acc,
                                                        SensorConfig::imuConstBias_gyro,
                                                        SensorConfig::imuConstBias_gyro,
                                                        SensorConfig::imuConstBias_gyro).finished());
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
        imuIntegrator = new gtsam::PreintegratedImuMeasurements( p,prior_imu_bias);
        //       对角平方根协方差矩阵
        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());  // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);  // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());  // rad,rad,rad,m, m, m
        //TODO correct!!!!!
        noiseModelBetweenBias = (gtsam::Vector(6) << SensorConfig::imuAccBiasN,
                SensorConfig::imuAccBiasN,
                SensorConfig::imuAccBiasN,
                SensorConfig::imuGyrBiasN,
                SensorConfig::imuGyrBiasN,
                SensorConfig::imuGyrBiasN).finished();

        imuIntegrator->resetIntegrationAndSetBias(prior_imu_bias);
        EZLOG(INFO)<<"Fuse node init successful!"<<std::endl;
    }

    void LIDAR_PoseRollBack(std::deque<std::shared_ptr<BaseType>> _lidar_data_deque){

        OdometryTypePtr cur_lidar_odom;
        cur_lidar_odom = std::static_pointer_cast<OdometryType>(std::move(_lidar_data_deque.front()));
        double currentCorrectionTime = cur_lidar_odom->timestamp;

        gtsam::Pose3 lidarPose =
                gtsam::Pose3(gtsam::Rot3::Quaternion(cur_lidar_odom->pose.GetQ().w(),
                                                     cur_lidar_odom->pose.GetQ().x(),
                                                     cur_lidar_odom->pose.GetQ().y(),
                                                     cur_lidar_odom->pose.GetQ().z()),
                             gtsam::Point3(cur_lidar_odom->pose.GetXYZ()));
        EZLOG(INFO)<<"lidarPose: "<<lidarPose;


    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
//        pubsub->addPublisher(topic_highFequency_odom, DataType::ODOMETRY, 10);
        HighFrequencyLoc_thread = new std::thread(&Fuse::DoWork, this);
    }

    void DoWork(){
        fuseInitialized();
        while(1){
            if(data_deque.size()!=0){
                OdometryType send_data;//final send data

                mutex_data.lock();
//                if(front_index > data_deque.size()-1){
//                    front_index = data_deque.size()-1;
//                    continue;
//                }
//                auto front_data = data_deque[front_index];
                auto front_data = data_deque.front();
                EZLOG(INFO)<<"data_deque size: "<<data_deque.size();
//                std::vector<GNSS_INSType> re_predict_imu_data;
                switch (front_data->getType()) {
                    case DataType::ODOMETRY:
                    {
                        lidar_data_deque.push_back(front_data);
                        EZLOG(INFO)<<"GET Lidar! now we got lidar_data_deque size: "<<lidar_data_deque.size();
                        data_deque.pop_front();
                        mutex_data.unlock();
                        if(firstLidarFlag == false){
                            key = 1;
                            firstLidarFlag = true;
                        }
                        LIDAR_PoseRollBack(lidar_data_deque);
//                        factor

                        break;
                    }

                    case DataType::GNSS:
                    {
                        if(firstLidarFlag == false){
                            data_deque.pop_front();
                            mutex_data.unlock();
                            EZLOG(INFO)<<"DataType::GNSS  : wait fot first lidar pose!!";
                            break;
                        }
                        gnss_data_deque.push_back(front_data);
                        data_deque.pop_front();
                        mutex_data.unlock();

                        EZLOG(INFO)<<"GET GNSS! now we got gnss_data_deque size: "<<gnss_data_deque.size();
                        GNSS_StatusCheck(gnss_data_deque);

//                        front_index++;
                        break;
                    }

                    case DataType::IMU:
                    {
                        if(firstLidarFlag == false){
                            data_deque.pop_front();
                            mutex_data.unlock();
                            EZLOG(INFO)<<"DataType::IMU  : wait fot first lidar pose!!";
                            break;
                        }
                        imu_data_deque.push_back(front_data);
                        data_deque.pop_front();
                        mutex_data.unlock();

                        EZLOG(INFO)<<"GET IMU! now we got imu_data_deque size: "<<imu_data_deque.size();
                        DR_predict(imu_data_deque);

//                        predict imu pose
//                        front_index++;
                        break;
                    }
                }
//                if(front_data.getType()==DataType::ODOMETRY){
//                    re_predict_imu_data = FindAfterTimestampIMU(front_data.time_stamp);
//
//                }

//                if(front_data.getType()==DataType::GNSS_INS){
//                    //just use imu data to predcict
//
//
//                }
//                else if(front_data.getType()==DataType::ODOMETRY){
//                    //just use lidar loc data to optimize factor
//                    //1.add prior or beteween factor to graph
//
//
//                    //2. optimize graph
//
//                    //3.re-predict imu factor
//                    for(auto& imu_data: re_predict_imu_data){
//
//                    }
//                }


//                finally send data to middle ware use send_data


                mutex_data.lock();
                while(data_deque.size()>=data_deque_max_size){
                    data_deque.pop_front();
//                    front_index--;
                }
                mutex_data.unlock();
            }
            else{
                sleep(0.001);
            }
        }
    }//end fucntion do work


};
#endif //SEU_LIDARLOC_FUSE_H
