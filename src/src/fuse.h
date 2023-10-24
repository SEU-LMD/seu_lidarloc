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
    std::mutex mtxGraph;

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::ISAM2 *isam;
    gtsam::Values initialEstimate;
    gtsam::PreintegratedImuMeasurements *imuIntegrator;//just use imu to predict
    gtsam::imuBias::ConstantBias prior_imu_bias;
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::Vector noiseModelBetweenBias;
    gtsam::Pose3 last_lidar_pose;
    gtsam::Values isamCurrentEstimate; // 所有关键帧位姿的优化结果
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;

    bool firstLidarFlag;
    int key = 1;
    int lidar_keyFrame_cnt = 0;
    double lastImuT_imu = -1;
    OdometryType current_pose_world;
    std::string topic_current_pose = "/loc_result";


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
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new gtsam::ISAM2(parameters);

        priorNoise = gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                                .finished());  // rad*rad, meter*meter
        odometryNoise = gtsam::noiseModel::Diagonal::Variances(
                        (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());


        EZLOG(INFO)<<" Fuse Init Successful!";
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
        fuseInitialized();

        pubsub->addPublisher(topic_current_pose, DataType::ODOMETRY, 10);
        HighFrequencyLoc_thread = new std::thread(&Fuse::DoWork, this);
    }

    void DoWork(){
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
                data_deque.pop_front();
                mutex_data.unlock();
                double current_timeStamp = front_data->timestamp;
//                std::vector<GNSS_INSType> re_predict_imu_data;
                switch (front_data->getType()) {
                    case DataType::ODOMETRY:
                    {
                        OdometryTypePtr cur_lidar_odom;
                        cur_lidar_odom = std::static_pointer_cast<OdometryType>(std::move(front_data));

                        gtsam::Pose3 current_lidar_pose(gtsam::Rot3(cur_lidar_odom->pose.GetR()),
                                                            gtsam::Point3(cur_lidar_odom->pose.GetXYZ()));
                        if(lidar_keyFrame_cnt == 0){

                            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, current_lidar_pose,
                                                                            priorNoise));
                            initialEstimate.insert(0, current_lidar_pose);

                            last_lidar_pose = current_lidar_pose;
                            lidar_keyFrame_cnt++;
                            EZLOG(INFO)<<"GTSAM Optimization Init Successful!";
                            break;
                        }
                        else{
//                            add lidar odom factor

                            gtsam::Pose3 poseFrom = last_lidar_pose;
                            gtsam::Pose3 poseTo = current_lidar_pose;

                            mtxGraph.lock();
                            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                    lidar_keyFrame_cnt - 1, lidar_keyFrame_cnt,
                                    poseFrom.between(poseTo), odometryNoise));
                            initialEstimate.insert(lidar_keyFrame_cnt, poseTo);
                            mtxGraph.unlock();
                        }

                        if(firstLidarFlag == false){
                            key = 1;
                            firstLidarFlag = true;
                        }

                        std::cout << "****************************************************" << std::endl;
                        gtSAMgraph.print("Fuse GTSAM Graph:\n");
                        // update iSAM
                        isam->update(gtSAMgraph, initialEstimate);
                        isam->update();
                        gtSAMgraph.resize(0);
                        initialEstimate.clear();

                        gtsam::Pose3 latestEstimate;
                        isamCurrentEstimate = isam->calculateEstimate();
                        latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);
                        std::cout << "****************************************************" << std::endl;
//                        isamCurrentEstimate.print("Fuse Current estimate: ");

                        PoseT current_pose(latestEstimate.translation(),latestEstimate.rotation().matrix());
                        current_pose_world.frame = "map";
                        current_pose_world.timestamp = current_timeStamp;
                        current_pose_world.pose.pose = current_pose.pose;
                        pubsub->PublishOdometry(topic_current_pose, current_pose_world);
//                        factor
                        last_lidar_pose = current_lidar_pose;
                        lidar_keyFrame_cnt++;

                        break;
                    }

                    case DataType::GNSS:
                    {
//                        if(firstLidarFlag == false){
//                            data_deque.pop_front();
//                            mutex_data.unlock();
//                            EZLOG(INFO)<<"DataType::GNSS  : wait fot first lidar pose!!";
//                            break;
//                        }
//                        gnss_data_deque.push_back(front_data);
//                        data_deque.pop_front();
//                        mutex_data.unlock();
//
//                        EZLOG(INFO)<<"GET GNSS! now we got gnss_data_deque size: "<<gnss_data_deque.size();
//                        GNSS_StatusCheck(gnss_data_deque);
                            ;

//                        front_index++;
                        break;
                    }

                    case DataType::IMU:
                    {
//                        if(firstLidarFlag == false){
//                            data_deque.pop_front();
//                            mutex_data.unlock();
//                            EZLOG(INFO)<<"DataType::IMU  : wait fot first lidar pose!!";
//                            break;
//                        }
//                        imu_data_deque.push_back(front_data);
//                        data_deque.pop_front();
//                        mutex_data.unlock();
//
//                        EZLOG(INFO)<<"GET IMU! now we got imu_data_deque size: "<<imu_data_deque.size();
//                        DR_predict(imu_data_deque);
                        ;

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
