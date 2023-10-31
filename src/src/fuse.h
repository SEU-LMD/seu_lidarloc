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
    std::deque<std::shared_ptr<DROdometryType>> DR_data_deque;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToDR;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToMapManager;
    std::function<void(const OdometryType&)> Function_AddLidarOdometryTypeToImageProjection;
    std::mutex mutex_data;
    std::mutex mutex_DR_data;
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

    PoseT delta_pose_DR;
    PoseT current_T_i_0;

    bool firstLidarFlag;
    int key = 1;
    int lidar_keyFrame_cnt = 0;
    double lastImuT_imu = -1;
    double current_lidar_time;
    OdometryType loc_result;
    OdometryType current_lidar_world;
    PoseT last_pose;
    bool if_roll_back = 0;

    std::string topic_highHz_pose = "/loc_result";
    std::string topic_testforRollBack_pose = "/loc_result_roll_back";
    std::string topic_current_lidar_pose = "/loc_lidar_result";


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

    void AddIMUToFuse(const DROdometryType &imu_odom){
        mutex_data.lock();
        std::shared_ptr<BaseType> odometryPtr = std::make_shared<DROdometryType>(imu_odom);
        data_deque.push_back(odometryPtr);
        mutex_data.unlock();
    }

//    std::vector<GNSS_INSType> = FindAfterTimestampIMU(front_data.time_stamp){
//
//    }

    void GNSS_StatusCheck(std::deque<std::shared_ptr<BaseType>> _gnss_data_deque){
        static int cnt_test = 0;
        if(cnt_test > 100){
          //  EZLOG(INFO)<<"GNSS: "<< cnt_test;
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
        last_pose = PoseT(Eigen::Matrix4d::Identity());

        EZLOG(INFO)<<" Fuse Init Successful!";
    }

    void RollBack(const OdometryType _current_pose,
                  std::deque<std::shared_ptr<DROdometryType>> _DR_data_deque,
                  OdometryType &_current_pose_align){
        while(!_DR_data_deque.empty()){
            if(_current_pose.timestamp <_DR_data_deque.front()->timestamp){ // -------lidar | DR DR DR ------>>>>
                gtsam::Pose3 poseFrom(gtsam::Rot3(_DR_data_deque.front()->pose.GetR()),
                                                gtsam::Point3(_DR_data_deque.front()->pose.GetXYZ()));
                gtsam::Pose3 poseTo(gtsam::Rot3(_DR_data_deque.back()->pose.GetR()),
                                      gtsam::Point3(_DR_data_deque.back()->pose.GetXYZ()));

                _current_pose_align.frame = "map";
                _current_pose_align.timestamp = _DR_data_deque.back()->timestamp;
                _current_pose_align.pose = PoseT(_current_pose.pose.pose * poseFrom.between(poseTo).matrix());

                pubsub->PublishOdometry(topic_testforRollBack_pose, _current_pose_align);
                _DR_data_deque.clear();
                if_roll_back = 1;
                last_pose = _current_pose_align.pose;

                break;
            }
            else{
                _DR_data_deque.pop_front();
            }
        }
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        fuseInitialized();

        pubsub->addPublisher(topic_highHz_pose, DataType::ODOMETRY, 10);
        pubsub->addPublisher(topic_current_lidar_pose, DataType::ODOMETRY, 10);
        pubsub->addPublisher(topic_testforRollBack_pose, DataType::ODOMETRY, 10);
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
                        current_lidar_time = cur_lidar_odom->timestamp;
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
                        current_lidar_world.frame = "map";
                        current_lidar_world.timestamp = current_timeStamp;
                        current_lidar_world.pose.pose = current_pose.pose;
                        pubsub->PublishOdometry(topic_current_lidar_pose, current_lidar_world);

                        mutex_DR_data.lock();
                        RollBack(current_lidar_world, DR_data_deque, loc_result);
                        mutex_DR_data.unlock();

//                        factor
                        last_lidar_pose = current_lidar_pose;
                        lidar_keyFrame_cnt++;

                        break;
                    }

                    case DataType::GNSS:
                    {
                        // TODO  GNSS factor
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

                    case DataType::WHEEL:
                    {
                        TicToc t1;
                        // 1. get wheel data and save it to deque
                        DROdometryTypePtr  cur_Wheel_odom;
                        static PoseT last_DR_pose = PoseT(Eigen::Matrix4d::Identity());
                        cur_Wheel_odom = std::static_pointer_cast<DROdometryType>(std::move(front_data));
                        mutex_DR_data.lock();
                        DR_data_deque.push_back(cur_Wheel_odom);
                        mutex_DR_data.unlock();

                        // 2. cal dR of DR
                        gtsam::Pose3 current_Wheel_pose(gtsam::Rot3(cur_Wheel_odom->pose.GetR()),
                                                        gtsam::Point3(cur_Wheel_odom->pose.GetXYZ()));

                        PoseT current_DR_pose(current_Wheel_pose.translation(),current_Wheel_pose.rotation().matrix());
                        delta_pose_DR = last_DR_pose.between(current_DR_pose);

                        // 3.iteration of DR: update iteration init point when rollback
                        loc_result.frame = "map";
                        loc_result.timestamp = current_timeStamp;
                        if(if_roll_back){// predict from last rollback
                            loc_result.pose.pose = last_pose.pose * delta_pose_DR.pose;
                            last_pose = loc_result.pose.pose;
                        }else{ // normal predict
                            loc_result.pose.pose = last_DR_pose.pose * delta_pose_DR.pose;
                        }

                        // 4.iteration settings and pub the high frequency loc result
                        pubsub->PublishOdometry(topic_highHz_pose, loc_result);
//                        Function_AddLidarOdometryTypeToMapManager(loc_result);
                        if(MappingConfig::use_DR_or_fuse_in_loc == 0){
                            Function_AddLidarOdometryTypeToImageProjection(loc_result);
                        }
                        last_DR_pose = current_DR_pose;
                        EZLOG(INFO)<<"wheel pub cost in ms : "<<t1.toc();

                        break;
                    }
                }

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
