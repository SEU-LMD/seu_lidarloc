//
// Created by fyy on 23-10-12.
//

#ifndef SEU_LIDARLOC_FUSE_H
#define SEU_LIDARLOC_FUSE_H
#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
#include "utils/timer.h"

class Fuse{
public:
    PubSubInterface* pubsub;
    std::thread* HighFrequencyLoc_thread;
    int data_deque_max_size = 200;
    int front_index = 0;
    std::deque<std::shared_ptr<BaseType>> data_deque;
    std::deque<std::shared_ptr<IMUOdometryType>> imu_data_deque;
    std::deque<std::shared_ptr<OdometryType>> lidar_data_deque;
    std::deque<std::shared_ptr<GNSSOdometryType>> gnss_data_deque;
    std::mutex mutex_data;
    gtsam::PreintegratedImuMeasurements *imuIntegrator;//just use imu to predict

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

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
//        pubsub->addPublisher(topic_highFequency_odom, DataType::ODOMETRY, 10);
        HighFrequencyLoc_thread = new std::thread(&Fuse::DoWork, this);
    }

    void DoWork(){
        EZLOG(INFO)<<"Fuse init success!"<<std::endl;
        while(1){
            if(data_deque.size()!=0){
                OdometryType send_data;//final send data

                mutex_data.lock();
                auto front_data = data_deque[front_index];
//                std::vector<GNSS_INSType> re_predict_imu_data;
                switch (front_data->getType()) {
                    case DataType::ODOMETRY:
                        EZLOG(INFO)<<"GET Lidar!";
//                        OdometryTypePtr lidar_data;
//                        lidar_data_deque.push_back(front_data);

//                        factor
                        front_index++;
                        mutex_data.unlock();
                        break;
                    case DataType::GNSS:
                        EZLOG(INFO)<<"GET GNSS!";
//                        prior factor
                        front_index++;
                        mutex_data.unlock();
                        break;
                    case DataType::IMU:
                        EZLOG(INFO)<<"GET IMU!";
//                        predict imu pose
                        front_index++;
                        mutex_data.unlock();
                        break;

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
                    front_index--;
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
