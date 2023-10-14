//
// Created by fyy on 23-10-12.
//

#ifndef SEU_LIDARLOC_FUSE_H
#define SEU_LIDARLOC_FUSE_H
//#include "pubsub/pubusb.h"
//#include "pubsub/data_types.h"
//#include "utils/timer.h"
//
//class Fuse{
//public:
//    int data_deque_max_size = 200;
//    int front_index = 0;
//    std::deque<BaseType> data_deque;
//    std::mutex mutex_data;
//    gtsam::PreintegratedImuMeasurements *imuIntegrator;//just use imu to predict
//
//
//    //this fucntion needs to be binded by lidar loc node
//    void AddLidarLocToFuse(const& OdometryType lidar_loc_res){
//        mutex_data.lock();
//        data_deque.push_back(lidar_loc_res);
//        mutex_data.unlock();
//    }
//
//
//    voiid AddGNSSINSToFuse(const& GNSS_INSType gnss_ins_data){
//        mutex_data.lock();
//        data_deque.push_back(gnss_ins_data);
//        mutex_data.unlock();
//    }
//
//    std::vector<GNSS_INSType> = FindAfterTimestampIMU(front_data.time_stamp){
//
//    }
//
//    void DoWork(){
//        while(1){
//            if(data_deque.size()!=0){
//                OdometryType send_data;//final send data
//
//                mutex_data.lock();
//                auto front_data = data_deque[front_index];
//                std::vector<GNSS_INSType> re_predict_imu_data;
//                if(front_data.getType()==DataType:ODOMETRY){
//                    re_predict_imu_data = FindAfterTimestampIMU(front_data.time_stamp);
//
//                }
//                front_index++;
//                mutex_data.unlock();
//
//                if(front_data.getType()==DataType::GNSS_INS){
//                    //just use imu data to predcict
//
//
//                }
//                else if(front_data.getType()==DataType:ODOMETRY){
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
//
//
//                //finally send data to middle ware use send_data
//
//
//                mutex_data.lock();
//                while(data_deque.size()>=data_deque_max_size){
//                    data_deque.pop_front();
//                    front_index--;
//                }
//                mutex_data.unlock();
//            }
//            else{
//                sleep(0.001);
//            }
//        }
//    }//end fucntion do work
//
//
//};
#endif //SEU_LIDARLOC_FUSE_H
