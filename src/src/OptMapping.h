//
// Created by fyy on 23-9-10.
//

#ifndef SEU_LIDARLOC_OPTMAPPING_H
#define SEU_LIDARLOC_OPTMAPPING_H

#include "pubsub/pubusb.h"
#include "pubsub/data_types.h"
class OPTMapping{
public:
    PubSubInterface* pubsub;
    std::thread* do_work_thread;
    std::mutex cloud_mutex;
    std::mutex gnss_ins_mutex;

    std::deque<CloudFeature> deque_cloud;
    std::deque<GNSSINSType> deque_gnssins;


    void AddCloudData(const CloudFeature& cloud_ft){
        cloud_mutex.lock();
        deque_cloud.push_back(cloud_ft);
        cloud_mutex.unlock();

    }

    void AddGNSSINSData(const GNSSINSType& gnss_ins_data){
        gnss_ins_mutex.lock();
        deque_gnssins.push_back(gnss_ins_data);
        gnss_ins_mutex.unlock();
    }

    void DoWork(){
//        内存初始化
        while(1){
            if(deque_cloud.size()==0){
                CloudFeature cur_ft;
                cloud_mutex.lock();
                cur_ft = deque_cloud.front();
                deque_cloud.pop_front();
                cloud_mutex.unlock();

                //just do something
//                code

            }
        }
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
//        填发布
//        pubsub->addPublisher(topic_corner_world, DataType::LIDAR, 10);
//        pubsub->addPublisher(topic_surf_world, DataType::LIDAR, 10);
        do_work_thread = new std::thread(&OPTMapping::DoWork, this);
    }
};
#endif //SEU_LIDARLOC_OPTMAPPING_H
