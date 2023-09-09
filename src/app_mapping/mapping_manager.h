//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_MAPPING_MANAGER_H
#define SEU_LIDARLOC_MAPPING_MANAGER_H

#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "imageProjection_gnss.h"
#include "featureExtraction.h"
#include "utils/config_helper.h"

class MappingManager{
public:
    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;

//    std::mutex cloud_mutex, gnssins_mutex;
//    std::deque<CloudType> deque_cloud;
//    std::deque<GNSSINSType> deque_gnssins;

    void CloudCallback(const BaseType& msg){
        const CloudType& cloud_data = *((CloudType*)&msg);
        img_proj.addCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
//        deque_gnssins.push_back(gnssins_data);
    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        img_proj.init(pubsub);
        ft_extr.init(pubsub);

        //构建数据流关系
        img_proj.ft_extr_ptr = &ft_extr;
    }
};
#endif //SEU_LIDARLOC_MAPPING_MANAGER_H
