//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_MAPPING_MANAGER_H
#define SEU_LIDARLOC_MAPPING_MANAGER_H

#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "imageProjection.h"
#include "featureExtraction.h"
#include "utils/config_helper.h"
#include "opt_mapping.h"

//#include "dead_reckoning.h"

class MappingManager{
public:
    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;
//    DeadReckoning dead_reckoning;
    OPTMapping opt_mapping;
    IMUPreintegration imu_pre;

    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        img_proj.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        img_proj.AddGNSSINSSData(gnssins_data);
        opt_mapping.AddGNSSINSData(gnssins_data);
        imu_pre.AddGNSSINSData(gnssins_data);
//        dead_reckoning.AddGNSSINSSData(gnssins_data);
    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        img_proj.Init(pubsub);
        EZLOG(INFO)<< "img_proj success!!"<<endl;
        ft_extr.Init(pubsub);
        EZLOG(INFO)<< "ft_extr success!!"<<endl;
        opt_mapping.Init(pubsub);
        EZLOG(INFO)<< "opt_mapping success!!"<<endl;
        imu_pre.Init(pubsub);
        EZLOG(INFO)<< "imu_pre success!!"<<endl;
        //构建数据流关系
        img_proj.ft_extr_ptr = &ft_extr;
//        img_proj.opt_mapping_ptr = &opt_mapping;
        ft_extr.opt_mapping_ptr = &opt_mapping;
        opt_mapping.imu_pre_ptr = &imu_pre;
        //opt_mapping.im
    }
};
#endif //SEU_LIDARLOC_MAPPING_MANAGER_H
