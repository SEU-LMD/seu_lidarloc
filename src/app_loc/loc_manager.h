//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_LOC_MANAGER_H
#define SEU_LIDARLOC_LOC_MANAGER_H
#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "imageProjection.h"
#include "featureExtraction.h"
#include "imu_preintegration.h"
#include "utils/config_helper.h"
#include "LocMapping.h"

class LocManager{
public:
    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    IMUPreintegration imu_pre;
//    IMUOptimization imu_opt;


    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        img_proj.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        img_proj.AddGNSSINSSData(gnssins_data);
        loc_mapping.AddGNSSINSData(gnssins_data);
        imu_pre.AddGNSSINSData(gnssins_data);
    }

////    sub lidar odom, save in imu_pre and imu_opt
//    void OdomCallback(const BaseType& msg){
//        const OdometryType& odom_data = *((OdometryType*)&msg);
//        imu_pre.AddOdomData(odom_data);
////        imu_opt.AddOdomData(odom_data);
//    }

//    sub imu_pre, save in imu_opt
//    void imuCallback(const BaseType& msg){
//        const OdometryType& odom_data = *((OdometryType*)&msg);
////        imu_opt.AddOdomData(odom_data);
//
//    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        img_proj.Init(pubsub);
        ft_extr.Init(pubsub);
        loc_mapping.Init(pubsub);
        imu_pre.Init(pubsub);
//        imu_opt.Init(pubsub);

        //构建数据流关系
        img_proj.ft_extr_ptr = &ft_extr;
        ft_extr.loc_mapping_ptr = &loc_mapping;
        loc_mapping.imu_pre_ptr = &imu_pre;
//        imu_pre.imu_opt_ptr = &imu_opt;
    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
