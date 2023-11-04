//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_DATACHECK_MANAGER_H
#define SEU_LIDARLOC_DATACHECK_MANAGER_H

#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "data_preprocess.h"
#include "feature_extraction.h"
#include "imu_wheel_dr.h"
#include "opt_mapping.h"
#include "utils/config_helper.h"

#include "data_check.h"

class DataCheckManager{
public:
    PubSubInterface* pubsub;

    DataCheck data_check;

    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        data_check.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        data_check.AddGNSSINSData(gnssins_data);
    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        data_check.Init(pubsub);
//        EZLOG(INFO)<< "img_proj success!!"<<endl;

        //构建数据流关系
        EZLOG(INFO) << " DataCheck Init Finish!!! " << std::endl;
    }
};
#endif //SEU_LIDARLOC_MAPPING_MANAGER_H
