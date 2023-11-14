//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_MAPPING_MANAGER_H
#define SEU_LIDARLOC_MAPPING_MANAGER_H

#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "data_preprocess.h"
#include "feature_extraction.h"
#include "imu_wheel_dr.h"
#include "opt_mapping.h"
#include "utils/config_helper.h"
//#include "dead_reckoning.h"

class MappingManager{
public:
    PubSubInterface* pubsub;

    DataPreprocess data_pre;
    FeatureExtraction ft_extr;
    imu_wheel_dr imu_wheeldr;//TODO 1111 change name to dr
    OPTMapping opt_mapping;
   // MapManager mapManager;


    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        data_pre.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
//        TicToc timer;
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        data_pre.AddGNSSINSSData(gnssins_data);
     //TODO 1111 remove
        imu_wheeldr.AddGNSSINSData(gnssins_data);
    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
        //然后开启各个线程
        data_pre.Init(pubsub,0);//TODO 1111 read different config file
        EZLOG(INFO)<< "img_proj success!!"<<endl;
        ft_extr.Init(pubsub,0);
        EZLOG(INFO)<< "ft_extr success!!"<<endl;
        opt_mapping.Init(pubsub);
        EZLOG(INFO)<< "opt_mapping success!!"<<endl;
        imu_wheeldr.Init(pubsub,0);
        EZLOG(INFO)<< "imu_pre success!!"<<endl;

        //构建数据流关系
        //imgproj 1111 chanage name
        auto add_CloudInfo_from_datapre_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_optmapping =
                std::bind(&OPTMapping::AddCloudData, &opt_mapping,std::placeholders::_1);

        auto add_DROdometryType_from_DR_to_imgproj =
                std::bind(&DataPreprocess::AddDrOdomData, &data_pre,std::placeholders::_1);


        data_pre.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_datapre_to_ftextr;
        ft_extr.Function_AddCloudFeatureToOPTMapping = add_CloudFeature_from_ftextr_to_optmapping;
        imu_wheeldr.Function_AddDROdometryTypeToDataPreprocess = add_DROdometryType_from_DR_to_imgproj;
        EZLOG(INFO) << "Init finish!!! " << std::endl;
    }
};
#endif //SEU_LIDARLOC_MAPPING_MANAGER_H
