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

    DataPreprocess img_proj;
    FeatureExtraction ft_extr;
    imu_wheel_dr imu_pre;
    OPTMapping opt_mapping;
   // MapManager mapManager;


    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        img_proj.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
//        TicToc timer;
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        img_proj.AddGNSSINSSData(gnssins_data);
     //   opt_mapping.AddGNSSINSData(gnssins_data);
        imu_pre.AddGNSSINSData(gnssins_data);
        opt_mapping.AddGNSSINSData(gnssins_data);
//        EZLOG(INFO)<<"GNSSINSCallback cost time(ms)"<<timer.toc()<<std::endl;
    }


    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;
       // udp_thread =udp_thread_;
        //然后开启各个线程
        img_proj.Init(pubsub,0);
        EZLOG(INFO)<< "img_proj success!!"<<endl;
        ft_extr.Init(pubsub,0);
        EZLOG(INFO)<< "ft_extr success!!"<<endl;
        opt_mapping.Init(pubsub);
        EZLOG(INFO)<< "opt_mapping success!!"<<endl;
        imu_pre.Init(pubsub,0);
        EZLOG(INFO)<< "imu_pre success!!"<<endl;
      //  mapManager.Init(pubsub);

        //构建数据流关系
        auto add_CloudInfo_from_imgproj_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_optmapping =
                std::bind(&OPTMapping::AddCloudData, &opt_mapping,std::placeholders::_1);

      //  auto add_GNSSOdometryType_from_imgproj_to_optmapping =
       //         std::bind(&OPTMapping::AddGNSSToOpt, &opt_mapping,std::placeholders::_1);
        auto add_DROdometryType_from_imupre_to_imgproj =
                std::bind(&DataPreprocess::AddDrOdomData, &img_proj,std::placeholders::_1);


        img_proj.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_imgproj_to_ftextr;

        ft_extr.Function_AddCloudFeatureToOPTMapping = add_CloudFeature_from_ftextr_to_optmapping;
        //img_proj.Function_AddGNSSOdometryTypeToOPTMapping = add_GNSSOdometryType_from_imgproj_to_optmapping;
        imu_pre.Function_AddDROdometryTypeToImageProjection = add_DROdometryType_from_imupre_to_imgproj;
        EZLOG(INFO) << "Init finish!!! " << std::endl;
    }
};
#endif //SEU_LIDARLOC_MAPPING_MANAGER_H
