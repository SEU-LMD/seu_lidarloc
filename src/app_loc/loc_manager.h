//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_LOC_MANAGER_H
#define SEU_LIDARLOC_LOC_MANAGER_H
#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "data_preprocess.h"
#include "feature_extraction.h"
#include "imu_wheel_dr.h"
#include "utils/config_helper.h"
#include "opt_loc.h"
#include "fuse_info.h"
#include "map_loader.h"

#include "utils/udp_thread.h"  //add udp

class LocManager{
public:
    PubSubInterface* pubsub;

    DataPreprocess data_prep;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    imu_wheel_dr imu_wheeldr;
    Fuse fuse;
    MapManager mapManager;

    std::shared_ptr<UDP_THREAD> udp_thread;


    void CloudCallback(const BaseType& msg){
        const CloudTypeXYZIRT& cloud_data = *((CloudTypeXYZIRT*)&msg);
        data_prep.AddCloudData(cloud_data);
    }

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        data_prep.AddGNSSINSSData(gnssins_data);
        imu_wheeldr.AddGNSSINSData(gnssins_data);
    }

    void Init(PubSubInterface* pubsub_,std::shared_ptr<UDP_THREAD> udp_thread_){
        pubsub = pubsub_;
        udp_thread =udp_thread_;
        //然后开启各个线程
        // 将data_prep to map_manager
        data_prep.Init(pubsub, udp_thread,1);//remove magic number!!!!! TODO 1111
        ft_extr.Init(pubsub,1);
        loc_mapping.Init(pubsub,&mapManager,udp_thread);
        imu_wheeldr.Init(pubsub, udp_thread, 1);
        fuse.Init(pubsub,udp_thread);
        mapManager.Init(pubsub);

        //构建数据流关系
//        auto add_imuodo_to_imgproj = std::bind(&DataPreprocess::AddIMUOdomData, &data_prep,std::placeholders::_1);
        auto add_CloudInfo_from_imgproj_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_locmapping =
                std::bind(&LOCMapping::AddCloudData, &loc_mapping,std::placeholders::_1);

        if(MappingConfig::use_DR_or_fuse_in_loc == 0){ // use_DR_or_fuse_in_loc = 0, use fuse
            EZLOG(INFO)<<" use fuse to imageProj";
            auto add_OdometryType_from_fuse_to_imageProj =
                    std::bind(&DataPreprocess::AddDrOdomData, &data_prep, std::placeholders::_1);
            //        fuse 2 imageProjection
            fuse.Function_AddLidarOdometryTypeToImageProjection = add_OdometryType_from_fuse_to_imageProj;
        }
        else{ // use_DR_or_fuse_in_loc = 1 , use DR
            EZLOG(INFO)<<" use DR to imageProj";
            auto add_OdometryType_from_DR_to_imageProj =
                    std::bind(&DataPreprocess::AddDrOdomData, &data_prep, std::placeholders::_1);
            imu_wheeldr.Function_AddDROdometryTypeToDataPreprocess = add_OdometryType_from_DR_to_imageProj;
        }

//       to fuse
        auto add_GNSSOdometryType_from_imgproj_to_fuse =
                std::bind(&Fuse::AddGNSSToFuse, &fuse,std::placeholders::_1);
        auto add_LidarOdometryType_from_locmapping_to_fuse =
                std::bind(&Fuse::AddLidarLocToFuse, &fuse,std::placeholders::_1);
        auto add_DROdometryType_from_DR_to_fuse =
                std::bind(&Fuse::AddIMUToFuse, &fuse,std::placeholders::_1);
        auto add_OdometryType_from_fuse_to_mapManager =
                std::bind(&MapManager::AddLoctoMapManager, &mapManager,std::placeholders::_1);

//        data_prep 2 ft_extr
        data_prep.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_imgproj_to_ftextr;
//        ft_extr 2 Loc
        ft_extr.Function_AddCloudFeatureToLOCMapping = add_CloudFeature_from_ftextr_to_locmapping;

//        data_prep 2 fuse
        data_prep.Function_AddGNSSOdometryTypeToFuse = add_GNSSOdometryType_from_imgproj_to_fuse;
//        Loc 2 fuse
        loc_mapping.Function_AddLidarOdometryTypeToFuse = add_LidarOdometryType_from_locmapping_to_fuse;
//        DR 2 fuse
        imu_wheeldr.Function_AddDROdometryTypeToFuse = add_DROdometryType_from_DR_to_fuse;
        // fuse 2 MapManager
        fuse.Function_AddLidarOdometryTypeToMapManager = add_OdometryType_from_fuse_to_mapManager;
//      img 2 DR load once
//        auto add_firstGNSSPoint2DR = std::bind(&imu_wheel_dr::AddFirstGNSSPoint, &imu_wheeldr,std::placeholders::_1);
//        data_prep.Function_AddFirstGNSSPoint2DR = add_firstGNSSPoint2DR;

    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
