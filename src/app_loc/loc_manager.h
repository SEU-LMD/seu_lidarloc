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
#include "IMU_DR.h"
#include "utils/config_helper.h"
#include "opt_loc.h"
#include "fuse_info.h"
#include "map_loader.h"
//#include "map_engine/mapmanager.h"

class LocManager{
public:

    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    IMU_DR imu_pre;
    Fuse fuse;
    MapManager mapManager;



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

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        if(MappingConfig::if_LoadFromMapManager == 1){
            mapManager.Init(pubsub);
        }

        img_proj.Init(pubsub,1);
        ft_extr.Init(pubsub,1);
        loc_mapping.Init(pubsub, &mapManager);
        imu_pre.Init(pubsub,1);
        fuse.Init(pubsub);
        

        //构建数据流关系
//        auto add_imuodo_to_imgproj = std::bind(&ImageProjection::AddIMUOdomData, &img_proj,std::placeholders::_1);
        auto add_CloudInfo_from_imgproj_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_locmapping =
                std::bind(&LOCMapping::AddCloudData, &loc_mapping,std::placeholders::_1);
        auto add_OdometryType_from_locmapping_to_imupre =
                std::bind(&IMU_DR::AddOdomData_fromLoc, &imu_pre, std::placeholders::_1);
        if(MappingConfig::use_DR_or_fuse_in_loc == 0){ // use_DR_or_fuse_in_loc = 0, use fuse
            EZLOG(INFO)<<" use fuse to imageProj";
            auto add_OdometryType_from_fuse_to_imageProj =
                    std::bind(&ImageProjection::AddIMUOdomData, &img_proj, std::placeholders::_1);
            //        fuse 2 imageProjection
            fuse.Function_AddLidarOdometryTypeToImageProjection = add_OdometryType_from_fuse_to_imageProj;
        }
        else{ // use_DR_or_fuse_in_loc = 1 , use DR
            EZLOG(INFO)<<" use DR to imageProj";
            auto add_OdometryType_from_DR_to_imageProj =
                    std::bind(&ImageProjection::AddIMUOdomData, &img_proj, std::placeholders::_1);
            imu_pre.Function_AddDROdometryTypeToImageProjection = add_OdometryType_from_DR_to_imageProj;
        }

        auto add_OdometryType_from_fuse_to_mapManager =
                std::bind(&MapManager::AddLoctoMapManager, &mapManager,std::placeholders::_1);

//       to fuse
        auto add_GNSSOdometryType_from_imgproj_to_fuse =
                std::bind(&Fuse::AddGNSSToFuse, &fuse,std::placeholders::_1);
        auto add_LidarOdometryType_from_locmapping_to_fuse =
                std::bind(&Fuse::AddLidarLocToFuse, &fuse,std::placeholders::_1);
        auto add_DROdometryType_from_DR_to_fuse =
                std::bind(&Fuse::AddDRToFuse, &fuse, std::placeholders::_1);


//        img_proj 2 ft_extr
        img_proj.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_imgproj_to_ftextr;
//        ft_extr 2 Loc
        ft_extr.Function_AddCloudFeatureToLOCMapping = add_CloudFeature_from_ftextr_to_locmapping;
        //Loc 2 DR
        loc_mapping.Function_AddOdometryTypeToIMUPreintegration = add_OdometryType_from_locmapping_to_imupre;

//        img_proj 2 fuse
        img_proj.Function_AddGNSSOdometryTypeToFuse = add_GNSSOdometryType_from_imgproj_to_fuse;
//        Loc 2 fuse
        loc_mapping.Function_AddLidarOdometryTypeToFuse = add_LidarOdometryType_from_locmapping_to_fuse;
//        DR 2 fuse
        imu_pre.Function_AddDROdometryTypeToFuse = add_DROdometryType_from_DR_to_fuse;

    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
