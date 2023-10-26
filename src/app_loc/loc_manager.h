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
#include "IMU_DR.h"
#include "utils/config_helper.h"
#include "opt_lopc.h"
#include "fuse.h"
#include "mapManager.h"

class LocManager{
public:
    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    IMU_DR DR;
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
        DR.AddGNSSINSData(gnssins_data);
    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        img_proj.Init(pubsub);
        ft_extr.Init(pubsub);
        loc_mapping.Init(pubsub);
        DR.Init(pubsub);
        fuse.Init(pubsub);
        mapManager.Init(pubsub);


        //构建数据流关系
//        auto add_imuodo_to_imgproj = std::bind(&ImageProjection::AddIMUOdomData, &img_proj,std::placeholders::_1);
        auto add_CloudInfo_from_imgproj_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_locmapping =
                std::bind(&LOCMapping::AddCloudData, &loc_mapping,std::placeholders::_1);
        auto add_OdometryType_from_fuse_to_mapManager =
                std::bind(&MapManager::AddLoctoMapManager, &mapManager,std::placeholders::_1);
        auto add_PriorMap_from_mapManager_to_Loc =
                std::bind(&LOCMapping::AddPriorMap, &loc_mapping,std::placeholders::_1);

//       to fuse
        auto add_GNSSOdometryType_from_imgproj_to_fuse =
                std::bind(&Fuse::AddGNSSToFuse, &fuse,std::placeholders::_1);
        auto add_LidarOdometryType_from_locmapping_to_fuse =
                std::bind(&Fuse::AddLidarLocToFuse, &fuse,std::placeholders::_1);
        auto add_DROdometryType_from_DR_to_fuse =
                std::bind(&Fuse::AddIMUToFuse, &fuse,std::placeholders::_1);

//        img_proj 2 ft_extr
        img_proj.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_imgproj_to_ftextr;
//        ft_extr 2 Loc
        ft_extr.Function_AddCloudFeatureToLOCMapping = add_CloudFeature_from_ftextr_to_locmapping;
//        mapManager 2 Loc
        mapManager.Function_AddPriorMapToLoc = add_PriorMap_from_mapManager_to_Loc;


//        img_proj 2 fuse
        img_proj.Function_AddGNSSOdometryTypeToFuse = add_GNSSOdometryType_from_imgproj_to_fuse;
//        Loc 2 fuse
        loc_mapping.Function_AddLidarOdometryTypeToFuse = add_LidarOdometryType_from_locmapping_to_fuse;
//        DR 2 fuse
        DR.Function_AddDROdometryTypeToFuse = add_DROdometryType_from_DR_to_fuse;
//        fuse 2 mapManager
        fuse.Function_AddLidarOdometryTypeToMapManager = add_OdometryType_from_fuse_to_mapManager;

    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
