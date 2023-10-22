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

class LocManager{
public:
    PubSubInterface* pubsub;

    ImageProjection img_proj;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    IMU_DR imu_pre;
    Fuse fuse;

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
        img_proj.Init(pubsub);
        ft_extr.Init(pubsub);
        loc_mapping.Init(pubsub);
        imu_pre.Init(pubsub);


//        fuse.Init(pubsub);


        //构建数据流关系
//        auto add_imuodo_to_imgproj = std::bind(&ImageProjection::AddIMUOdomData, &img_proj,std::placeholders::_1);
        auto add_CloudInfo_from_imgproj_to_ftextr =
                std::bind(&FeatureExtraction::AddCloudData, &ft_extr,std::placeholders::_1);
        auto add_CloudFeature_from_ftextr_to_locmapping =
                std::bind(&LOCMapping::AddCloudData, &loc_mapping,std::placeholders::_1);
        auto add_OdometryType_from_locmapping_to_imupre =
                std::bind(&IMU_DR::AddOdomData, &imu_pre, std::placeholders::_1);
        auto add_OdometryType_from_imupre_to_imgproj =
                std::bind(&ImageProjection::AddIMUOdomData, &img_proj,std::placeholders::_1);

//       to fuse
        auto add_GNSSOdometryType_from_imgproj_to_fuse =
                std::bind(&Fuse::AddGNSSToFuse, &fuse,std::placeholders::_1);
        auto add_LidarOdometryType_from_locmapping_to_fuse =
                std::bind(&Fuse::AddLidarLocToFuse, &fuse,std::placeholders::_1);
        auto add_IMUOdometryType_from_imupre_to_fuse =
                std::bind(&Fuse::AddIMUToFuse, &fuse,std::placeholders::_1);

        img_proj.Function_AddCloudInfoToFeatureExtraction = add_CloudInfo_from_imgproj_to_ftextr;
        ft_extr.Function_AddCloudFeatureToLOCMapping = add_CloudFeature_from_ftextr_to_locmapping;
        loc_mapping.Function_AddOdometryTypeToIMUPreintegration = add_OdometryType_from_locmapping_to_imupre;
//        imu_pre.Function_AddOdometryTypeToImageProjection = add_OdometryType_from_imupre_to_imgproj;

//        to fuse
        img_proj.Function_AddGNSSOdometryTypeToFuse = add_GNSSOdometryType_from_imgproj_to_fuse;
        loc_mapping.Function_AddLidarOdometryTypeToFuse = add_LidarOdometryType_from_locmapping_to_fuse;
        imu_pre.Function_AddIMUOdometryTypeToFuse = add_IMUOdometryType_from_imupre_to_fuse;

    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
