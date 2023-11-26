//
// Created by fyy on 23-9-9.
//

#ifndef SEU_LIDARLOC_DRCALIB_MANAGER_H
#define SEU_LIDARLOC_DRCALIB_MANAGER_H
#include <thread>
#include <mutex>
#include <deque>

#include "pubsub/pubusb.h"
#include "data_preprocess.h"
#include "feature_extraction.h"
#include "utils/config_helper.h"
#include "opt_loc.h"
#include "fuse_info.h"
#include "dr_calib.h"

class DRCalibManager{
public:
    PubSubInterface* pubsub;

    DataPreprocess data_prep;
    FeatureExtraction ft_extr;
    LOCMapping loc_mapping;
    Fuse fuse;
    DRCalibration dr_calib;

    void GNSSINSCallback(const BaseType& msg){
        const GNSSINSType& gnssins_data = *((GNSSINSType*)&msg);
        dr_calib.AddGNSSINSData(gnssins_data);
    }

//    void DrCallback(const BaseType& msg){
//        const WheelType& dr_data = *((WheelType*)&msg);
//        dr_calib.AddDrData(dr_data);
//    }

    void Init(PubSubInterface* pubsub_){
        pubsub = pubsub_;

        //然后开启各个线程
        dr_calib.Init(pubsub);
    }
};

#endif //SEU_LIDARLOC_LOC_MANAGER_H
