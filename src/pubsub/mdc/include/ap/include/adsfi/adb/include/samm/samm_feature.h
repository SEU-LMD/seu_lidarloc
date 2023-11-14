/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  samm_feature_pub.h samm场景分析模块发送场景特征的信息
 * Create: 2020-09-04
 */
#ifndef HAF_SAMM_SAMM_FEATURE_H
#define HAF_SAMM_SAMM_FEATURE_H

#include <string>
#include <vector>
#include "core/types.h"
namespace Adsfi {
    struct HafFeatureElement {
        uint32_t id;
        std::string name;
        std::string description;
    };

    struct HafSammFeature {
        HafHeader header;
        std::vector<HafFeatureElement> defaultFeatures;
        std::vector<HafFeatureElement> customFeatures;
    };

    struct HafMdcFaultEvent {
        uint16_t eventId;
        std::string actionName;
        uint64_t firstTimeStamp;
        uint64_t lastTimeStamp;
    };
}
#endif // HAF_SAMM_SAMM_FEATURE_H