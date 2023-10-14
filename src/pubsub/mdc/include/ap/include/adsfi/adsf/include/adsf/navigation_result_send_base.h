/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  NavigationResultSendBase.h NavigationResult发送数据
 */

#ifndef HAF_ADSF_NAVIGATION_RESULT_SEND_BASE_H
#define HAF_ADSF_NAVIGATION_RESULT_SEND_BASE_H

#include <shared_mutex>
#include "mdc/navigation/navigationresultserviceinterface_skeleton.h"
#include "data_send_base.h"
#include "core/types.h"
#include "navigation/navigation_result.h"

namespace Adsfi {
    class NavigationResultSendBase
        : public DataSendBase<mdc::navigation::skeleton::NavigationResultServiceInterfaceSkeleton,
                 HafNavigationResult> {
    public:
        explicit NavigationResultSendBase(const uint32_t idx) : DataSendBase(idx){};
        ~NavigationResultSendBase() override;
        void SendingData() override;
    private:
        static void NavSendLanePieceTranslate(ara::navigation::LanePiece &lanePiece,
                                              const HafLanePiece &hafLanePieceFrom);
        static void RoadPieceTranslate(ara::navigation::RoadPiece &roadPiece, const HafRoadPiece &hafRoadPieceFrom);
        template <typename T1, typename T2>
        static void AssignData(const T1& srcData, T2& desData);
    };
}

#endif // HAF_ADSF_NAVIGATION_RESULT_SEND_BASE_H
