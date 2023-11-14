/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  NavigationResultReceiveBase.h 规控导航结果接收
 */


#ifndef HAF_ADSF_NAVIGATION_RESULT_RECEIVE_BASE_H__
#define HAF_ADSF_NAVIGATION_RESULT_RECEIVE_BASE_H__

#include <shared_mutex>
#include "mdc/navigation/navigationresultserviceinterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "navigation/navigation_result.h"

namespace Adsfi {
    class NavigationResultReceiveBase :
        public DataReceiveBase<mdc::navigation::proxy::NavigationResultServiceInterfaceProxy,
        mdc::navigation::proxy::NavigationResultServiceInterfaceProxy::HandleType, HafNavigationResult> {
    public:
        explicit NavigationResultReceiveBase(const uint32_t instanceIdx)
            : DataReceiveBase(instanceIdx){};
        virtual ~NavigationResultReceiveBase();
        void RegisterHandle() override;
        void OnDataReceive();
    private:
        static void HafRoadPieceTranslate(HafRoadPiece &navigationRoadPiece,
                                          const ara::navigation::RoadPiece &roadPiece);
        template <typename T1, typename T2>
        static void NavReceiveLanePieceTranslate(const T1 &fromPoint, T2 &endPoint);
    };
}
#endif // HAF_ADSF_NAVIGATION_RESULT_RECEIVE_BASE_H__