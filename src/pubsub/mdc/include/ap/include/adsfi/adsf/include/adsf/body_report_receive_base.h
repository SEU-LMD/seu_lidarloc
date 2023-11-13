/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  BodyReportReceiveBase.h
 */

#ifndef HAF_ADSF_BODY_REPORT_RECEIVEBASE_H
#define HAF_ADSF_BODY_REPORT_RECEIVEBASE_H

#include <shared_mutex>
#include "ara/body/bodyreportserviceinterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "control/body_report.h"

namespace Adsfi {
class BodyReportReceiveBase : public DataReceiveBase<ara::body::proxy::BodyReportServiceInterfaceProxy,
    ara::body::proxy::BodyReportServiceInterfaceProxy::HandleType, HafBodyReport> {
public:
    explicit BodyReportReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID) {};
    template <typename T1, typename T2> static void BodyReportHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
    template <typename T1, typename T2>
    static void BodyReportUint8WithValidTranslate(const T1 &fromHeader, T2 &endHeader);
    template <typename T1, typename T2> static void BodyReportLightSwitchTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportModSwitchTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportMirrorSwitchTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportWiperTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportPadSwitchTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportLightStatusTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportDoorStatusTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportWindowStatusTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportTyreStatusTransLate(const T1 &fromPoint, T2 &endPoint);
    template <typename T1, typename T2> static void BodyReportSafetyStatusTransLate(const T1 &fromPoint, T2 &endPoint);
    void RegisterHandle() override;
    void OnDataReceive();
    virtual ~BodyReportReceiveBase();
};
}
#endif // HAF_ADSF_VEHICLE_INFO_RECEIVEBASE_H
