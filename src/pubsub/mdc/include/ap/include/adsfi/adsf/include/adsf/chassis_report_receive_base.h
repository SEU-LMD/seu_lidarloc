/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  ChassisReportReceiveBase.cpp 底盘信息的AP消息接收
 */

#ifndef HAF_ADSF_CHASSIS_REPORT_RECEIVEBASE_H
#define HAF_ADSF_CHASSIS_REPORT_RECEIVEBASE_H

#include <shared_mutex>
#include "ara/chassis/chassisreportserviceinterface_proxy.h"
#include "data_receive_base.h"
#include "core/types.h"
#include "control/chassis_report.h"

namespace Adsfi {
    class ChassisReportReceiveBase
        : public DataReceiveBase<ara::chassis::proxy::ChassisReportServiceInterfaceProxy,
            ara::chassis::proxy::ChassisReportServiceInterfaceProxy::HandleType, HafChassisReport> {
    public:
        explicit ChassisReportReceiveBase(const uint32_t instanceID) : DataReceiveBase(instanceID){};
        template <typename T1, typename T2>
        static void ChassisReportHeaderTranslate(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void ChassisReportUint8WithValidTranslate(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void ChassisReportFloat32WithValidTranslate(const T1 &fromHeader, T2 &endHeader);
        template <typename T1, typename T2>
        static void ChassisReportActuatorWorkStatusTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportActuatorStatusTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportSteerInfoTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportWheelSpeedTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportBrakeInfoTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportBrakeMoreInfoTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportThrottleInfoTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportGearInfoTransLate(const T1 &fromPoint, T2 &endPoint);
        template <typename T1, typename T2>
        static void ChassisReportVehicleMotionTransLate(const T1 &fromPoint, T2 &endPoint);
        void RegisterHandle() override;
        void OnDataReceive();
        virtual ~ChassisReportReceiveBase();
    };
}
#endif // HAF_ADSF_VEHICLE_INFO_RECEIVEBASE_H