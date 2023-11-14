/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  bodyReport.h Control中所使用的结构体,表征车辆状态
 */

#ifndef HAF_CONTROL_BODY_REPORT_H__
#define HAF_CONTROL_BODY_REPORT_H__

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafLightSwitch
        功能描述		:  车辆灯光信息
    ****************************************************************************** */
    struct HafLightSwitch {
        uint8_t turnLightSwitch;
        uint8_t hazardLightSwicth;
        uint8_t positionLightSwitch;
        uint8_t daytimeRunningLightSwitch;
        uint8_t highLowBeamSwitch;
        uint8_t fogFrontSwitch;
        uint8_t fogRearSwitch;
        uint8_t hornSwitch;
    };
    /* ******************************************************************************
        结构 名		:  HafModeSwitch
        功能描述		:  车辆模式信息
    ****************************************************************************** */
    struct HafModeSwitch {
        uint8_t autoDriveSwitch;
        uint8_t emergencySwitch;
        uint8_t apaSwitch;
        uint8_t lkaSwitch;
        uint8_t speedLimitSwitch;
        uint8_t accSwitchOnOff;
        uint8_t accSwitchResume;
        uint8_t accSwitchCancel;
        uint8_t accSwitchSpeedInc;
        uint8_t accSwitchSpeedDec;
        uint8_t accSwitchGapInc;
        uint8_t accSwitchGapDec;
    };
    /* ******************************************************************************
        结构 名		:  HafMirrorSwitch
        功能描述		:  车辆玻璃信息
    ****************************************************************************** */
    struct HafMirrorSwitch {
        uint8_t rearFlViewMirrorSwitch;
        uint8_t rearFrViewMirrorSwitch;
    };
    /* ******************************************************************************
        结构 名		:  HafWiper
        功能描述		:  车辆雨刮器信息
    ****************************************************************************** */
    struct HafWiper {
        uint8_t wiperValue;
        uint8_t wiperSpeedValue;
    };
    /* ******************************************************************************
        结构 名		:  HafPadSwitch
        功能描述		:  车辆中控屏信息
    ****************************************************************************** */
    struct HafPadSwitch {
        uint8_t up;
        uint8_t down;
        uint8_t left;
        uint8_t right;
        uint8_t ok;
    };
    /* ******************************************************************************
        结构 名		:  HafLightStatus
        功能描述		:  车辆灯光信息
    ****************************************************************************** */
    struct HafLightStatus {
        uint8_t turnLightStatus;
        uint8_t hazardLightStatus;
        uint8_t brakeLightStatus;
        uint8_t positionLightStatus;
        uint8_t reverseLightStatus;
        uint8_t daytimeRunningLightStatus;
        uint8_t ambientLightStatus;
        uint8_t highLowBeamStatus;
    };
    /* ******************************************************************************
        结构 名		:  HafDoorStatus
        功能描述		:  车辆车门信息
    ****************************************************************************** */
    struct HafDoorStatus {
        uint8_t doorFlStatus;
        uint8_t doorFrStatus;
        uint8_t doorRlStatus;
        uint8_t doorRrStatus;
        uint8_t doorHoodStatus;
        uint8_t doorTrunkStatus;
    };
    /* ******************************************************************************
        结构 名		:  HafWindowStatus
        功能描述		:  车辆车窗信息
    ****************************************************************************** */
    struct HafWindowStatus {
        uint8_t windowFlStatus;
        uint8_t windowFrStatus;
        uint8_t windowRlStatus;
        uint8_t windowRrStatus;
        uint8_t windowTopStatus;
    };
    /* ******************************************************************************
        结构 名		:  HafTyreStatus
        功能描述		:  车辆轮胎信息
    ****************************************************************************** */
    struct HafTyreStatus {
        float32_t tyrePressureValue;
        HafUint8WithValid tyrePressureLeakageStatus;
        HafUint8WithValid tyrePressureTirePStatus;
    };
    /* ******************************************************************************
        结构 名		:  HafSafetyStatus
        功能描述		:  车辆安全信息
    ****************************************************************************** */
    struct HafSafetyStatus {
        uint8_t safeBeltDriverStatus;
        uint8_t safeBeltPassengerFrStatus;
        uint8_t safeBeltPassengerRlStatus;
        uint8_t safeBeltPassengerRmStatus;
        uint8_t safeBeltPassengerRrStatus;
        HafUint8WithValid collisionEvent;
    };
    /* ******************************************************************************
        结构 名		:  HafBodyReport
        功能描述		:  车辆车身信息
    ****************************************************************************** */
    struct HafBodyReport {
        HafLightSwitch lightSwitch;
        HafModeSwitch modeSwitch;
        HafMirrorSwitch mirrorSwitch;
        HafWiper wiperFrontSwitch;
        HafWiper wiperRearSwitch;
        HafWiper wiperFrontStatus;
        HafWiper wiperRearStatus;
        HafPadSwitch padSwitch;
        HafLightStatus lightStatus;
        HafDoorStatus doorStatus;
        HafWindowStatus windowStatus;
        HafTyreStatus tyreFlStatus;
        HafTyreStatus tyreFrStatus;
        HafTyreStatus tyreRlStatus;
        HafTyreStatus tyreRrStatus;
        HafSafetyStatus safetyStatus;
        HafHeader bodyReportHeader;
    };
}

#endif // HAF_CONTROL_BODY_REPORT_H__
