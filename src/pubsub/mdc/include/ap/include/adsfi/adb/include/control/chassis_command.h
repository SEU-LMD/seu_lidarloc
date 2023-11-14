/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  chassisCommand.h Control中所使用的结构体,表征车辆状态
 */

#ifndef HAF_CONTROL_CHASSIS_COMMAND_H__
#define HAF_CONTROL_CHASSIS_COMMAND_H__

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafChassisCommand
        功能描述		:  车辆底盘命令信息
    ****************************************************************************** */
    struct HafChassisCommand {
        HafGear gearCmd;
        uint8_t source;
        uint16_t moduleID;
        uint16_t adasFunction;
        uint16_t adasWorkStatus;
        float32_t steerAngleCmd;
        bool steerAngleValid;
        uint8_t steerAngleState;
        float32_t steerAngleMaxLimit;
        float32_t steerAngleRateMaxLimit;
        uint8_t steerAngleMode;
        float32_t steerTorqueSmoothFactor;
        float32_t steerTorqueCmd;
        bool steerTorqueValid;
        uint8_t steerTorqueState;
        float32_t steerTorqueMaxLimit;
        uint8_t steerTorqueMode;
        uint8_t steerVibrationCmd;
        uint8_t steerVibrationState;
        float32_t steerCrvtCmd;
        bool steerCrvtValid;
        uint8_t steerCrvtState;
        float32_t steerCrvtGrdCmd;
        float32_t steerFactorSstyCmd;
        uint8_t steerForceLmtReqSts;
        float32_t longAccelerationCmd;
        bool longAccelerationValid;
        uint8_t longAccelerationState;
        uint8_t longControlMode;
        float32_t jerkMax;
        float32_t jerkMin;
        float32_t comfortBoundaryUp;
        float32_t comfortBoundaryLow;
        float32_t driveTorqueCmd;
        bool driveTorqueValid;
        uint8_t driveTorqueState;
        float32_t driveTorqueLimit;
        uint8_t rapidRespEnable;
        uint8_t brakePreferEnable;
        float32_t brakeTorqueCmd;
        uint8_t prefillEnable;
        uint8_t jerkBrakeEnable;
        bool ebaCtrlEnable;
        uint8_t ebaLevel;
        float32_t aebBrakeTorqueCmd;
        bool aebBrakeTorqueValid;
        uint8_t aebBrakeTorqueState;
        float32_t aebBrakeTorqueLimit;
        bool brakeTorqueValid;
        uint8_t brakeTorqueState;
        float32_t brakeTorqueLimit;
        uint8_t stopEnable;
        uint8_t driveOffEnable;
        uint8_t emergencyStopEnable;
        bool emerStopEnableValid;
        uint8_t brakeLightOnReq;
        uint8_t rpaEnable;
        float32_t speedLimit;
        float32_t distanceRemain;
        uint8_t immediateStopEnable;
        bool gearCmdValid;
        uint8_t gearCmdState;
        float32_t brakePedalCmd;
        bool brakePedalValid;
        uint8_t brakePedalState;
        float32_t throttlePedalCmd;
        bool throttlePedalValid;
        uint8_t throttlePedalState;
        HafHeader chassisCmdHeader;
        uint8_t epbCmd;
    };
}

#endif // HAF_CONTROL_CHASSIS_COMMAND_H__
