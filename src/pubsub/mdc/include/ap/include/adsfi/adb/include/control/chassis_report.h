/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  chassisReport.h Control中所使用的结构体,用于表征车控上报的信息
 */

#ifndef HAF_CONTROL_CHASSIS_REPORT_H
#define HAF_CONTROL_CHASSIS_REPORT_H

#include "core/types.h"
namespace Adsfi {
    /* ******************************************************************************
        结构 名		:  HafActuatorWorkStatus
        功能描述		:  车辆传动工作信息
    ****************************************************************************** */
    struct HafActuatorWorkStatus {
        uint8_t capability;
        uint8_t response;
    };
    /* ******************************************************************************
        结构 名		:  HafActuatorStatus
        功能描述		:  车辆传动器信息
    ****************************************************************************** */
    struct HafActuatorStatus {
        HafActuatorWorkStatus angleSteer;
        HafActuatorWorkStatus torqueSteer;
        HafActuatorWorkStatus emergencyAngleSteer;
        HafActuatorWorkStatus emergencyTorqueSteer;
        HafActuatorWorkStatus steerVibrate;
        HafActuatorWorkStatus accelerate;
        HafActuatorWorkStatus decelerate;
        HafActuatorWorkStatus emergencyDecelerate;
        HafActuatorWorkStatus drive;
        HafActuatorWorkStatus brake;
        HafActuatorWorkStatus vlc;
        HafActuatorWorkStatus emergencyStop;
        HafActuatorWorkStatus stop;
        HafActuatorWorkStatus park;
        HafActuatorWorkStatus gear;
    };
    /* ******************************************************************************
        结构 名		:  HafSteerInfo
        功能描述		:  车辆传动器信息
    ****************************************************************************** */
    struct HafSteerInfo {
        HafFloat32WithValid steerAngle;
        HafFloat32WithValid steerAngleRate;
        HafFloat32WithValid steerPinionAngle;
        HafFloat32WithValid steerPinionAngleRate;
        HafFloat32WithValid frontSteerAngle;
        HafFloat32WithValid frontSteerAngleRate;
        HafUint8WithValid driverHandOn;
        HafFloat32WithValid driverHandTorque;
        HafFloat32WithValid steerTorque;
        HafFloat32WithValid motorCurrent;
        HafUint8WithValid driverOverride;
        uint8_t personalMode;
        uint8_t commandFault;
        uint8_t epsStatusMaster;
        uint8_t epsStatusSlave;
        int32_t faultCode;
        bool drvrSteerMonrEnaSts;
        HafUint8WithValid handsOffConf;
        HafFloat32WithValid epsMotorTq;
        uint8_t epsMotorTemp;
        uint8_t ldwWarnSts;
        uint8_t epsOperMod;
        uint8_t epsAbortFb;
        uint8_t epsTqSensSts;
        uint8_t epsSteerAgSensFilr;
    };
    /* ******************************************************************************
        结构 名		:  HafGearInfo
        功能描述		:  车辆齿轮信息
    ****************************************************************************** */
    struct HafGearInfo {
        HafGear gear;
        HafGear gearlever;
        uint8_t gearShiftStatus;
        HafUint8WithValid driverOverride;
        int32_t faultCode;
    };
    /* ******************************************************************************
        结构 名		:  HafThrottleInfo
        功能描述		:  车辆油门信息
    ****************************************************************************** */
    struct HafThrottleInfo {
        HafFloat32WithValid throttlePedal;
        HafFloat32WithValid throttlePedalRate;
        HafUint8WithValid driverOverride;
        HafFloat32WithValid driveTorque;
        HafFloat32WithValid driveTorqueMax;
        HafFloat32WithValid driveTorqueMin;
        HafFloat32WithValid driverDesiredTorque;
        HafFloat32WithValid engineSpeed;
        HafFloat32WithValid motorSpeed;
        HafUint8WithValid powerTrainReady;
        HafFloat32WithValid fuelRange;
        HafFloat32WithValid socHighVoltBattery;
        HafFloat32WithValid sohHighVoltBattery;
        HafFloat32WithValid socLowVoltBattery;
        uint8_t vcuStatus;
        uint8_t personalMode;
        uint8_t commandFault;
        int32_t faultCode;
        HafFloat32WithValid frontMotorSpeed;
        HafFloat32WithValid frontAxleActualTorque;
        HafFloat32WithValid rearMotorSpeed;
        HafFloat32WithValid rearAxleActualTorque;
        uint8_t vcuAbortFbk;
        HafFloat32WithValid sohLowVoltBattery;
    };
    /* ******************************************************************************
        结构 名		:  HafWheelSpeed
        功能描述		:  车辆车轮信息
    ****************************************************************************** */
    struct HafWheelSpeed {
        uint8_t wheelDirection;
        int32_t wheelCount;
        float32_t wheelSpeedMps;
        bool wheelCountValid;
        bool wheelSpeedMPSValid;
    };
    /* ******************************************************************************
        结构 名		:  HafBrakeInfo
        功能描述		:  车辆刹车信息
    ****************************************************************************** */
    struct HafBrakeInfo {
        HafFloat32WithValid brakePedal;
        HafUint8WithValid brakePedalSwitch;
        HafUint8WithValid driverOverride;
        HafFloat32WithValid mastCylinderPressure;
        HafFloat32WithValid brakeTorque;
        HafFloat32WithValid brakeForce;
        HafFloat32WithValid brakeTorqueMax;
        HafFloat32WithValid driverDesiredTorque;
        HafWheelSpeed wheelSpeedFl;
        HafWheelSpeed wheelSpeedFr;
        HafWheelSpeed wheelSpeedRl;
        HafWheelSpeed wheelSpeedRr;
        HafUint8WithValid standstill;
        HafFloat32WithValid roadFriction;
        HafFloat32WithValid brakeTemperature;
        HafFloat32WithValid slope;
        uint8_t brakeStatusMaster;
        uint8_t brakeStatusSlave;
        uint8_t commandFault;
        uint8_t ebdStatus;
        uint8_t epbLockStatus;
        uint8_t epbButtonStatus;
        int32_t faultCode;
        bool ebdActive;
        bool absActive;
        bool escActive;
        bool tcsActive;
        HafFloat32WithValid frontWhlIncTarTq;
        HafFloat32WithValid frontWhlDecTarTq;
        HafFloat32WithValid rearWhlIncTarTq;
        HafFloat32WithValid rearWhlDecTarTq;
        uint8_t autoHoldActvSts;
        uint8_t arpActvSts;
        uint8_t arpFctAvl;
        uint8_t arpFailrSts;
        uint8_t dbfActvSts;
        uint8_t hdcActvSts;
        uint8_t hhcActvSts;
        uint8_t aebAvl;
        uint8_t aebActv;
        uint8_t awbAvl;
        uint8_t awbActv;
        uint8_t brkCtrlAvl;
        uint8_t prefillAvl;
        uint8_t prefillActv;
        uint8_t epbOverride;
        uint8_t epbAvl;
        uint8_t espAbortFb;
        uint8_t brakeOverHeatStatus;
        HafFloat32WithValid velocity;
    };
    /* ******************************************************************************
        结构 名		:  HafVehicleMotion
        功能描述		:  车辆运动信息
    ****************************************************************************** */
    struct HafVehicleMotion {
        HafFloat32WithValid vx;
        HafFloat32WithValid vy;
        HafFloat32WithValid vz;
        HafFloat32WithValid ax;
        HafFloat32WithValid ay;
        HafFloat32WithValid az;
        HafFloat32WithValid yaw;
        HafFloat32WithValid roll;
        HafFloat32WithValid yawRate;
        HafFloat32WithValid pitchRate;
        HafFloat32WithValid rollRate;
        HafFloat32WithValid yawAcceleration;
        HafFloat32WithValid pitchAcceleration;
        HafFloat32WithValid rollAcceleration;
        HafFloat32WithValid pitch;
    };
    /* ******************************************************************************
        结构 名		:  HafChassisReport
        功能描述		:  车辆底盘信息
    ****************************************************************************** */
    struct HafChassisReport {
        HafHeader chassisReportHeader;
        float32_t velocity;
        float32_t steerAngle;
        HafGear actualGear;
        HafActuatorStatus master;
        HafActuatorStatus slave;
        HafSteerInfo steer;
        HafBrakeInfo brake;
        HafThrottleInfo vcu;
        HafGearInfo gear;
        HafVehicleMotion egoMotion;
    };
}

#endif // HAF_CONTROL_CHASSIS_REPORT_H
