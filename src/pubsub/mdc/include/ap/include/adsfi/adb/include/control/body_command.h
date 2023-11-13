/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description:  bodyCommand.h Control中所使用的结构体,用于表征车身信息
 */

#ifndef HAF_CONTROL_BODY_COMMAND_H__
#define HAF_CONTROL_BODY_COMMAND_H__

#include "core/types.h"


namespace Adsfi {
    /* ******************************************************************************
    结构 名		:  HafBodyCommand
    功能描述		:  车辆车身命令信息
    ****************************************************************************** */
    struct HafBodyCommand {
        uint8_t source;
        uint16_t adsFunctionCmd;
        uint16_t adsWorkStatusCmd;
        uint16_t moduleIdCmd;
        uint8_t turnLightCmd;
        uint8_t hazardLightCmd;
        uint8_t reverseLightCmd;
        uint8_t hornCmd;
        uint8_t highLowBeamLightCmd;
        uint8_t dayTimeRunLightCmd;
        uint8_t sidePositionLightCmd;
        uint8_t windShieldWashCmd;
        uint8_t windShieldWiperCmd;
        uint8_t rearViewMirrorCmd;
        uint8_t parkFinishCmd;
        uint8_t leftFrontWindowCmd;
        uint8_t rightFrontWindowCmd;
        uint8_t leftRearWindowCmd;
        uint8_t rightRearWindowCmd;
        uint8_t topWindowCmd;
        HafHeader bodyCmdHeader;
        uint8_t turnLightState;
        uint8_t hazardLightState;
        uint8_t doorLockCmd;
    };
}

#endif // HAF_CONTROL_BODY_COMMAND_H__
