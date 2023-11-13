/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The definition of E2E_ConfigurationType
 * Create: 2019-06-17
 */
/**
* @file
*
* @brief The definition of E2E_ConfigurationType
*/
#ifndef E2EXF_CONFIG_TYPE_H
#define E2EXF_CONFIG_TYPE_H

#include <vector>
#include <array>
#include <iostream>
#include <cstdint>
#include <string>
#include "vrtf/com/e2e/E2EXf/E2EXf_ConfigIndexImpl.h"

namespace vrtf {
namespace com {
namespace e2e {
/* ----------------------------------------- Type definitions for Profile 4 ----------------------------------------- */
struct E2EXf_P04ConfigType {
    std::uint32_t DataID;
    std::uint16_t Offset;
    std::uint16_t MinDataLength;
    std::uint16_t MaxDataLength;
    std::uint16_t MaxDeltaCounter; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* ----------------------------------------- Type definitions for Profile 5 ----------------------------------------- */
struct E2EXf_P05ConfigType {
    std::uint16_t DataID;
    std::uint16_t Offset;
    std::uint16_t DataLength;
    std::uint8_t MaxDeltaCounter; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* ----------------------------------------- Type definitions for Profile 6 ----------------------------------------- */
struct E2EXf_P06ConfigType {
    std::uint16_t Offset;
    std::uint16_t MinDataLength;
    std::uint16_t MaxDataLength;
    std::uint16_t DataID;
    std::uint8_t MaxDeltaCounter; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* ----------------------------------------- Type definitions for Profile 7 ----------------------------------------- */
struct E2EXf_P07ConfigType {
    std::uint32_t DataID;
    std::uint32_t Offset;
    std::uint32_t MinDataLength;
    std::uint32_t MaxDataLength;
    std::uint32_t MaxDeltaCounter; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* ----------------------------------------- Type definitions for Profile 11 ---------------------------------------- */
enum class E2EXf_P11DataIDMode : std::uint8_t {
    E2EXF_P11_DATAID_BOTH = 0,
    E2EXF_P11_DATAID_ALT = 1,
    E2EXF_P11_DATAID_LOW = 2,
    E2EXF_P11_DATAID_NIBBLE = 3
};

struct E2EXf_P11ConfigType {
    std::uint16_t DataLength;
    std::uint16_t DataID;
    /* AXIVION Next Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    std::uint8_t MaxDeltaCounter;
    /* AXIVION Next Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    E2EXf_P11DataIDMode DataIDMode;
    std::uint16_t CounterOffset;
    std::uint16_t CRCOffset;
    /* AXIVION Next Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    std::uint16_t DataIDNibbleOffset;
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* ----------------------------------------- Type definitions for Profile 22 ---------------------------------------- */
struct E2EXf_P22ConfigType {
    std::uint16_t DataLength;
    std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList;
    /* AXIVION Next Line AutosarC++19_03-M0.1.3 : Convert user configuration to c var */
    std::uint16_t CRCOffset;
    /* AXIVION Next Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    std::uint8_t MaxDeltaCounter;
    std::uint16_t Offset;
    bool EnableCRC; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
    bool EnableCRCHW; /* AXIVION Same Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
};

/* --------------------------------------- Type definitions for State Machine --------------------------------------- */
struct E2EXf_SMConfigType {
    std::uint8_t WindowSizeValid;
    std::uint8_t MinOkStateInit;
    std::uint8_t MaxErrorStateInit;
    std::uint8_t MinOkStateValid;
    std::uint8_t MaxErrorStateValid;
    std::uint8_t MinOkStateInvalid;
    std::uint8_t MaxErrorStateInvalid;
    std::uint8_t WindowSizeInit;
    std::uint8_t WindowSizeInvalid;
    bool ClearToInvalid;
};

struct E2EXf_ConfigType {
    enum class Profile : std::uint8_t {
        PROFILE04 = 4,
        PROFILE05 = 5,
        PROFILE06 = 6,
        PROFILE07 = 7,
        PROFILE11 = 11,
        PROFILE22 = 22,
        UNDEFINED_PROFILE = 255
    };

    /* AXIVION disable style AutosarC++19_03-A9.5.1: It's allowed to use tagged unions until C++17 */
    /* AXIVION disable style AutosarC++19_03-A7.1.9: It's allowed to use tagged unions until C++17 */
    union {
        E2EXf_P04ConfigType Profile04;
        E2EXf_P05ConfigType Profile05;
        E2EXf_P06ConfigType Profile06;
        E2EXf_P07ConfigType Profile07;
        E2EXf_P11ConfigType Profile11;
        E2EXf_P22ConfigType Profile22;
    } ProfileConfig;
    /* AXIVION enable style AutosarC++19_03-A7.1.9 */
    /* AXIVION enable style AutosarC++19_03-A9.5.1 */
    Profile ProfileName;

    bool DisableE2ECheck;
    bool EnableTimeout;
    std::uint32_t Timeout;
};
} /* End e2e namespace */
} /* End com namespace */
} /* End vrtf namespace */
#endif

