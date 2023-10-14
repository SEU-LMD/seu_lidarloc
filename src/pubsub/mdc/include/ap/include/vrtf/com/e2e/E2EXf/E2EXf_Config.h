/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The definition of E2E_Config
 * Create: 2019-06-17
 */
/**
* @file
*
* @brief The definition of E2E_Config
*/
#ifndef E2EXF_CONFIG_H
#define E2EXF_CONFIG_H

#include <vector>
#include <array>
#include <iostream>
#include <cstdint>
#include <string>
#include "vrtf/com/e2e/E2EXf/E2EXf_ConfigType.h"
#include "vrtf/com/e2e/E2EXf/E2EXf_ConfigIndexImpl.h"
#include "vrtf/com/e2e/E2EXf/ResultImpl.h"

namespace vrtf {
namespace com {
namespace e2e {
/* AXIVION Next Line AutosarC++19_03-A0.1.6 : Convert user configuration to c var */
using ProfileCheckStatus = vrtf::com::e2e::impl::ProfileCheckStatusImpl;
/* AXIVION Next Line AutosarC++19_03-A0.1.6 : Convert user configuration to c var */
using SMState = vrtf::com::e2e::impl::SMStateImpl;
using Result = vrtf::com::e2e::impl::ResultImpl;
using ProtectResult = vrtf::com::e2e::impl::ProtectResultImpl;

using Payload = vrtf::com::e2e::impl::PayloadImpl;
using E2EXf_ConfigIndex = vrtf::com::e2e::impl::E2EXf_ConfigIndexImpl;

class E2EXf_Config final {
public:
    E2EXf_Config();
    ~E2EXf_Config() = default;
    E2EXf_Config(const E2EXf_Config&) = default;
    E2EXf_Config& operator = (const E2EXf_Config&) = delete;
    E2EXf_Config(const E2EXf_ConfigType& Config, const E2EXf_SMConfigType SMConfig);
    const E2EXf_ConfigType::Profile& GetProfileName() const;
    /**
     * @brief Get E2E configuration.
     * @return The configuration.
     * @req{
     * AR-iAOS-E2EXf_Config-00001,
     * E2EXf_Config shall provide E2E protect configuration class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     * }
     */
    const E2EXf_ConfigType& GetE2EXfConfig() const
    {
        return ConfigType_;
    }

    /**
     * @brief Get E2E State Machine configuration.
     * @return The State Machine configuration.
     * @req{
     * AR-iAOS-E2EXf_Config-00001,
     * E2EXf_Config shall provide E2E protect configuration class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     * }
     */
    const E2EXf_SMConfigType& GetSMConfig() const
    {
        return SMConfig_;
    }

    /**
     * @brief Set E2E configuration.
     * @param[in] Config The configuration.
     * @req{
     * AR-iAOS-E2EXf_Config-00001,
     * E2EXf_Config shall provide E2E protect configuration class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     * }
     */
    void SetE2EXfConfig(const E2EXf_ConfigType& Config)
    {
        ConfigType_ = Config;
    }

    /**
     * @brief Set E2E SM configuration.
     * @param[in] Config The SM configuration.
     * @req{
     * AR-iAOS-E2EXf_Config-00001,
     * E2EXf_Config shall provide E2E protect configuration class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     * }
     */
    void SetSMConfig(const E2EXf_SMConfigType SMConfig)
    {
        SMConfig_ = SMConfig;
    }
private:
    E2EXf_SMConfigType SMConfig_ {0, 0, 0, 0, 0, 0, 0, 0, 0, false};
    E2EXf_ConfigType ConfigType_ {};
};

/**
 * @brief Getting the size of E2E header
 *
 * @req{ AR-iAOS-E2EXf_Config-00003,
 * E2EXf_Config shall provide E2E ConfigIndex class,
 * ASIL-D,
 * DR-iAOS-RTF-RTFE2E-00005,
 * }
 *
 * @param[in] Index The Index of the profile configuration for finding the corresponding Profile
 * @return The length of E2E header in byte.
 */
std::uint32_t E2EXf_GetHeaderSize(const E2EXf_ConfigIndex& Index);

/**
 * @brief Binding configuration to config index and initialize configuration.
 *
 * @req{ AR-iAOS-E2EXf_Config-00004,
 * E2EXf_Config shall provide add configuration to configIndex,
 * ASIL-D,
 * DR-iAOS-RTF-RTFE2E-00005,
 * }
 *
 * @param[in] Index              The Index of the profile configuration for finding the corresponding Profile
 * @param[in] E2EXfConfig        The configuration of Profile
 * @return bool.
 * @retval true Succeed to add config.
 * @retval false Failed to add config.
 */
bool AddE2EXfConfig(const E2EXf_ConfigIndex& Index, const E2EXf_Config& Config);

/**
 * @brief Protecting user data with specific configuration in in-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00005,
 * E2EXf_Config shall provide in-place protect interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00006,
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @return The result of protecting data with specific configuration of Index.
 */
ProtectResult E2EXf_Protect(const E2EXf_ConfigIndex& Index, Payload& Buffer, const std::uint32_t& InputBufferLength);

/**
 * @brief Protecting user data with specific Counter in in-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00005,
 * E2EXf_Config shall provide in-place protect interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00006,
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @param[in] Counter              The counter will be written into E2E header.
 * @return The result of protecting data with specific Counter.
 */
ProtectResult E2EXf_Protect(const E2EXf_ConfigIndex& Index, Payload& Buffer, const std::uint32_t& InputBufferLength,
                            const std::uint32_t& Counter);

/**
 * @brief Protecting user data with specific configuration in Out-of-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00007,
 * E2EXf_Config shall provide out-of-place protect interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00006,
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @param[in] Counter              The counter will be written into E2E header.
 * @return The result of protecting data in Out-of-place mode.
 */
ProtectResult E2EXf_ProtectOutOfPlace(const E2EXf_ConfigIndex& Index, const Payload& InputBuffer,
                                      Payload& Buffer, const std::uint32_t& InputBufferLength);

/**
 * @brief  Protecting user data with specific counter in Out-of-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00007,
 * E2EXf_Config shall provide out-of-place protect interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00006,
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @param[in] Counter              The counter will be written into E2E header.
 * @return The result of adding E2E header
 */
ProtectResult E2EXf_ProtectOutOfPlace(const E2EXf_ConfigIndex& Index, const Payload& InputBuffer, Payload& Buffer,
                                      const std::uint32_t& InputBufferLength, const std::uint32_t& Counter);

/**
 * @brief Checking user data in in-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00006
 * E2EXf_Config shall provide in-place check interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00007
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @return The result of checking user data
 */
Result E2EXf_Check(const E2EXf_ConfigIndex& Index, Payload& Buffer, const std::uint32_t& InputBufferLength);

/**
 * @brief Checking user data in out-of-place mode.
 *
 * @req{ AR-iAOS-E2EXf_Config-00008
 * E2EXf_Config shall provide out-of-place check interface,
 * D,
 * DR-iAOS-RTF-RTFE2E-00007
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @param[inout] Buffer            The payload including E2E header buffer which located in the begin of the payload
 * @param[in] InputBuffer          Only used in out-of-place transmission, pointer to the user data
 * @param[in] InputBufferLength    The length of Protected Data without the length of E2E header
 * @return The result of checking user data
 */
Result E2EXf_CheckOutOfPlace(const E2EXf_ConfigIndex& Index, const Payload& InputBuffer,
                             Payload& Buffer, const std::uint32_t& InputBufferLength);

/**
 * @brief Setting state machine to init state.
 *
 * @req{ AR-iAOS-E2EXf_Config-00002,
 * E2EXf_Config shall provide E2E state machine configuration class,
 * D,
 * DR-iAOS-RTF-RTFE2E-00005,
 * }
 *
 * @param[in] Index                The Index of the profile configuration for finding the corresponding Profile
 * @returnval true                 Succeeded
 * @returnval false                Failed
 */
bool SetSMStateToInit(const E2EXf_ConfigIndex& Index);

/**
* @brief E2E Deinitialize
 *
 * @req{ AR-iAOS-E2EXf_Config-00002,
 * E2EXf_Config shall provide E2E state machine configuration class,
 * D,
 * DR-iAOS-RTF-RTFE2E-00005,
 * }
 *
*/
void DeInit();

bool SetOffset(const E2EXf_ConfigIndex& Index, const uint32_t offset);

} /* End e2e namespace */
} /* End com namespace */
} /* End vrtf namespace */
#endif
