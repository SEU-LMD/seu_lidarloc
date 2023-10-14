/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of E2EConfig.h
 * Create: 2020-11-19
 */

#ifndef RTF_COM_E2E_CONFIG_H_
#define RTF_COM_E2E_CONFIG_H_

#include <string>
#include <array>
#include "ara/com/e2e/E2EXf/E2EXf_CM.h"
namespace rtf {
namespace com {
namespace e2e {
using Result = ara::com::e2e::Result;
using ProfileCheckStatus = ara::com::e2e::ProfileCheckStatus;
using SMState = ara::com::e2e::SMState;
}
namespace config {
constexpr std::uint8_t  E2E_DATAID_LIST_LENGTH = 16U;
constexpr std::uint32_t UNDEFINED_HEADER_LENGTH = 0xFFFF'FFFFU;

enum class E2EProfile : std::uint8_t {
    PROFILE04 = 4,
    PROFILE05 = 5,
    PROFILE06 = 6,
    PROFILE07 = 7,
    PROFILE11 = 11,
    PROFILE22 = 22
};

enum class E2EDataIDMode : std::uint8_t {
    E2E_DATAID_BOTH = 0,
    E2E_DATAID_NIBBLE = 3
};

class E2EConfig {
public:
    /**
     * @brief Default constructor (deleted)
     *
     */
    E2EConfig() = delete;

    /**
     * @brief Construct a new E2EConfig object
     *
     * @param[in] profile  The E2E Profile will be used, excluding Profile 22
     * @param[in] dataId   The Data ID will be used
     */
    E2EConfig(const E2EProfile profile, const std::uint32_t dataId);

    /**
     * @brief Construct a new E2EConfig object using Profile 22
     *
     * @param[in] profile     The E2E Profile will be used, only for Profile 22
     * @param[in] dataIdList  The Data ID List will be used
     */
    E2EConfig(const std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH>& dataIdList);

    /**
     * @brief Construct a new E2EConfig object
     *
     * @param[in] profile     The E2E Profile will be used, only for Profile 5, 11
     * @param[in] dataId      The Data ID will be used
     * @param[in] dataLength  The length of serialied data will be protected or checked in bits (multiple of 8),
     *                        including E2E header length
     */
    E2EConfig(const E2EProfile profile, const std::uint32_t dataId,
              const std::uint16_t dataLength);

    /**
     * @brief Construct a new E2EConfig object
     *
     * @param[in] profile          The E2E Profile will be used, only for Profile 4, 6, 7
     * @param[in] dataId           The Data ID will be used
     * @param[in] minDataLength    The minimun data length of serialized data will be protected or checked in bits
     *                             (multiple of 8), including E2E header length
     * @param[in] maxDataLength    The maximun data length of serialized data will be protected or checked in bits
     *                             (multiple of 8), including E2E header length
     */
    E2EConfig(const E2EProfile profile, const std::uint32_t dataId,
              const std::uint32_t minDataLength, const std::uint32_t maxDataLength);

    /**
     * @brief Construct a new E2EConfig object only for Profile 22
     *
     * @param[in] name          The event name or method name will uing E2E
     * @param[in] dataIdList    The Data ID List will be used
     * @param[in] dataLength    The length of serialied data will be protected or checked in bits (multiple of 8),
     *                          including E2E header length
     */
    E2EConfig(const std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH>& dataIdList,
              const std::uint16_t dataLength);
    /**
     * @brief Default copy constructor
     *
     * @param[in] other The other instance will be copy
     */
    E2EConfig(const E2EConfig& other) = default;

    /**
     * @brief Default copy assignment
     *
     * @param[in] other The other instance will be copy
     * @return E2EConfig&   The new instance
     */
    E2EConfig& operator=(const E2EConfig& other) = default;

    /**
     * @brief Default destructor
     *
     */
    ~E2EConfig() = default;

    /**
     * @brief Get the Profile object
     *
     * @return E2EProfile   The Profile will be used
     */
    E2EProfile GetProfile() const noexcept { return profile_; }

    /**
     * @brief Get the Data ID object
     *
     * @return std::uint32_t   The DataID will be used if the Profile is not Profile 22
     */
    std::uint32_t GetDataID() const noexcept { return dataId_; }

    /**
     * @brief Get the Data ID List only for profile 22
     *
     * @return std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH>   the DataID List will be used if it is Profile 22
     */
    std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH> GetDataIDList() const noexcept { return dataIdList_; }

    /**
     * @brief Set whether to disable E2E check function
     *
     * @param[in] isDisable    whether to disable E2E check function
     */
    void DisableE2ECheck(const bool isDisable) noexcept { isDisableE2ECheck_ = isDisable; }

    /**
     * @brief Get whether disable E2E check function
     *
     * @retval true   disable E2E check function
     * @retval false  enable E2E check function
     */
    bool IsDisableE2ECheck() const noexcept { return isDisableE2ECheck_; }

    /**
     * @brief Set whether enable using CRC Hardware function only for Profile 4
     *
     * @param[in] isEnable    whether enable using CRC Hardware function
     */
    void EnableCRCHardware(const bool isEnable) noexcept { isEnableCRCHardware_ = isEnable; }

    /**
     * @brief Get whether enable using CRC Hardware function only for Profile 4
     *
     * @retval true   Enable using CRC Hardware when using Profile 4
     * @retval false  Disable using CRC Hardware when using Profile 4
     */
    bool IsEnableCRCHardware() const noexcept { return isEnableCRCHardware_; }

    /**
     * @brief Set the minimun serialized data length, including E2E Header size corresponding to the chosing profile,
     *        when using the Profile protecting dynamix serialized data length
     *
     * @param[in] minDataLength  The minimun serialized data length will be protected
     */
    void SetMinDataLength(const std::uint32_t minDataLength) noexcept { minDataLength_ = minDataLength; }

    /**
     * @brief Get the minimun serialized data length
     *
     * @return std::uint32_t    the minimun serialized data length,
     *                          the default value is the E2E header size corresponding dynamic profile,
     *                          otherwise it should be 0
     */
    std::uint32_t GetMinDataLength() const noexcept { return minDataLength_; }

    /**
     * @brief Set the maximun length of serilaized data, including E2E Header size corresponding to the chosing profile,
     *        when using the Profile protecting dynamix serialized data length
     *
     * @param[in] maxDataLength   The maximum serialized data length will be protected
     */
    void SetMaxDataLength(const std::uint32_t maxDataLength) noexcept { maxDataLength_ = maxDataLength; }

    /**
     * @brief Get the maximun length of serialized data
     *
     * @return std::uint32_t   The maxumun length of serialized data,
     *                         the default value is the maximum value range corresponding dynamic profile,
     *                         otherwise it should be 0
     */
    std::uint32_t GetMaxDataLength() const noexcept { return maxDataLength_; }

    /**
     * @brief Set the length of serialized data, including E2E Header size corresponding to the chosing profile,
     *        when using the Profile protecting fixed serialized data length
     *
     * @param[in] dataLength  the length of serailzed data will be protected
     */
    void SetDataLength(const std::uint16_t dataLength) noexcept { dataLength_ = dataLength; }

    /**
     * @brief Get the length of serialized data
     *
     * @return std::uint16_t  The serailized data's length, the default is 0
     */
    std::uint16_t GetDataLength() const noexcept { return dataLength_; }

    /**
     * @brief Set the allowed maximum difference between counter values for two consecutive packets
     *
     * @param[in] maxDeltaCounter   the allowed maximum difference between counter values for two consecutive packets
     */
    void SetMaxDeltaCounter(const std::uint32_t& maxDeltaCounter) noexcept { maxDeltaCounter_ = maxDeltaCounter; }

    /**
     * @brief Get the allowed maximum difference between counter values for two consecutive packets
     *
     * @return std::uint32_t  The setting allowed maximum difference between counter values for two consecutive packets,
     *                        and the default value is 1
     */
    std::uint32_t GetMaxDeltaCounter() const noexcept { return maxDeltaCounter_; }

    /**
     * @brief Set the mode of Data ID for how to transimit Data ID, and this is only for Profile 11
     *
     * @param[in] dataIDMode  The mode of DataID
     */
    void SetDataIDMode(const E2EDataIDMode dataIdMode) noexcept { dataIdMode_ = dataIdMode; }

    /**
     * @brief Get the mode of Data ID for transimitting Data ID
     *
     * @return E2EDataIDMode   The mode of data id if using Profile 11, the default value is E2EXF_DATAID_NIBBLE
     */
    E2EDataIDMode GetDataIDMode() const noexcept { return dataIdMode_; }
    /**
     * @brief Set the monitor size when State Machine is in the State Valid
     *
     * @param[in] windowSizeValid   the monitor size when State Machine is in the State Valid
     */
    void SetWindowSizeValid(const std::uint8_t windowSizeValid) noexcept;

    /**
     * @brief Get the monitor size when State Machine is in the State Valid
     *
     * @return std::uint8_t    The monitor size when State Machine is in the State Valid, and the default value is 3
     */
    std::uint8_t GetWindowSizeValid() const noexcept { return windowSizeValid_; }

    /**
     * @brief Set the minimun number of OK in monitor if State Machine want to swich from State Init to State Valid
     *
     * @param[in] minOkStateInit   the minimun number of OK in monitor
     */
    void SetMinOkStateInit(const std::uint8_t minOkStateInit) noexcept;

    /**
     * @brief Get the minimun number of OK in monitor if State Machine want to swich from State Init to State Valid
     *
     * @return std::uint8_t  The minimun number of OK in monitor if State Machine want to swich state
     *                       from State Init to State Valid, the default value is 1.
     */
    std::uint8_t GetMinOkStateInit() const noexcept { return minOkStateInit_; }

    /**
     * @brief Set the allowed maximun number of Error in monitor
     *        if State Machine want to swich from State Init to State Valid
     *
     * @param[in] maxErrorStateInit  The allowed maximun number of Error in monitor
     */
    void SetMaxErrorStateInit(const std::uint8_t maxErrorStateInit) noexcept;

    /**
     * @brief Get the allowed maximun number of Error in monitor
     *        if State Machine want to swich from State Init to State Valid
     *
     * @return std::uint8_t  The allowed maximun number of Error in monitor
     *                       if State Machine want to swich from State Init to State Valid, the default value is 1
     */
    std::uint8_t GetMaxErrorStateInit() const noexcept { return maxErrorStateInit_; }

    /**
     * @brief Set the minimun number of OK in monitor if State Machine want to remains in the State Valid
     *
     * @param[in] minOkStateValid   The minimun number of OK in monitor
     */
    void SetMinOkStateValid(const std::uint8_t minOkStateValid) noexcept;

    /**
     * @brief Get the minimun number of OK in monitor if State Machine want to remains in the State Valid
     *
     * @return std::uint8_t  The minimun number of OK in monitor if State Machine want to remains in the State Valid,
     *                       and the default value is 1.
     */
    std::uint8_t GetMinOkStateValid() const noexcept { return minOkStateValid_; }

    /**
     * @brief Set the allowed maximun number of Error in monitor if State Machine want to remains in the State Valid
     *
     * @param[in] maxErrorStateValid    The allowed maximun number of Error in monitor
     *                                  if State Machine want to remains in the State Valid
     */
    void SetMaxErrorStateValid(const std::uint8_t maxErrorStateValid) noexcept;

    /**
     * @brief Get the allowed maximun number of Error in monitor if State Machine want to remains in the State Valid
     *
     * @return std::uint8_t    The allowed maximun number of Error in monitor
     *                         if State Machine want to remains in the State Valid, and the defualt value is 1
     */
    std::uint8_t GetMaxErrorStateValid () const noexcept { return maxErrorStateValid_; }

    /**
     * @brief Set the minimun number of OK in monitor if State Machine want to switch from State Invalid to State Valid
     *
     * @param[in] minOkStateInvalid   The minimun number of OK in monitor
     */
    void SetMinOkStateInvalid(const std::uint8_t minOkStateInvalid) noexcept;

    /**
     * @brief Get the minimun number of OK in monitor if State Machine want to switch from State Invalid to State Valid
     *
     * @return std::uint8_t  The minumun number of OK in monitor
     *                       if State Machine want to switch from State Invalid to State Valid,
     *                       and the default value is 2.
     */
    std::uint8_t GetMinOkStateInvalid () const noexcept { return minOkStateInvalid_; }

    /**
     * @brief Set the allowed maximun number of Error in monitor
     *        if State Machine want to switch from State Invalid to State Valid
     *
     * @param[in] maxErrorStateInvalid  The allowed maximun number of Error in monitor
     */
    void SetMaxErrorStateInvalid(const std::uint8_t maxErrorStateInvalid) noexcept;

    /**
     * @brief Get the allowed maximun number of Error in monitor
     *        if State Machine want to switch from State Invalid to State Valid
     *
     * @return std::uint8_t   The allowed maximun number of Error in monitor
     *                        if State Machine want to switch from State Invalid to State Valid,
     *                        and the default value is 1.
     */
    std::uint8_t GetMaxErrorStateInvalid() const noexcept { return maxErrorStateInvalid_; }

    /**
     * @brief Set the monitor size when the State Machine in the State Init
     *
     * @param[in] windowSizeInit  The size of monitor window when in the State Init.
     */
    void SetWindowSizeInit(const std::uint8_t windowSizeInit) noexcept;

    /**
     * @brief Get the monitor size when the State Machine in the State Init
     *
     * @return std::uint8_t   The size of monitor window when in the State Init.
     */
    std::uint8_t GetWindowSizeInit() const noexcept { return windowSizeInit_; }

    /**
     * @brief Set the monitor size when the State Machine in the State Invalid
     *
     * @param[in] windowSizeInvalid  The size of monitor window when in the State Invalid.
     */
    void SetWindowSizeInvalid(const std::uint8_t windowSizeInvalid) noexcept;

    /**
     * @brief Get the monitor size when the State Machine in the State Invalid
     *
     * @return std::uint8_t  The size of monitor window when in the State Invalid
     */
    std::uint8_t GetWindowSizeInvalid() const noexcept { return windowSizeInvalid_; }

    /**
     * @brief Set whether enable clear the records in the monitor window if State Machine Switch to State Invalid
     *
     * @param[in] isClear  true means enable record clearing, otherwise not disable record clearing
     */
    void SetClearToInvalid(bool isClear) noexcept;

    /**
     * @brief Get whether enable clear the records in the monitor window if State Machine Switch to State Invalid
     *
     * @return true    enable record clearing
     * @return false   disable record clearing
     */
    bool GetClearToInvalid() const noexcept { return clearToInvalid_; }

    /**
     * @brief Get the Header Size of corresponding Profile
     *
     * @return std::size_t  The header size of cooresponding Profile
     */
    std::size_t GetHeaderSize() const noexcept;

    /**
     * @brief Whether the user sets the state machine
     *
     * @return true    user setting
     * @return false   using default value
     */
    bool IsSettingSMConfig() const noexcept { return isSettingSM_; }

    /**
     * @brief Set the Incorrect DataId object
     *
     * @param[in] incorrectDataId  Incorrect dataId will be used in response
     */
    void SetIncorrectDataId(const std::uint32_t incorrectDataId) noexcept { incorrectDataId_ = incorrectDataId; }

    /**
     * @brief Get the Incorrect DataId object
     *
     * @return std::uint32_t    The setted incorrect dataId
     */
    std::uint32_t GetIncorrectDataId() const noexcept { return incorrectDataId_; }

    /**
     * @brief Set the Incorrect DataIdList object
     *
     * @param[in] incorrectDataIdList   Incorrect dataIdList will be used in response
     */
    void SetIncorrectDataIdList(const std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH>& incorrectDataIdList) noexcept;

    /**
     * @brief Get the Incorrect DataIdList object
     *
     * @return std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH> The setted incorrect dataIdList
     */
    std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH> GetIncorrectDataIdList() const noexcept;

    /**
     * @brief Set E2E Offset in bit
     *
     * @param[in] offset   The offset of E2E Header in payload in bit, which shoule be multiple of 8
     */
    void SetE2EOffset(const std::uint32_t offset) noexcept;

    /**
     * @brief Get E2E Offset
     *
     * @return std::pair<bool, std::uint32_t>
     *      @retval first       Whether E2E Offset is set
     *      @retval second      The setted E2E Offset
     */
    std::pair<bool, std::uint32_t> GetE2EOffset() const noexcept { return offset_; }
private:
    // Profile Config
    E2EProfile profile_;
    std::uint32_t dataId_ {0xFFFFFFFEU};
    std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH> dataIdList_ {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool isDisableE2ECheck_ {false};
    bool isEnableCRCHardware_ {false};
    std::uint32_t minDataLength_ {0};
    std::uint32_t maxDataLength_ {0};
    std::uint16_t dataLength_ {0};
    std::uint32_t maxDeltaCounter_ {1}; // 1 is the default value of MaxDeltaCounter
    E2EDataIDMode dataIdMode_ {E2EDataIDMode::E2E_DATAID_NIBBLE};

    // StateMachine Config
    bool isSettingSM_ {false};
    std::uint8_t windowSizeValid_ {3}; // 3 is the default value of WindowSizeValid
    std::uint8_t minOkStateInit_ {1}; // 1 is the default value of MinOkStateInit
    std::uint8_t maxErrorStateInit_ {1}; // 1 is the default value of MaxErrorStateInit
    std::uint8_t minOkStateValid_ {1}; // 1 is the default value of MinOkStateValid
    std::uint8_t maxErrorStateValid_ {1}; // 1 is the default value of MaxErrorStateValid
    std::uint8_t minOkStateInvalid_ {2}; // 2 is the default value of MinOkStateInvalid
    std::uint8_t maxErrorStateInvalid_ {1}; // 1 is the default value of MaxErrorStateInvalid
    std::uint8_t windowSizeInit_ {2}; // 2 is the default value of WindowSizeInit
    std::uint8_t windowSizeInvalid_ {3}; // 3 is the default value of WindowSizeInvalid
    bool clearToInvalid_ {true};

    std::uint32_t incorrectDataId_ {0xFFFFFFFEU};
    std::array<std::uint8_t, E2E_DATAID_LIST_LENGTH> incorrectDataIdList_
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::pair<bool, std::uint32_t> offset_;

    std::uint32_t GetDefaultMaxDataLength(const E2EProfile Profile) const noexcept;
};
} /* End e2e namespace */
} /* End com namespace */
} /* End ara namespace */

#endif