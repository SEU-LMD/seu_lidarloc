/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of E2EXf_CMConfig.h
 * Create: 2020-11-10
 */

#ifndef VRTF_COM_E2EXF_CMCONFIG_H
#define VRTF_COM_E2EXF_CMCONFIG_H

#include <string>
#include <array>
#include "vrtf/com/e2e/E2EXf/E2EXf_ConfigIndexImpl.h"

namespace vrtf {
namespace com {
namespace e2e {
enum class E2EXf_Profile : std::uint8_t {
    PROFILE04 = 4,
    PROFILE05 = 5,
    PROFILE06 = 6,
    PROFILE07 = 7,
    PROFILE11 = 11,
    PROFILE22 = 22,
    UNDEFINE = 255
};

enum class E2EXf_DataIDMode : std::uint8_t {
    E2EXF_DATAID_BOTH = 0,
    E2EXF_DATAID_NIBBLE = 3
};

class E2EXf_CMConfig final {
public:
    E2EXf_CMConfig() = delete;

    /**
     * @brief Construct a new e2exf cmconfig object
     *
     * @param[in] Profile  The profile will be used
     * @param[in] DataID   The dataId will be used
     */
    E2EXf_CMConfig(const E2EXf_Profile Profile, const std::uint32_t DataID);

    /**
     * @brief Construct a new e2exf cmconfig object
     *
     * @param[in] Profile       The profile will be used
     * @param[in] DataIDList    The dataId list will be used
     */
    E2EXf_CMConfig(const E2EXf_Profile Profile, const std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList);

    /**
     * @brief Construct a new e2exf cmconfig object
     *
     * @param[in] Profile     The profile will be used
     * @param[in] DataID      The dataId will be used
     * @param[in] DataLength  The
     * @param[in] Offset
     */
    E2EXf_CMConfig(const E2EXf_Profile Profile, const std::uint32_t DataID,
                   const std::uint16_t DataLength, const std::uint32_t Offset);

    /**
     * @brief Construct a new e2exf cmconfig object better used in Profile 4, 6, 7
     *
     * @param[in] Profile         The profile will be used
     * @param[in] DataID          The dataId will be used
     * @param[in] MinDataLength   The min data length of serialized data will be protected or checked
     * @param[in] MaxDataLength   The max data length of serialized data will be protected or checked
     * @param[in] Offset          The offset of E2E header in payload
     */
    E2EXf_CMConfig(const E2EXf_Profile Profile, const std::uint32_t DataID,
                   const std::uint32_t MinDataLength, const std::uint32_t MaxDataLength, const std::uint32_t Offset);

    /**
     * @brief Construct a new e2exf cmconfig object better used in Profile 22
     *
     * @param[in] Profile       The profiled will be used
     * @param[in] DataIDList    The dataIdList will be used
     * @param[in] DataLength    The data length of serialized data will be protected or checked
     * @param[in] Offset        The offset of E2E header in payload
     */
    E2EXf_CMConfig(const E2EXf_Profile Profile,
                   const std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList,
                   const std::uint16_t DataLength, const std::uint32_t Offset);

    /**
     * @brief Construct a new e2exf cmconfig object
     *
     * @param[in] Other   other e2exf cmconfig object
     */
    E2EXf_CMConfig(const E2EXf_CMConfig& Other) = default;

    /**
     * @brief Construct a new e2exf cmconfig object
     *
     * @param[in] Other   other e2exf cmconfig object
     * @return E2EXf_CMConfig&   the e2exf cmconfig object
     */
    E2EXf_CMConfig& operator=(const E2EXf_CMConfig& Other) & = default;

    /**
     * @brief Destroy the e2exf cmconfig object
     *
     */
    ~E2EXf_CMConfig() = default;

    /**
     * @brief Get the Profile object
     *
     * @return E2EXf_Profile  the configured profile
     */
    E2EXf_Profile GetProfile() const noexcept { return Profile_; }

    /**
     * @brief Get the DataID object
     *
     * @return std::uint32_t  the configured profile
     */
    std::uint32_t GetDataID() const noexcept { return DataID_; }

    /**
     * @brief Get the DataIDList object
     *
     * @return std::array<std::uint8_t, DATAIDLIST_LENGTH> The configured dataIdList
     */
    std::array<std::uint8_t, DATAIDLIST_LENGTH> GetDataIDList() const noexcept { return DataIDList_; }

    /**
     * @brief Set the Offset object
     *
     * @param[in] Offset  the offset will be configured
     */
    void SetOffset(std::uint32_t Offset) noexcept { Offset_ = Offset; }

    /**
     * @brief Get the Offset object
     *
     * @return std::uint32_t
     */
    std::uint32_t GetOffset() const noexcept { return Offset_; }

    /**
     * @brief Do not used E2E check
     *
     * @param[in] IsDisable  is disable e2e checking function
     */
    void DisableE2ECheck(bool IsDisable) noexcept { IsDisableE2E_ = IsDisable; }

    /**
     * @brief If using diable e2e check
     *
     * @return bool
     *      @retval true   disabale e2e check
     *      @retval false  enbale e2e check
     */
    bool IsDisableE2ECheck() const noexcept { return IsDisableE2E_; }

    /**
     * @brief Is using crc hardware
     *
     * @param[in] IsEnable  Is enable crc hardware which only used in Profile 4
     */
    void EnableCRCHardware(bool IsEnable) noexcept { IsEnableCRCHardware_ = IsEnable; }

    /**
     * @brief If is useing crc hardware
     *
     *  @return bool
     *      @retval true   using crc hardware
     *      @retval false  not using crc hardware
     */
    bool IsEnableCRCHardware() const noexcept { return IsEnableCRCHardware_; }

    /**
     * @brief Set the Min Data Length object
     *
     * @param[in] MinDataLength   The min data length of serialized data will be protected or check
     */
    void SetMinDataLength(const std::uint32_t MinDataLength) noexcept { MinDataLength_ = MinDataLength; }

    /**
     * @brief Get the Min Data Length object
     *
     * @return std::uint32_t  The configured min data length of serialized data
     */
    std::uint32_t GetMinDataLength() const noexcept { return MinDataLength_; }

    /**
     * @brief Set the Max Data Length object
     *
     * @param[in] MaxDataLength   The max data length of serialized data will be protected or check
     */
    void SetMaxDataLength(const std::uint32_t MaxDataLength) noexcept { MaxDataLength_ = MaxDataLength; }

    /**
     * @brief Get the MaxDataLength object
     *
     * @return std::uint32_t  The configured max data length of serialized data
     */
    std::uint32_t GetMaxDataLength() const noexcept { return MaxDataLength_; }

    /**
     * @brief Set the Data Length object
     *
     * @param[in] DataLength  The data length of serialized data will be protected or check
     */
    void SetDataLength(const std::uint16_t DataLength) noexcept { DataLength_ = DataLength; }

    /**
     * @brief Get the Data Length object
     *
     * @return std::uint16_t  The configured data length of serialized data
     */
    std::uint16_t GetDataLength() const noexcept { return DataLength_; }

    /**
     * @brief Set the Max Delta Counter object
     *
     * @param[in] MaxDeltaCounter  The max deleta counter between two received payload
     */
    void SetMaxDeltaCounter(const std::uint32_t MaxDeltaCounter) noexcept { MaxDeltaCounter_ = MaxDeltaCounter; }

    /**
     * @brief Get the Max Delta Counter object
     *
     * @return std::uint32_t  The configured max delta counter
     */
    std::uint32_t GetMaxDeltaCounter() const noexcept { return MaxDeltaCounter_; }

    /**
     * @brief Set the Data ID Mode object which will be used in Profile 11
     *
     * @param[in] DataIDMode   The mode of using dataId in Profile 11
     */
    void SetDataIDMode(const E2EXf_DataIDMode DataIDMode) noexcept { DataIDMode_ = DataIDMode; }

    /**
     * @brief Get the DataID Mode object
     *
     * @return E2EXf_DataIDMode  The configured dataIdMode
     */
    E2EXf_DataIDMode GetDataIDMode() const noexcept { return DataIDMode_; }

    /**
     * @brief Set the Window Size Valid object
     *
     * @param[in] WindowSizeValid  Size of the monitoring window for the state machine during state VALID.
     */
    void SetWindowSizeValid(const std::uint8_t WindowSizeValid) noexcept;

    /**
     * @brief Get the Window Size Valid object
     *
     * @return std::uint8_t   The configured WindowSizeValid
     */
    std::uint8_t GetWindowSizeValid() const noexcept { return WindowSizeValid_; }

    /**
     * @brief Set the Min Ok State Init object
     *
     * @param[in] MinOkStateInit   Minimal number of checks in which ProfileStatus equal to E2E_P_OK
     *                             was determined within the last WindowSize checks (for the state E2E_SM_INIT)
     *                             required to change to state E2E_SM_VALID.
     */
    void SetMinOkStateInit(const std::uint8_t MinOkStateInit) noexcept;

    /**
     * @brief Get the Min Ok State Init object
     *
     * @return std::uint8_t The configured MinOkStateInit
     */
    std::uint8_t GetMinOkStateInit() const noexcept { return MinOkStateInit_; }

    /**
     * @brief Set the Max Error State Init object
     *
     * @param[in] MaxErrorStateInit Maximal number of checks in which ProfileStatus equal to E2E_P_ERROR was determined,
     *                              within the last WindowSize checks (for the state E2E_SM_INIT).
     */
    void SetMaxErrorStateInit(const std::uint8_t MaxErrorStateInit) noexcept;

    /**
     * @brief Get the Max Error State Init object
     *
     * @return std::uint8_t  The configured MaxErrorStateInit
     */
    std::uint8_t GetMaxErrorStateInit() const noexcept { return MaxErrorStateInit_; }

    /**
     * @brief Set the Min Ok State Valid object
     *
     * @param[in] MinOkStateValid   Minimal number of checks in which ProfileStatus equal to E2E_P_OK was determined
     *                              within the last WindowSize checks (for the state E2E_SM_VALID) required to
     *                              keep in state E2E_SM_VALID.
     */
    void SetMinOkStateValid(const std::uint8_t MinOkStateValid) noexcept;

    /**
     * @brief Get the Min Ok State Valid object
     *
     * @return std::uint8_t   The configured MinOkStateValid
     */
    std::uint8_t GetMinOkStateValid() const noexcept { return MinOkStateValid_; }

    /**
     * @brief Set the Max Error State Valid object
     *
     * @param[in] MaxErrorStateValid   Maximal number of checks in which ProfileStatus equal to
     *                                 E2E_P_ERROR was determined, within the last WindowSize checks
     *                                 (for the state E2E_SM_VALID).
     */
    void SetMaxErrorStateValid(const std::uint8_t MaxErrorStateValid) noexcept;

    /**
     * @brief Get the Max Error State Valid object
     *
     * @return std::uint8_t  The configured MaxErrorStateValid;
     */
    std::uint8_t GetMaxErrorStateValid() const noexcept { return MaxErrorStateValid_; }

    /**
     * @brief Set the Min Ok State Invalid object
     *
     * @param[in] MinOkStateInvalid  Minimum number of checks in which ProfileStatus equal to E2E_P_OK was determined
     *                               within the last WindowSize checks (for the state E2E_SM_INVALID)
     *                               required to change to state E2E_SM_VALID.
     */
    void SetMinOkStateInvalid(const std::uint8_t MinOkStateInvalid) noexcept;

    /**
     * @brief Get the Min Ok State Invalid object
     *
     * @return std::uint8_t  The configured MinOkStateInvalid
     */
    std::uint8_t GetMinOkStateInvalid () const noexcept { return MinOkStateInvalid_; }

    /**
     * @brief Set the Max Error State Invalid object
     *
     * @param[in] MaxErrorStateInvalid Minimum number of checks in which ProfileStatus equal to E2E_P_OK was determined
     *                                 within the last WindowSize checks (for the state E2E_SM_INVALID) required to
     *                                 change to state E2E_SM_VALID.
     */
    void SetMaxErrorStateInvalid(const std::uint8_t MaxErrorStateInvalid) noexcept;

    /**
     * @brief Get the Max Error State Invalid object
     *
     * @return std::uint8_t The configured MaxErrorStateInvalid
     */
    std::uint8_t GetMaxErrorStateInvalid() const noexcept { return MaxErrorStateInvalid_; }

    /**
     * @brief Set the Window Size Init object
     *
     * @param[in] WindowSizeInit  Size of the monitoring windows for the state machine during state INIT.
     */
    void SetWindowSizeInit(const std::uint8_t WindowSizeInit) noexcept;

    /**
     * @brief Get the Window Size Init object
     *
     * @return std::uint8_t  The configured WindowSizeInit
     */
    std::uint8_t GetWindowSizeInit() const noexcept { return WindowSizeInit_; }

    /**
     * @brief Set the Window Size Invalid object
     *
     * @param[in] WindowSizeInvalid  Size of the monitoring window for the state machine during state INVALID.
     */
    void SetWindowSizeInvalid(const std::uint8_t WindowSizeInvalid) noexcept;

    /**
     * @brief Get the Window Size Invalid object
     *
     * @return std::uint8_t  The configured WindowSizeInvalid
     */
    std::uint8_t GetWindowSizeInvalid() const noexcept { return WindowSizeInvalid_; }

    /**
     * @brief Set the Clear To Invalid object
     *
     * @param[in] IsClear  Clear monitoring window data on transition to state INVALID.
     */
    void SetClearToInvalid(bool IsClear) noexcept;

    /**
     * @brief Get the Clear To Invalid object
     *
     * @return bool
     *      @retval true    Clear monitoring window data on transition to state INVALID.
     *      @retval false   Do not clear monitoring window data on transition to state INVALID.
     */
    bool GetClearToInvalid() const noexcept { return ClearToInvalid_; }

    /**
     * @brief Get the Header Size object
     *
     * @return std::size_t  The size of E2E Header according to the configired Profile
     */
    std::size_t GetHeaderSize() const noexcept;

    /**
     * @brief If the State Machine configuration was configured
     *
     * @return bool
     *      @retval true    The State Machine configuration was configured
     *      @retval false   The State Machine configuration was not configured
     */
    bool IsSettingSMConfig() const noexcept { return IsSettingSM_; }
private:
    // Profile Config
    E2EXf_Profile Profile_ {E2EXf_Profile::UNDEFINE};
    std::uint32_t DataID_ {UNDEFINED_DATAID};
    std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList_ {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool IsDisableE2E_ {false};
    bool IsEnableCRCHardware_ {false};
    std::uint32_t Offset_ {0}; // 0 is the default value of Offset
    std::uint32_t MinDataLength_ {0};
    std::uint32_t MaxDataLength_ {0};
    std::uint16_t DataLength_ {0};
    std::uint32_t MaxDeltaCounter_ {1}; // 1 is the default value of MaxDeltaCounter
    E2EXf_DataIDMode DataIDMode_ {E2EXf_DataIDMode::E2EXF_DATAID_NIBBLE};

    // StateMachine Config
    bool IsSettingSM_ {false};
    /* AXIVION MAGIC: 3 is the default value of WindowSizeValid */
    std::uint8_t WindowSizeValid_ {3};
    std::uint8_t MinOkStateInit_ {1}; // 1 is the default value of MinOkStateInit
    std::uint8_t MaxErrorStateInit_ {1}; // 1 is the default value of MaxErrorStateInit
    std::uint8_t MinOkStateValid_ {1}; // 1 is the default value of MinOkStateValid
    std::uint8_t MaxErrorStateValid_ {1}; // 1 is the default value of MaxErrorStateValid
    std::uint8_t MinOkStateInvalid_ {2}; // 2 is the default value of MinOkStateInvalid
    std::uint8_t MaxErrorStateInvalid_ {1}; // 1 is the default value of MaxErrorStateInvalid
    std::uint8_t WindowSizeInit_ {2}; // 2 is the default value of WindowSizeInit
    /* AXIVION MAGIC: 3 is the default value of WindowSizeInvalid */
    std::uint8_t WindowSizeInvalid_ {3};
    bool ClearToInvalid_ {true};

    static std::uint32_t GetDefaultMaxDeltaCounter(const E2EXf_Profile Profile);
};
} /* End e2e namespace */
} /* End com namespace */
} /* End ara namespace */

#endif

