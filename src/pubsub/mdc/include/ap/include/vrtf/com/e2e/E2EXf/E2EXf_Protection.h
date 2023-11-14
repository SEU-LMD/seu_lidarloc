/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The declaration of E2EXf_Proetction.h
 * Create: 2019-06-17
 */

#ifndef VRTF_COM_E2EXF_PROTECTION_H
#define VRTF_COM_E2EXF_PROTECTION_H

#include <string>
#include <memory>
#include "vrtf/com/e2e/E2EXf/ResultImpl.h"
#include "vrtf/com/e2e/E2EXf/E2EXf_Object.h"
#include "vrtf/com/e2e/E2EXf/E2EXf_CMConfig.h"

namespace ara {
namespace com {
namespace e2e {
using SMState = vrtf::com::e2e::impl::SMStateImpl;

enum class ProfileCheckStatus : std::uint8_t {
    kOk = 0U,
    kRepeated,
    kWrongSequence,
    kError,
    kNotAvailable,
    kNoNewData,
    kCheckDisabled
};

class Result final {
public:
    /**
     * @brief Construct a new Result object
     *
     * @param[in] CheckStatus   represents the results of the check of a single sample
     * @param[in] State         represents in what state is the E2E supervision after the most recent check of
     *                          the sample(s) of a received sample of the event.
     */
    Result(ProfileCheckStatus CheckStatus, SMState State): CheckStatus_(CheckStatus), SMState_(State) {}

    /**
     * @brief Construct a new Result object
     *
     */
    Result(): Result(ProfileCheckStatus::kNotAvailable, SMState::kStateMDisabled) {}

    /**
     * @brief Destroy the Result object
     *
     */
    ~Result() = default;

    /**
     * @brief Construct a new Result object
     *
     * @param Other[in]     The other instance will be copied
     */
    Result(const Result& Other) : CheckStatus_(Other.CheckStatus_), SMState_(Other.SMState_) {}

    /**
     * @brief The copy assignment constructor
     *
     * @param[in] Other          The other instance will be copied
     * @return Result&          The new instance which contains the same value with input instance
     */
    Result& operator= (const Result& Other) &;

    /**
     * @brief Get the Profile Check Status
     *
     * @return ProfileCheckStatus  represents the results of the check of a single sample
     */
    ProfileCheckStatus GetProfileCheckStatus() const noexcept { return CheckStatus_;}

    /**
     * @brief
     *
     * @return SMState  represents in what state is the E2E supervision after the most recent check of the sample(s) of
     *                  a received sample of the event.
     */
    SMState GetSMState() const noexcept { return SMState_; }
private:
    ProfileCheckStatus CheckStatus_;
    SMState SMState_;
};
} // End of namespace e2e
} // End of namesapce com
} // End of namesapce ara

namespace vrtf {
namespace com {
namespace e2e {
using ProtectResult = vrtf::com::e2e::impl::ProtectResultImpl;
using Result = vrtf::com::e2e::impl::ResultImpl;

class E2EXf_Protection final {
public:
    /**
     * @brief Add E2E Profile Configuration and StateMachine Configuration to initialize E2E
     *
     * @param CMConfig[in]  The configurations of Profile and StateMachine
     * @return bool  ture if initialize E2E successfully(the first time), otherwise return false
     */
    static bool AddCMConfig(const E2EXf_CMConfig& CMConfig, std::string const &NetworkName = "");

    /**
     * @brief Delete E2E configuration of the profile using DataID
     *
     * @param Object[in]   The object will be deleted
     */
    static void DeleteCMConfig(const E2EXf_Object& Object);

    /**
     * @brief Protects the buffer to be transmitted for in-place transmission using pointer
     *
     * @param Object[in]              The object will be protected
     * @param Buffer[inout]           The data block contains user data and E2E header(which will be filled by E2E)
     * @param InputBufferLength[in]   The length of user data which doesn't contains E2E Header
     *
     * @return ProtectResult          E_OK if success, otherwise return corresponding error code.
     */
    static ProtectResult Protect(E2EXf_Object& Object, std::uint8_t* Buffer, const std::uint32_t InputBufferLength);

    /**
     * @brief Protects the buffer to be transmitted for in-place transmission using pointer and specific counter
     *
     * @param[in]    Object               The object will be protected
     * @param[inout] Buffer               The data block contains user data and E2E header(which will be filled by E2E)
     * @param[in]    InputBufferLength    The length of user data which doesn't contains E2E Header
     * @param[in]    Counter              The Counter will be used
     * @return ProtectResult              E_OK if success, otherwise return corresponding error code.
     */
    static ProtectResult Protect(E2EXf_Object& Object, std::uint8_t* Buffer, const std::uint32_t InputBufferLength,
                                 const std::uint32_t Counter, bool IsUsingIncorrectId = false);

    /**
     * @brief Checks the buffer to be transmitted for in-place transmission using pointer
     *
     * @param Object[inout]              The object will be checked
     * @param Buffer[inout]              The received data which contains E2E Header(which will be removed by E2E)
     * @param InputBufferLength[in]      The length of user data
     *
     * @return Result   return corresponding ProfileCheckStatus & SMState
     */
    static ara::com::e2e::Result Check(const E2EXf_Object& Object, std::uint8_t* Buffer,
                                       const std::uint32_t InputBufferLength);

    /**
     * @brief Checks the buffer to be transmitted for in-place transmission using pointer
     *
     * @param[inout] Object              The object will be checked
     * @param[inout] Buffer              The received data which contains E2E Header(which will be removed by E2E)
     * @param[in] InputBufferLength      The length of user data
     * @param[in] Counter                The counter will be used in check
     * @return Result    return corresponding ProfileCheckStatus & SMState
     */
    static ara::com::e2e::Result Check(const E2EXf_Object& Object, std::uint8_t* Buffer,
        const std::uint32_t InputBufferLength, const std::uint32_t Counter);

    /**
     * @brief Set the State of E2E State Machine to Init
     *
     * @param Object[in]   The index will be used to find the corresponding e2e configuration
     * @return true        set state sucessfully
     * @return false       set the state failed
     */
    static bool SetSMStateToInit(const E2EXf_Object& Object);

    /**
     * @brief Get the E2E Header size
     *
     * @param Object[in]               The index will be used to find the corresponding e2e configuration
     * @return std::uint32_t           The length of E2E Header
     */
    static std::uint32_t GetHeaderSize(const E2EXf_Object& Object);

    /**
     * @brief Get the counter of the received buffer
     *
     * @param[in] Buffer            The buffer contains E2E header
     * @param[in] BufferLength      The length of buffer in byte
     * @param[in] Profile           The profile used in the buffer
     * @param[in] Offset            The Offset of E2E Header in bits
     * @return std::pair<bool, std::uint32_t>   The first value represents the valid status of index,
     *                                          the second value represents the obtained counter
     */
    static std::pair<bool, std::uint32_t> GetReceivedBufferCounter(const std::uint8_t* Buffer,
                            const std::uint32_t BufferLength, const E2EXf_Profile Profile, const std::uint32_t Offset);

private:
    static constexpr std::uint8_t P04_COUNTER_LENGTH_BYTE {2U};
    static constexpr std::uint8_t P05_COUNTER_LENGTH_BYTE {1U};
    static constexpr std::uint8_t P06_COUNTER_LENGTH_BYTE {1U};
    static constexpr std::uint8_t P07_COUNTER_LENGTH_BYTE {4U};
    static constexpr std::uint8_t P04_COUNTER_OFFSET {2U};
    static constexpr std::uint8_t P05_COUNTER_OFFSET {2U};
    static constexpr std::uint8_t P06_COUNTER_OFFSET {4U};
    static constexpr std::uint8_t P07_COUNTER_OFFSET {12U};
    static constexpr std::uint8_t P11_P22_COUNTER_OFFSET {1U};
    static constexpr std::uint8_t BITS_PER_BYTE {8U};
    static std::uint32_t GetCounter(const std::uint8_t* Buffer, const std::uint32_t CounterOffset,
                                    const std::uint32_t CounterLength);

    static bool CheckParam(const std::uint8_t* Buffer, const std::uint32_t BufferLength,
                           const std::uint32_t CounterEndOffset, const std::uint32_t Offset);
};
} /* End e2e namespace */
} /* End com namespace */
} /* End vrtf namespace */
#endif

