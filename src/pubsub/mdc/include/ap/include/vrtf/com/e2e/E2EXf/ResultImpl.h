/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The declaration of ReturnImpl
 * Create: 2019-06-17
 */
/**
* @file
*
* @brief The declaration of ReturnImpl
*/
#ifndef VRTF_COM_E2EXF_RESULT_H
#define VRTF_COM_E2EXF_RESULT_H

#include <vector>
#include <cstdint>

namespace vrtf {
namespace com {
namespace e2e {
namespace impl {
constexpr std::uint32_t UNDEFINED_HEADER_SIZE {0xFFFEU};
/* AXIVION Next Line AutosarC++19_03-M0.1.4 : Convert user configuration to c var */
constexpr std::uint32_t UNDEFINED_DATAID {0xFFFFFFFEU};
enum class ProfileCheckStatusImpl : std::uint8_t {
    kOk = 0U,
    kRepeated,
    kWrongSequence,
    kError,
    kNotAvailable,
    kNoNewData,
    kCheckDisabled,
    kTimeout
};

enum class SMStateImpl : std::uint8_t {
    kValid = 0U,
    kNoData,
    kInit,
    kInvalid,
    kStateMDisabled
};

class ResultImpl final {
public:
    ResultImpl(ProfileCheckStatusImpl CheckStatus, SMStateImpl State): CheckStatus_(CheckStatus), SMState_(State) {}
    ResultImpl(): ResultImpl(ProfileCheckStatusImpl::kCheckDisabled, SMStateImpl::kStateMDisabled) {}
    ~ResultImpl() = default;
    ResultImpl(const ResultImpl& ResultPre);

    ResultImpl& operator=(const ResultImpl& ResultPre) &;

    ProfileCheckStatusImpl GetProfileCheckStatus() const noexcept
    {
        return CheckStatus_;
    }
    SMStateImpl GetSMState() const noexcept
    {
        return SMState_;
    }

private:
    ProfileCheckStatusImpl CheckStatus_;
    SMStateImpl SMState_;
};

using PayloadImpl = std::vector<std::uint8_t>;

enum class ProtectResultImpl : std::uint8_t {
    OK = 0U,
    HardRuntimeError = 0xFFU
};
} /* End namespace impl */
} /* End namesapce e2e */
} /* End namesapce com */
} /* End namespace vrtf */
#endif
