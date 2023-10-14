/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef ARA_DIAG_CONDITION_H
#define ARA_DIAG_CONDITION_H

#include "ara/core/result.h"
#include "ara/core/string.h"
#include "ara/core/instance_specifier.h"

namespace mdc {
namespace diag {
namespace internal {
class ConditionAgent;
}
}
}

namespace ara {
namespace diag {
class Condition {
public:
    enum class ConditionType : uint8_t {
        kConditionFalse = 0x00U,
        kConditionTrue = 0x01U
    };

    explicit Condition(const ara::core::InstanceSpecifier &specifier);
    virtual ~Condition() noexcept = default;

    ara::core::Result<ConditionType> GetCondition();
    ara::core::Result<void> SetCondition(const ConditionType condition);
private:
    ara::core::String conditionSpecifier_;
    std::shared_ptr<mdc::diag::internal::ConditionAgent> agent_;
};
}
}
#endif // ARA_DIAG_CONDITION_H_