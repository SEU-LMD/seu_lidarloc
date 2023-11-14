/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef MDC_DIAG_CONDITION_H
#define MDC_DIAG_CONDITION_H

#include <string>
#include "mdc/common/result.h"

namespace mdc {
namespace diag {
namespace internal {
class ConditionAgent;
}
class Condition {
public:
    enum class ConditionType : uint8_t {
        CONDITION_FALSE = 0x00U,
        CONDITION_TRUE = 0x01U
    };

    explicit Condition(const std::string &specifier);
    virtual ~Condition() noexcept = default;

    mdc::common::Result<ConditionType> GetCondition();
    mdc::common::Result<void>SetCondition(const ConditionType condition);
private:
    std::string conditionSpecifier_;
    std::shared_ptr<internal::ConditionAgent> agent_;
};
}
}
#endif // MDC_DIAG_CONDITION_H