/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:  definition
 * Create: ProcessNotifier 2020-12-10
 */
#ifndef ARA_PHM_PROCESS_NOTIFIER_H
#define ARA_PHM_PROCESS_NOTIFIER_H

#include <string>
#include <rtf/com/rtf_com.h>
#include "supervised_entity.h" // todo: should delete it

namespace ara {
namespace phm {
class ProcessNotifier {
public:
    explicit ProcessNotifier();
    ~ ProcessNotifier() = default;
    void ProcessChanged(std::string const &processName, std::string const &processState);

private:
    rtf::com::InstanceId instanceId_ {};
};
} // namespace phm
} // namespace ara
#endif

