/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: StateManager Header
 * Create: 2020-6-6
 */
#ifndef ARA_PHM_PHM_STATE_MANAGER_H
#define ARA_PHM_PHM_STATE_MANAGER_H

#include <string>
#include <iostream>
#include <functional>
#include <map>
#include <vector>
#include <atomic>
#include <rtf/com/rtf_com.h>

namespace ara {
namespace phm {
using communicateMode = std::set<rtf::com::TransportMode>;
class PhmStateManager final {
public:
    ~PhmStateManager();
    PhmStateManager(const PhmStateManager &) = delete;
    PhmStateManager &operator=(const PhmStateManager &) = delete;
    static PhmStateManager &GetInstance();
    bool SetConcurrentNumber(uint32_t number);

    inline uint32_t GetConcurrentNumber() const
    {
        return concurrentNumber_;
    }

    bool Init(const std::string &user = "",
              const communicateMode &mode = {rtf::com::TransportMode::UDP, rtf::com::TransportMode::SHM});
    void DeInit();

    inline void RegisterRecoveryCallback(const std::function<void(std::string ruleInfo, std::string actionName,
        std::string portName, std::string methodName, std::string processName)> &cb)
    {
        cbRecovery_ = cb;
    }

    std::map<std::string, bool> QueryRuleResults(const std::string &ruleName);
    void RegisterProcessStatesCallback(const std::function<std::vector<std::pair<std::string, std::string>>(void)> &cb);
    void SetProcessState(const std::string& processName, const std::string& processState);
    std::function<void(std::string, std::string, std::string, std::string, std::string)> cbRecovery_ {};
    std::function<std::vector<std::pair<std::string, std::string>>(void)> cbProcessStates_ {};
private:
    PhmStateManager();
    void ReleaseResource();
    void GetProcessList();
    void GetProcessListFromCb();
    void ClearData();
    bool IsCommunicateModeValid(const communicateMode &mode);
    std::atomic_bool isInitialled_ {};
    uint32_t concurrentNumber_ {};
};
}
}
#endif

