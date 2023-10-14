/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class HeterogeneousComputingFramework declaration
*/

#ifndef HCFI_HETEROGENOUS_COMPUTING_FRAMEWORK_H
#define HCFI_HETEROGENOUS_COMPUTING_FRAMEWORK_H

#include "module.h"
#include "context.h"

namespace mdc {
namespace hcfi {
class BasicHeterogeneousComputingFramework;
class HeterogeneousComputingFramework {
public:
    /*
     * 单例模式，用户通过HeterogeneousComputingFramework::GetInstance()获取框架实例
     */
    static HeterogeneousComputingFramework& GetInstance();
    ~HeterogeneousComputingFramework();

    bool Init(const std::string& appName);
    bool Run();
    bool RegModule(const std::shared_ptr<Module> moduleInstance);
    const std::vector<std::shared_ptr<Module>> GetModules() const;
    BasicHeterogeneousComputingFramework& GetBasicHcfi() const;

private:
    /*
     * 单例模式固定格式，构造函数私有化
     */
    HeterogeneousComputingFramework();
    HeterogeneousComputingFramework(const HeterogeneousComputingFramework&) = delete;
    HeterogeneousComputingFramework& operator=(const HeterogeneousComputingFramework&) = delete;
    HeterogeneousComputingFramework(HeterogeneousComputingFramework&&) = delete;
    HeterogeneousComputingFramework& operator=(HeterogeneousComputingFramework&&) = delete;
    std::unique_ptr<BasicHeterogeneousComputingFramework> basicHcfi_;
};
} // namespace hcfi
} // namespace mdc
#endif
