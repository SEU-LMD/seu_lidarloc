/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class Module declaration
*/

#ifndef HCFI_MODULE_H
#define HCFI_MODULE_H

#include <vector>
#include "context.h"

namespace mdc {
namespace hcfi {
class BasicModule;
class Task;
class Module : public std::enable_shared_from_this<Module> {
public:
    struct Attribute {
        std::string firstTaskName;
        std::shared_ptr<Context> moduleContext;
    };

    Module(const std::string& moduleName, const Attribute& attr);
    virtual ~Module();
    virtual void InitCmService() = 0;

    /* 返回大于等于0表示成功，返回值为当前积压的消息数，调用者应该根据此返回值调整触发的频率。
     * 返回小于0表示失败，入参上下文及其携带的消息如何处理由调用者决定。积压数超过极限也按照失败处理。
     */
    int32_t Trigger(const std::shared_ptr<Context> originContext) const;
    bool RegTask(const std::shared_ptr<Task> taskInstance);

    const std::string& GetModuleName() const;
    const std::string& GetFirstTaskName() const;
    std::shared_ptr<Context> GetModuleContext() const;
    const std::vector<std::shared_ptr<Task>> GetTasks() const;
    bool operator==(const Module&) const;
    BasicModule& GetBasicModule() const;  // HCFI 内部使用接口
private:
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;
    Module(Module&&) = delete;
    Module& operator=(Module&&) = delete;
    std::unique_ptr<BasicModule> basicModuleIns_;
};
} // namespace hcfi
} // namespace mdc
#endif
