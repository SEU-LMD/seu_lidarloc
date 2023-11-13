/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
* Description: class Context declaration
*/
#ifndef HCFI_CONTEXT_H
#define HCFI_CONTEXT_H

#include <thread>
#include <mutex>
#include <chrono>
#include <set>
#include <atomic>

namespace mdc {
namespace hcfi {
class BasicContext;
class Context {
public:
    enum class ContextType : uint8_t {
        PIPELINE_CONTEXT,
        TASK_CONTEXT,
        MODULE_CONTEXT,
    };

    Context(const ContextType type);
    virtual ~Context();

    /*
     * 阻塞操作
     * 同个job的exec内，不允许递归加锁
     */
    void Readlock();
    void Writelock();
    void Unlock();

    /* 非阻塞操作 */
    bool TryReadlock();
    bool TryWritelock();
    BasicContext& GetBasicContext() const;  // HCFI 内部使用接口

private:
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;
    std::unique_ptr<BasicContext> basicCtxIns_;
};
} // namespace hcfi
} // namespace mdc
#endif
