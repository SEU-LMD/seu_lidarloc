/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class Task declaration
*/

#ifndef HCFI_TASK_H
#define HCFI_TASK_H

#include <string>
#include <memory>
#include <functional>
#include "context.h"
#include "event_set.h"
#include "event.h"

namespace mdc {
namespace hcfi {
class Module;
class Job;
class BasicTask;
class Task : public std::enable_shared_from_this<Task> {
public:
    enum class ScheduleMode : uint8_t {
        INVALID,        // TASK 级调度策略选无效值时，JOB 级调度策略才会生效，TASK 级策略有效时自动忽略 JOB 级调度参数
        STRICT_ORDER,
        END_JOB_ORDER,
        IGNORE_ORDER,
        BUTT
    };
    enum class TaskStatus : uint8_t {
        DISABLE,
        ENABLE
    };
    struct Attribute {
        std::shared_ptr<Context> taskCtx;
        Event triggerEvent {""};                // 使用者必须对此字段赋值，不可使用默认的空字符串
        TaskStatus status {TaskStatus::ENABLE};
        ScheduleMode mode {ScheduleMode::STRICT_ORDER};
    };
    /*
    * TASK构造函数传入的taskContext指的是TASK内的共享taskContext, 可以在pipeline间共享，一个TASK实例只有一份
    * 如果用户自身在实例化context后，不需要继续持有实例，建议用std::move 将控制权转移至框架
    * triggerEvent指示该TASK能接收的事件，用于框架内部使用
    */
    Task(const std::string& taskName, const Attribute& attr);
    virtual ~Task();

    /*
    * 将JOB注册至TASK，用户实例化JOB
    */
    bool RegJob(const std::shared_ptr<Job> jobInstance);

    /*
    * 使能TASK，使能自身，不能使能其他TASK
    * 该接口不会创建新的pipeline
    */
    bool Enable();

    /*
    * 去使能TASK，去使能自身
    * 去使能后，该task的所有pipeline不再运行，所有pipeline的信息会逐渐被删除
    */
    bool Disable();

    /*
    * 驱动TASK创建一个新的pipeline, pipelineContext作为该pipeline内共享的上下文
    * pipelineContext不能在不同的pipeline间共享，更不能在不同task间共享
    * 由于JOB需要通过该接口启动一条新的pipeline，故该接口放置于task内，而不是框架内
    * 根据当前设计，task的输入事件是唯一的，因此用户在启动pipeline时不需要指定事件
    * 返回大于等于0表示成功，返回值为当前积压的消息数（不是精确值），调用者应该根据此返回值调整触发的频率。
    * 返回小于0表示失败，入参上下文及其携带的消息如何处理由调用者决定。积压数超过极限也按照失败处理。
    */
    int32_t Trigger(const std::shared_ptr<Context> originContext);

    /*
    * 如果没有显式使用此接口指定TASK之间的关系，则HCFI不会主动进行TASK的trigger，应由用户代码触发下一个TASK
    * 否则应该由HCFI负责触发下一个TASK的运行，用户代码中不允许再调用下一个TASK的trigger接口
    */
    bool PubNextTask(const std::string& taskName);

    /*
    * 获取task域的context
    */
    std::shared_ptr<Context> GetTaskContext() const;

    const std::string& GetTaskName() const;

    const Event& GetTaskTriggerEvent() const;

    std::shared_ptr<Module> GetModule() const;

    const std::vector<std::shared_ptr<Job>> GetJobs() const;

    /*
    * 根据预处理提供的上下文，生成本task的pipelineContext,根据原始输入上下文生成特定pipelineContext的方法由用户定义
    * 除非事先调用PubNextTask设置好前后两个TASK的执行路径，由HCFI完成执行时前一级TASK的处理结果传递给后一TASK的过程
    * 否则生成pipeline上下文不允许将入参的上下文原封不动返回，必须重新生成一个新的实例（包括首TASK），否则会导致混乱。
    */
    virtual std::shared_ptr<Context> CreatePipelineContext(std::shared_ptr<Context> originContext) = 0;
    bool operator==(const Task&) const;
    BasicTask& GetBasicTask() const;  // HCFI 内部使用接口

private:
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    Task(Task&&) = delete;
    Task& operator=(Task&&) = delete;
    std::unique_ptr<BasicTask> basicTaskIns_;
};
} // namespace hcfi
} // namespace mdc

#endif
