/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class Job declaration
*/

#ifndef HCFI_JOB_H
#define HCFI_JOB_H

#include <string>
#include <memory>
#include "context.h"
#include "event_set.h"
#include "event.h"

namespace mdc {
namespace hcfi {
class Task;
class BasicJob;
class Job {
public:
    enum class TargetProcessor : uint8_t {
        CPU,
        AI,
        BUTT
    };

    enum class ScheduleMode : uint8_t {
        INVALID,        // TASK 级调度策略选无效值时，JOB 级调度策略才会生效，TASK 级策略有效时自动忽略 JOB 级调度参数
        STRICT_ORDER,
        START_ORDER,
        IGNORE_ORDER,
        BUTT
    };

    struct Attribute {
        EventSet triggerEvents {EventSet::Relation::LOGIC_OR};   // 触发 JOB 运行的事件，或、与两种关系只能选其一
        EventSet ejectionEvents {EventSet::Relation::LOGIC_OR};  // JOB 可能输出的事件，同上一项，由使用者赋正确的值
        uint32_t execTimeLimit {0U}; // millisecond，由使用者赋正确的值
        uint32_t memLimit {0U};      // Byte，由使用者赋正确的值
        TargetProcessor execTarget {TargetProcessor::CPU};
        ScheduleMode mode {ScheduleMode::INVALID}; // TASK 级调度策略选无效值时，JOB 级调度策略才会生效，否则忽略此参数
    };
    /*
     * 预期的执行时间execTimeLimit和预期的内存消耗memLimit用于维测
     * 目标处理器execTarget当前只能选CPU
     */
    Job(const std::string& name, const Attribute& attr);
    virtual ~Job();

    /*
     * Exec为算法执行实体，框架提供的Job类为抽象类，用户需要继承该类后，自定义内部的算法执行函数
     */
    virtual void Exec(const std::shared_ptr<Context>& pipelineContext) = 0;

    /*
     * Job执行过程中可以通过该接口输出一个event, 由框架根据该event驱动下一个Job
     * 该接口不能用于驱动创建新的pipeline或者切换TASK
     */
    bool PubJobEvent(const Event& e);

    /*
     * 用户根据当前场景，选择DAG路径，配置当前job的下一个job
     * 注意： 如果当前 task 中有pipeline正在运行，直接修改 DAG 路径可能导致正在运行的pipeline按照新路径运行，
       如果希望避免该种情况，则用户应当确认所有pipeline运行完成后再切换 DAG 路径
     */
    bool PubNextJob(const std::string& nextJobName);

    const std::string& GetJobName() const;
    const EventSet& GetTriggerEvents() const;
    const EventSet& GetEjectionEvents() const;
    uint32_t GetExecTimeLimit() const;
    uint32_t GetMemLimit() const;
    TargetProcessor GetTargetProcessor() const;
    std::shared_ptr<Task> GetTask() const;
    bool operator==(const Job&) const;
    BasicJob& GetBasicJob() const;  // HCFI 内部使用接口

private:
    Job(const Job&) = delete;
    Job& operator=(const Job&) = delete;
    Job(Job&&) = delete;
    Job& operator=(Job&&) = delete;
    std::unique_ptr<BasicJob> basicJobIns_;
};
} // namespace hcfi
} // namespace mdc
#endif
