/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: adsfi base
 */

#ifndef HAF_ADSF_ADSFI_BASE_H
#define HAF_ADSF_ADSFI_BASE_H

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#ifndef MDC_COMPILER_AOS_SEA_LLVM
#include "core/core.h"
#if defined(MDC_PRODUCTION_CORE)
#include "ara/rm/rm.h"
#endif // MDC_PRODUCTION_CORE
#endif
#include "core/status.h"
#include "yaml/haf_yaml.h"

#define RETURN_IF_ERROR(ret, str) \
if (!(ret)) { \
    HAF_LOG_ERROR << "Cannot find " << (str) << " in Config File!"; \
    return (HAF_ERROR); \
}
#define COUT_IF_ERROR(ret, str) \
if (!(ret)) { \
    std::cout << "Cannot find " << (str) << " in Config File!" << std::endl; \
    return (HAF_ERROR); \
}
namespace Adsfi {
class AdsfiBase {
public:
    AdsfiBase() = default;
    explicit AdsfiBase(const std::string configFile) : configFile_(configFile) {};
    virtual HafStatus Init();
    /**
     * @brief 启动订阅数据过程子线程
     *
     * @return HafStatus
     */
    virtual HafStatus StartSubThread() = 0;
    /**
     * @brief 启动发布数据过程子线程
     *
     * @return HafStatus
     */
    virtual HafStatus StartPubThread() = 0;
    /**
     * @brief 发送程序停止信号，释放资源；同时唤醒所有阻塞的收发线程，使其可以进行停止信号的判断并退出（外部需要结合IsStop()控制线程退出）。
     *
     */
    virtual void Stop() = 0;
    /**
     * @brief 查询程序接收到停止信号。
     *
     * @return true
     * @return false
     */
    bool IsStop() const
    {
        return stopFlag_;
    }
    std::string GetConfigName() const
    {
        return configFile_;
    }
    virtual ~AdsfiBase();
    virtual HafStatus ParsingInstanceId(const HafYamlNode& config) = 0;

protected:
    HafStatus LoadCommonConfig(const HafYamlNode& config);

    // 删除vector中的重复元素，留下的元素相对位置不变
    template <class T>
    void StableUnique(std::vector<T>& vv)
    {
        std::unordered_map<T, bool> flag;
        for (auto it = vv.begin(); it != vv.end();) {
            if (!flag[*it]) {
                flag[*it] = true;
                ++it;
            } else {
                it = vv.erase(it);
            }
        }
    }

    // 将只有一层普通数据类型的vector，排列为string，以便于打印
    template <class T>
    std::string Vec2Str(const std::vector<T>& vec)
    {
        static_assert(std::is_pod<T>::value, "Template T must be plain old data (POD) type");
        std::ostringstream oss;
        if (!vec.empty()) {
            std::copy(vec.begin(), vec.end() - 1, std::ostream_iterator<T>(oss, ", "));
            oss << vec.back();
        }
        return oss.str();
    }

protected:
    bool stopFlag_{false};
    bool initState_{false};
private:
    std::string configFile_{"Config.yaml"};   // 参数文件
    std::string logFile_{"./"};               // 日志文件路径
    std::string name_{"NODE"};                // 节点名称（模块）
    int32_t priority_{-1};                    // 节点优先级
    std::string description_{"description"};  // 节点描述
    bool isDetermineSchedule_{false};         // 是否是确定性调
    int32_t scheduleFrequency_{30};           // 调度频率(Hz)
    int32_t logMode_{static_cast<int32_t>(HafLogModeType::HAF_REMOTE)}; // 日志模式
    int32_t logLevel_{static_cast<int32_t>(HafLogLevelType::HAF_WARN)}; // 日志级别
};
}  // namespace Adsfi
#endif  // HAF_ADSF_ADSFI_BASE_H
