/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: action base
 * Author: z00398416
 * Create: 2021-03-02
 */
#ifndef FM_FAULT_ACTION_H
#define FM_FAULT_ACTION_H
#include <cstdint>
#include <vector>
#include <string>

namespace mdc {
namespace fm {
class FmFaultAction {
public:
    FmFaultAction() = default;
    virtual ~FmFaultAction() = default;

    virtual std::int32_t DoInit() = 0;
    virtual std::int32_t DeInit() = 0;
    virtual std::int32_t AlarmReport(const std::uint16_t alarmId, const std::uint16_t alarmObject,
        const std::uint16_t alarmStatus) = 0;
protected:
    FmFaultAction(const FmFaultAction& rfs) : m_name(rfs.m_name), m_confPath(rfs.m_confPath)
    {}
    FmFaultAction& operator=(const FmFaultAction& rfs)
    {
        m_name = rfs.m_name;
        m_confPath = rfs.m_confPath;
        return *this;
    }

private:
    std::string m_name;
    std::string m_confPath;
};
}
}
#endif