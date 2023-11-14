/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class Event declaration
*/

#ifndef HCFI_EVENT_H
#define HCFI_EVENT_H

#include <string>

namespace mdc {
namespace hcfi {
class Event {
public:
    explicit Event(const std::string& name);
    ~Event() = default;
    bool operator==(const Event& other) const;
    bool operator<(const Event& other) const;
    const std::string& GetName() const;

private:
    std::string name_;
};
} // namespace hcfi
} // namespace mdc
#endif