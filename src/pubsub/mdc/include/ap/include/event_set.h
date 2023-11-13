/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
* Description: class EventSet declaration
*/

#ifndef HCFI_EVENT_SET_H
#define HCFI_EVENT_SET_H

#include <vector>
#include <string>
#include <event.h>

namespace mdc {
namespace hcfi {
class EventSet {
public:
    enum class Relation : uint8_t {
        LOGIC_AND,
        LOGIC_OR
    };
    explicit EventSet(const Relation rel);
    ~EventSet() = default;

    void AddEvent(const Event& e);
    Relation GetRelation() const;
    const std::vector<Event>& GetEvents() const;
private:
    const Relation relat_;
    std::vector<Event> events_;
};
} // namespace hcfi
} // namespace mdc
#endif