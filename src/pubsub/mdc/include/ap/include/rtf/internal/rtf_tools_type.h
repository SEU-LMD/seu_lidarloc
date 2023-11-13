/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Define types in rtftools
 * Create: 2020-10-24
 */
#ifndef RTF_TOOLS_TYPES_H
#define RTF_TOOLS_TYPES_H
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "ara/core/map.h"
namespace rtf {
namespace rtfevent {
/**
 * @brief the class to store basic event info
 * @example Name Example <br>
 * EventName = ${EventType}[${InstanceShortname}] <br>
 * @note instanceShortName_ could be empty
 */
class RtfEventInfo {
public:
    ara::core::Vector<ara::core::String> GetSubs() const
    {
        return subList_;
    }
    void SetSubs(ara::core::Vector<ara::core::String> subs)
    {
        subList_.swap(subs);
    }
    ara::core::String GetPub() const
    {
        return pub_;
    }
    void SetPub(const ara::core::String pub)
    {
        pub_ = pub;
    }
    ara::core::String GetEventName() const
    {
        auto res = eventType_;
        if (!instanceShortName_.empty()) {
            res += '[' + instanceShortName_ + ']';
        }
        return res;
    }
    void SetEventName(const ara::core::String eventName)
    {
        ara::core::String eventNameTemp = eventName;
        ara::core::String instanceShortNameTemp = "";
        auto leftBracket = eventName.find_first_of('[');
        auto rightBracket = eventName.find_last_of(']');
        if (leftBracket != ara::core::String::npos && rightBracket != ara::core::String::npos &&
            leftBracket < rightBracket) {
                eventNameTemp = eventName.substr(0, leftBracket);
                instanceShortNameTemp = eventName.substr(leftBracket + 1, rightBracket - leftBracket - 1);
        }
        eventType_ = eventNameTemp;
        instanceShortName_ = instanceShortNameTemp;
    }
    const ara::core::String& GetInstanceShortName() const
    {
        return instanceShortName_;
    }
    void SetInstanceShortName(ara::core::String instanceName)
    {
        instanceShortName_ = std::move(instanceName);
    }
    ara::core::String GetEventType() const
    {
        return eventType_;
    }
    void SetEventType(const ara::core::String type)
    {
        eventType_ = type;
    }
    void SetAttribute(const ara::core::Map<ara::core::String, ara::core::String>& attributeList)
    {
        attributeList_ = attributeList;
    }
    ara::core::Map<ara::core::String, ara::core::String> GetAttribute() const
    {
        return attributeList_;
    }
private:
    ara::core::String pub_;
    ara::core::Vector<ara::core::String> subList_;
    ara::core::String eventType_;
    /** instance name is surrounded by '[' ']' */
    ara::core::String instanceShortName_;
    ara::core::Map<ara::core::String, ara::core::String> attributeList_;
};

class EventFilter {
public:
    enum class Type: uint8_t {
        PUBLISHER,
        SUBSCRIBER,
        ALL
    };
    EventFilter(const Type &type)
    {
        filterType_ = type;
    }
    ~EventFilter() = default;
    void SetType(const Type &type)
    {
        filterType_ = type;
    }
    Type GetType() const
    {
        return filterType_;
    }
private:
    Type filterType_;
};
}

namespace rtfmethodcall {
    struct BitBuffer {
        std::uint8_t*   dataBuffer;
        size_t      offset;
    };
}
}
#endif
