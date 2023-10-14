/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class RtfMsgEntity.
 *      MessageEntity is deisgned to store message
 * Author: xujunbin 00514454
 * Create: 2019-11-30
 * Notes: NA
 * History: 2019-12-05 xujunbin 00514454 revised this file.
 */
#ifndef RTF_MSG_ENTITY_H
#define RTF_MSG_ENTITY_H

#include <set>
#include <functional>

#include "rtf/internal/RtfBagFile.h"
#include "rtf/internal/RtfBagStructs.h"
#include "rtf/internal/RtfBuffer.h"
#include "ara/core/string.h"
#include "ara/core/vector.h"

namespace rtf {
namespace rtfbag {
struct RtfTrueQuery {
    bool operator()(Connection const&) const
    {
        return true;
    }
};

struct RtfEventQuery {
    RtfEventQuery(ara::core::String const& event);
    RtfEventQuery(ara::core::Vector<ara::core::String> const& events);

    bool operator()(Connection const&) const;
    ara::core::Vector<ara::core::String> events_;
};

struct RtfQuery {
    RtfQuery(std::function<bool(Connection const&)> const& query, uint64_t const& startTime, uint64_t const& endTime);

    std::function<bool(Connection const&)> query_;
    uint64_t startTime_;
    uint64_t endTime_;
};

struct RtfBagQuery {
    RtfBagQuery(RtfBagFile const& bag, RtfQuery const& query);

    RtfBagFile const *bag_;
    RtfQuery query_;
};

struct MessageRange	{
    MessageRange(std::multiset<MessageIndex>::const_iterator const& begin,
        std::multiset<MessageIndex>::const_iterator const& end,
        Connection const& connection, RtfBagQuery const& bagQuery);
    std::multiset<MessageIndex>::const_iterator begin_;
    std::multiset<MessageIndex>::const_iterator end_;
    Connection const *connection_;
    RtfBagQuery const *bagQuery_;
};

struct ViewIterHelper {
    ViewIterHelper(std::multiset<MessageIndex>::const_iterator const& iter, MessageRange const& range);
    std::multiset<MessageIndex>::const_iterator iter_;
    MessageRange const *range_;
};

struct ViewIterHelpCompare {
    bool operator()(ViewIterHelper const& first, ViewIterHelper const& second) const;
};

class RtfMsgEntity {
public:
    RtfMsgEntity(Connection const& conn, MessageIndex const& msgIndex, RtfBagFile const& bag);
    ~RtfMsgEntity();

    bool WriteMsg(RtfBuffer& buffer) const;
    ara::core::String GetEvent() const;
    ara::core::String GetDataType() const;
    uint64_t GetTime() const;
    bool GetMsgSize(uint32_t& size) const;
    const Connection& GetConnection() const;

private:
    Connection const *conn_;
    MessageIndex const *msgIndex_;
    RtfBagFile const *bag_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif
