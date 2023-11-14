/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class RtfView.
 *      RtfView is for publish messages
 * Author: xujunbin 00514454
 * Create: 2019-11-30
 * Notes: NA
 * History: 2019-12-14 xujunbin 00514454 revised this file.
 */
#ifndef RTF_VIEW_H
#define RTF_VIEW_H

#include "rtf/internal/RtfMsgEntity.h"
#include "rtf/internal/RtfBagFile.h"
#include "rtf/internal/RtfBagStructs.h"
#include "ara/core/string.h"
#include "ara/core/vector.h"

namespace rtf {
namespace rtfbag {
const uint64_t TIME_MIN = 0U;
const uint64_t TIME_MAX = UINT64_MAX;

class RtfView {
public:
    class Iterator {
        friend class RtfView;
    public:
        Iterator(RtfView const& view);
        ~Iterator();

        Iterator& operator++()
        {
            (void) Increase();
            return *this;
        }

        RtfMsgEntity& Value();
        uint64_t BeginTime();
        bool IsEnd() const;

    protected:
        bool Populate();
        bool Increase();
        RtfMsgEntity& Dereference();
    private:
        RtfView const *view_;
        uint32_t size_;
        RtfMsgEntity* msgEntity_;
        ara::core::Vector<ViewIterHelper> iters_;
    };

    RtfView();
    ~RtfView();

    Iterator Begin() const;

    void AddQuery(RtfBagFile const& bag, uint64_t const& startTime = TIME_MIN, uint64_t const& endTime = TIME_MAX);
    void AddQuery(RtfBagFile const& bag, std::function<bool(Connection const&)> const& query,
        uint64_t const& startTime = TIME_MIN, uint64_t const& endTime = TIME_MAX);
    ara::core::Vector<Connection const*> GetConnections();
    uint32_t Size();
    uint64_t GetBeginTime();
    uint64_t GetEndTime();

protected:
    void UpdateQueries(RtfBagQuery const& bagQuery);
    RtfMsgEntity* CreateMsgEntity(Connection const& connection,
        MessageIndex const& msgIndex, RtfBagFile const& bag) const;

private:
    ara::core::Vector<MessageRange*> ranges_;
    ara::core::Vector<RtfBagQuery*>  queries_;
    uint64_t                         size_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif
