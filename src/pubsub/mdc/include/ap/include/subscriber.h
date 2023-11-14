/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: Subscriber
 */

#ifndef VIZ_LIB_SUBSCRIBER_H
#define VIZ_LIB_SUBSCRIBER_H

#include <cstdint>
#include "display_types/viz_point.h"

namespace mdc {
namespace visual {
class Subscriber {
public:
    Subscriber() = default;

    /*
     * 功能：消息订阅接口
     * 注意：收到对应topic消息后，会通过callback将数据回调给客户
     */
    Subscriber(const ara::core::String &topic, const uint8_t * const traits, const uint32_t &traitsLength,
        const std::function<void(const uint8_t*, const uint32_t)> &func);

    Subscriber(const ara::core::String &topic, const RecvPointStamped callback);

    ~Subscriber();
    // 获取topic接收到的消息个数
    uint64_t GetReceivedMsgNum() const;
private:
    ara::core::String subTopic_;
};
}
}

#endif // VIZ_LIB_SUBSCRIBER_H
