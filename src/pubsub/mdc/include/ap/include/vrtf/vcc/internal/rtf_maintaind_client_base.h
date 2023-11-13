/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Add this to control ctor/dtor sequence.
 * Create: 2020-03-26
 */
#ifndef RTF_MAINTAIND_CLIENT_BASE_H
#define RTF_MAINTAIND_CLIENT_BASE_H
#include <string>
namespace vrtf {
namespace vcc {
class RtfMaintaindClientBase {
public:
    RtfMaintaindClientBase() {};
    virtual ~RtfMaintaindClientBase() = default;
    virtual void Initialize(const std::string& serviceName) = 0;
    virtual void Initialize(const uint16_t& serviceId) = 0;
};
}
}
#endif
