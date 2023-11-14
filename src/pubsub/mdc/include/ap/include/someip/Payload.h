/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */

#ifndef SOMEIP_PAYLOAD_H
#define SOMEIP_PAYLOAD_H

#include <vector>
#include <someip/SomeipTypes.h>

namespace Someip {
    class Payload {
    public:
        virtual ~Payload() = default;
        virtual bool operator==(const Payload &other) = 0;
        virtual uint8_t *GetData() = 0;
        virtual const uint8_t *GetData() const = 0;
        virtual void SetData(const uint8_t *data, uint32_t length) = 0;
        virtual void SetData(const std::vector<uint8_t> &data) = 0;
        virtual void SetData(std::vector<uint8_t> &&data) = 0;
        virtual uint32_t GetLength() const = 0;
        virtual void SetCapacity(uint32_t length) = 0;
        virtual SessionID GetE2ESessionId() const = 0;
        virtual void SetE2ESessionId(SessionID session) = 0;
        virtual InterfaceVersion GetE2EInterfaceVersion() const = 0;
        virtual void SetE2EInterfaceVersion(InterfaceVersion version) = 0;
        virtual void SetE2EEnabled(bool enabled) = 0;
        virtual bool IsE2EEnabled() const = 0;
        virtual const uint8_t *GetE2EData() const = 0;
    };
}

#endif