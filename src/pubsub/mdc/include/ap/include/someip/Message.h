/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 */

#ifndef SOMEIP_MESSAGE_H
#define SOMEIP_MESSAGE_H

#include <memory>
#include <someip/SomeipTypes.h>
#include <someip/SomeipEnumTypes.h>

namespace Someip {
    class Payload;

    class Message {
    public:
        virtual ~Message() {}
        virtual MessageID GetMessageId() const = 0;
        virtual void SetMessageId(MessageID message) = 0;
        virtual ServiceID GetServiceId() const = 0;
        virtual void SetServiceId(ServiceID service) = 0;
        virtual InstanceID GetInstanceId() const = 0;
        virtual void SetInstanceId(InstanceID instance) = 0;
        virtual MethodID GetMethodId() const = 0;
        virtual void SetMethodId(MethodID method) = 0;
        virtual uint32_t GetPayloadLength() const = 0;
        virtual RequestID GetRequestId() const = 0;
        virtual ClientID GetClientId() const = 0;
        virtual void SetClientId(ClientID client) = 0;
        virtual SessionID GetSessionId() const = 0;
        virtual void SetSessionId(SessionID session) = 0;
        virtual ProtocolVersion GetProtocolVersion() const = 0;
        virtual InterfaceVersion GetInterfaceVersion() const = 0;
        virtual void SetInterfaceVersion(InterfaceVersion version) = 0;
        virtual MessageType GetMessageType() const = 0;
        virtual void SetMessageType(MessageType type) = 0;
        virtual ErrorCode GetReturnCode() const = 0;
        virtual void SetReturnCode(ErrorCode code) = 0;
        virtual bool IsReliableMode() const = 0;
        virtual void SetReliableMode(bool isReliable) = 0;
        virtual bool IsInitialEvent() const = 0;
        virtual void SetInitialEvent(bool isInitial) = 0;
        virtual bool IsValidCrc() const = 0;
        virtual void SetIsValidCrc(bool isValidCrc) = 0;
        virtual std::shared_ptr<Payload> GetPayload() const = 0;
        virtual void SetPayload(std::shared_ptr<Payload> payload) = 0;
        virtual void AnchorTimestamp() = 0;
        virtual std::int64_t GetTimestamp() = 0;
    };
}

#endif