/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef MDC_DIAG_CONVERSION_H
#define MDC_DIAG_CONVERSION_H
#include <vector>
#include <string>
#include <functional>
#include "mdc/common/result.h"
#include "mdc/common/future.h"
#include "mdc/diag/meta_info.h"
#include "mdc/diag/cancellation_handler.h"

namespace mdc {
namespace diag {
namespace internal {
class ConversationManagerAgent;
}
enum class ActivityStatusType: uint8_t {
    kActive = 0x00,
    kInactive = 0x01
};

struct ConversationIdentifierType {
    ConversationIdentifierType() : protocolId(0U), connectionId(0U), sourceAddress(0U) {}
    ConversationIdentifierType(const uint8_t protocolIdLocal, const uint32_t connectionIdLocal,
        const uint16_t sourceAddressLocal)
        : protocolId(protocolIdLocal), connectionId(connectionIdLocal), sourceAddress(sourceAddressLocal) {}
    uint8_t protocolId;     // TP层协议ID
    uint32_t connectionId;  // TP层连接ID
    uint16_t sourceAddress; // 源地址
};

enum class ControlAction : uint8_t {
    START = 0x00,
    MAINTAIN = 0x01,
    END = 0x02
};

class Conversation {
public:
    using ConversationIdType = uint16_t;
    Conversation();
    Conversation(const ConversationIdType conversationId, const uint32_t sn);
    Conversation(const Conversation &other) = delete;
    Conversation(Conversation &&other);
    Conversation &operator=(const Conversation &other) = delete;
    Conversation &operator=(Conversation &&other);
    virtual ~Conversation();

    static mdc::common::Result<mdc::diag::Conversation> GetConversation(MetaInfo metaInfo);
    static std::vector<mdc::diag::Conversation> GetAllConversations();
    static std::vector<mdc::diag::Conversation> GetCurrentActiveConversations();

    mdc::common::Result<mdc::diag::ActivityStatusType> GetActivityStatus();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    mdc::common::Result<void> SetActivityNotifier(std::function<void(mdc::diag::ActivityStatusType)> notifier);
    mdc::common::Result<mdc::diag::ConversationIdentifierType> GetConversationIdentifier();
    mdc::common::Result<std::string> GetDiagnosticSession();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    mdc::common::Result<void> SetDiagnosticSessionNotifier(std::function<void(std::string)> notifier);
    mdc::common::Result<std::string> GetDiagnosticSecurityLevel();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    mdc::common::Result<void> SetSecurityLevelNotifier(std::function<void(std::string)> notifier);
    /* deprecated */
    mdc::common::Result<void> ResetToDefaultSession();
    /* deprecated */
    mdc::common::Result<void> Cancel();

    mdc::common::Result<void> ControlUpgradeState(const ControlAction action) const;

private:
    ConversationIdType conversationId_;
    uint64_t sn_;
    std::shared_ptr<internal::ConversationManagerAgent> agent_;
};
}
}
#endif // MDC_DIAG_CONVERSION_H