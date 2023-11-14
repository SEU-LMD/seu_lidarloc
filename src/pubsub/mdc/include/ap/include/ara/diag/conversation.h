/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description:
 */
#ifndef ARA_DIAG_CONVERSION_H
#define ARA_DIAG_CONVERSION_H
#include <functional>
#include "ara/core/result.h"
#include "ara/core/string.h"
#include "ara/core/vector.h"
#include "ara/diag/meta_info.h"
#include "ara/diag/cancellation_handler.h"

namespace mdc {
namespace diag {
namespace internal {
class ConversationManagerAgent;
}
}
}

namespace ara {
namespace diag {
enum class ControlAction : uint8_t {
    kStart = 0x00U,
    kMaintain = 0x01U,
    kEnd = 0x02U
};

class Conversation {
public:
    enum class ActivityStatusType: uint8_t {
        kActive = 0x00U,
        kInactive = 0x01U
    };

    struct ConversationIdentifierType {
        ConversationIdentifierType() : protocolId(0U), connectionId(0U), sourceAddress(0U) {}
        ConversationIdentifierType(const uint8_t protocolIdToSet,
            const uint32_t connectionIdToSet, const uint16_t sourceAddressToSet)
            : protocolId(protocolIdToSet), connectionId(connectionIdToSet), sourceAddress(sourceAddressToSet) {}
        uint8_t protocolId;     // TP Layer Protocol ID
        uint32_t connectionId;  // TP Layer Connection ID
        uint16_t sourceAddress;
    };

    using ConversationIdType = uint16_t;

    Conversation();
    Conversation(const ConversationIdType conversationId, const uint32_t sn);
    Conversation(const Conversation &other) = delete;
    Conversation(Conversation &&other);
    Conversation &operator=(Conversation &&other);
    Conversation &operator=(const Conversation &other) = delete;
    virtual ~Conversation();

    static ara::core::Result<Conversation> GetConversation(MetaInfo metaInfo);
    static ara::core::Vector<Conversation> GetAllConversations();
    static ara::core::Vector<Conversation> GetCurrentActiveConversations();

    ara::core::Result<ActivityStatusType> GetActivityStatus();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    ara::core::Result<void> SetActivityNotifier(std::function<void(ActivityStatusType)> notifier);
    ara::core::Result<ConversationIdentifierType> GetConversationIdentifier();
    ara::core::Result<ara::core::String> GetDiagnosticSession();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    ara::core::Result<void> SetDiagnosticSessionNotifier(std::function<void(ara::core::String)> notifier);
    ara::core::Result<ara::core::String> GetDiagnosticSecurityLevel();
    /*
    * params: notifier, if nullptr, means unset notifier
    */
    ara::core::Result<void> SetSecurityLevelNotifier(std::function<void(ara::core::String)> notifier);
    /* deprecated */
    ara::core::Result<void> ResetToDefaultSession();
    /* deprecated */
    ara::core::Result<void> Cancel();

private:
    ConversationIdType conversationId_;
    uint64_t sn_;
    std::shared_ptr<mdc::diag::internal::ConversationManagerAgent> agent_;
};
}
}
#endif // ARA_DIAG_CONVERSION_H_