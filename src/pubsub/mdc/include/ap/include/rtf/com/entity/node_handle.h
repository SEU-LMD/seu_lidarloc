/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This is a node can be used in sub or pub function
 * Create: 2020-04-22
 */
#ifndef RTF_COM_NODE_HANDLE_H_
#define RTF_COM_NODE_HANDLE_H_

#include "rtf/com/adapter/node_handle_adapter.h"
#include <functional>
#include "rtf/com/init.h"
#include "rtf/com/entity/thread_group.h"

namespace rtf {
namespace com {
/**
 * @brief The class to control a node
 */
class NodeHandle {
public:
    /**
     * @brief The default constructor of NodeHandle
     * @param[in] ns    The namespace of NodeHandle (default value "")
     */
    NodeHandle(const std::string& nameSpace = "");

    /**
     * @brief The default destructor of NodeHandle
     */
    ~NodeHandle(void) = default;

    /**
     * @brief NodeHandle copy constructor
     * @note deleted
     * @param[in] other    Other instance
     */
    NodeHandle(const NodeHandle& other) = delete;

    /**
     * @brief NodeHandle move constructor
     * @param[in] other    Other instance
     */
    NodeHandle(NodeHandle && other);

    /**
     * @brief NodeHandle copy assign operator
     * @note deleted
     * @param[in] other    Other instance
     */
    NodeHandle& operator=(const NodeHandle& other) = delete;

    /**
     * @brief NodeHandle move assign operator
     * @param[in] other    Other instance
     */
    NodeHandle& operator=(NodeHandle && other);

    /**
     * @brief Advertise an event on the node
     * @param[in] eventName    The name of the event
     * @return The publisher of the event
     */
    template<class EventDataType>
    Publisher<EventDataType> Advertise(const std::string& eventName) noexcept
    {
        using namespace rtf::com::utils;

        if (IsInitialized() && isValidNamespace_) {
            if (adapter_.Initialization(AdapterType::EVENT, Role::SERVER)) {
                return adapter_.Advertise<EventDataType>(GetEntityURI(eventName));
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Initialize NodeHandle failed of " << GetEntityURI(eventName);
                return Publisher<EventDataType>();
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Node is not initialized, or using invalid NodeHandle's namespace";
            return Publisher<EventDataType>();
        }
    }

    /**
     * @brief Send an event subscription request and create a subscriber using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The callback function for processing received data
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber Subscribe(const std::string& eventName, uint32_t queueSize,
                         std::function<void(EventDataType)> callback,
                         ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        if (IsValidCreateSubscriber(eventName, queueSize) && (callback != nullptr)) {
            return adapter_.Subscribe<EventDataType>(GetEntityURI(eventName), queueSize, callback, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Send an event subscription request and create a subscriber using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The callback function for processing received data and plog msg id
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber SubscribeEx(const std::string& eventName, uint32_t queueSize,
        std::function<void(EventDataType, const SampleInfo&)> callback = nullptr,
        ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        if (IsValidCreateSubscriber(eventName, queueSize)) {
            return adapter_.Subscribe<EventDataType>(GetEntityURI(eventName), queueSize, callback, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function
     * @param[in] instance     The class instance of the event callback function
     * @return The subscriber of the event
     */
    template<class EventDataType, class T>
    Subscriber Subscribe(const std::string& eventName, uint32_t queueSize, void(T::*callback)(EventDataType),
                         T* instance,
                         ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        const bool invalidCallback = (callback == nullptr) || (instance == nullptr);
        if (IsValidCreateSubscriber(eventName, queueSize) && (!invalidCallback)) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback, instance](EventDataType data) {
                    (instance->*callback)(std::move(data));
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function and plog msg id
     * @param[in] instance     The class instance of the event callback function
     * @return The subscriber of the event
     */
    template<class EventDataType, class T>
    Subscriber SubscribeEx(const std::string& eventName, uint32_t queueSize,
        void(T::*callback)(EventDataType, const SampleInfo&), T* instance,
        ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        const bool invalidCallback = (callback == nullptr) || (instance == nullptr);
        if (IsValidCreateSubscriber(eventName, queueSize) && (!invalidCallback)) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback, instance](EventDataType data, const SampleInfo& info) {
                    (instance->*callback)(std::move(data), info);
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function
     * @param[in] instance     The class instance of the event callback function
     * @return The subscriber of the event
     */
    template<class EventDataType, class T>
    Subscriber Subscribe(const std::string& eventName, uint32_t queueSize, void(T::*callback)(EventDataType) const,
                         T* instance,
                         ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        const bool invalidCallback = (callback == nullptr) || (instance == nullptr);
        if (IsValidCreateSubscriber(eventName, queueSize) && (!invalidCallback)) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback, instance](EventDataType data) {
                    (instance->*callback)(std::move(data));
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function and plog msg id
     * @param[in] instance     The class instance of the event callback function
     * @return The subscriber of the event
     */
    template<class EventDataType, class T>
    Subscriber SubscribeEx(const std::string& eventName, uint32_t queueSize,
        void(T::*callback)(EventDataType, const SampleInfo&) const, T* instance,
        ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        const bool invalidCallback = (callback == nullptr) || (instance == nullptr);
        if (IsValidCreateSubscriber(eventName, queueSize) && invalidCallback) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback, instance](EventDataType data, const SampleInfo& info) {
                    (instance->*callback)(std::move(data), info);
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber Subscribe(const std::string& eventName, uint32_t queueSize,
                         void(*callback)(EventDataType),
                         ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        if (IsValidCreateSubscriber(eventName, queueSize) && (callback != nullptr)) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback](EventDataType data) {
                    (*callback)(std::move(data));
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Subscribe an event using specific ThreadGroup
     * @param[in] eventName    The name of the event
     * @param[in] queueSize    The size of the queue
     * @param[in] callback     The event callback function and plog msg uid
     * @return The subscriber of the event
     */
    template<class EventDataType>
    Subscriber SubscribeEx(const std::string& eventName, uint32_t queueSize,
        void(*callback)(EventDataType, const SampleInfo&),
        ThreadGroup& threadGroup = adapter::NodeHandleAdapter::GetDefaultThreadGroup()) noexcept
    {
        if (IsValidCreateSubscriber(eventName, queueSize) && (callback != nullptr)) {
            return adapter_.Subscribe<EventDataType>(
                GetEntityURI(eventName), queueSize, [callback](EventDataType data, const SampleInfo& info) {
                    (*callback)(std::move(data), info);
                }, threadGroup);
        } else {
            return Subscriber();
        }
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function
     * @return The server of a specific method
     */
    template<class Request, class Reply>
    rtf::com::MethodServer RegisterMethod(const std::string& methodName,
                                          const std::function<bool(Request&, Reply&)>& callback)
    {
        using namespace rtf::com::utils;
        if (IsInitialized() && isValidNamespace_ && callback != nullptr) {
            if (adapter_.Initialization(AdapterType::METHOD, Role::SERVER)) {
                return adapter_.RegisterMethod(GetEntityURI(methodName), callback);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Initialize NodeHandle failed of " << GetEntityURI(methodName);
                return MethodServer();
            }
        } else {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logger_->Error() <<
                            "[RTFCOM] Node is not initialized, null callback, or using invalid NodeHandle's namespace";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return MethodServer();
        }
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function with E2E
     * @return The server of a specific method
     */
    template<class Request, class Reply>
    rtf::com::MethodServer RegisterMethodEx(const std::string& methodName,
                                          const std::function<bool(Request&, Reply&, MethodServerResult&)>& callback)
    {
        using namespace rtf::com::utils;

        if (IsInitialized() && isValidNamespace_ && callback != nullptr) {
            if (adapter_.Initialization(AdapterType::METHOD, Role::SERVER)) {
                return adapter_.RegisterMethod(GetEntityURI(methodName), callback);
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Initialize NodeHandle failed of " << GetEntityURI(methodName);
                return MethodServer();
            }
        } else {
            /* AXIVION disable style AutosarC++19_03-A5.1.1: Records the log */
            /* AXIVION disable style AutosarC++19_03-A5.0.1: Records the log */
            logger_->Error() <<
                "[RTFCOM] Node is not initialized, null callback, or using invalid NodeHandle's namespace";
            /* AXIVION enable style AutosarC++19_03-A5.0.1 */
            /* AXIVION enable style AutosarC++19_03-A5.1.1 */
            return MethodServer();
        }
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function
     * @param[in] instance      The class instance of the method callback function
     * @return The server of a specific method
     */
    template<class Request, class Reply, class T>
    rtf::com::MethodServer RegisterMethod(const std::string& methodName,
                                bool(T::*callback)(Request&, Reply&),
                                T* instance) noexcept
    {
        std::function<bool(Request&, Reply&)> wrappedCallback = std::bind(callback, instance,
                                                                          std::placeholders::_1,
                                                                          std::placeholders::_2);
        return RegisterMethod(methodName, wrappedCallback);
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function with E2E
     * @param[in] instance      The class instance of the method callback function
     * @return The server of a specific method
     */
    template<class Request, class Reply, class T>
    rtf::com::MethodServer RegisterMethodEx(const std::string& methodName,
                                bool(T::*callback)(Request&, Reply&, MethodServerResult&),
                                T* instance) noexcept
    {
        std::function<bool(Request&, Reply&, MethodServerResult&)> wrappedCallback = std::bind(callback, instance,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        return RegisterMethodEx(methodName, wrappedCallback);
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function
     * @return The server of a specific method
     */
    template<class Request, class Reply>
    rtf::com::MethodServer RegisterMethod(const std::string& methodName, bool(*callback)(Request&, Reply&))
    {
        std::function<bool(Request&, Reply&)> wrappedCallback =
            std::bind(callback, std::placeholders::_1, std::placeholders::_2);
        return RegisterMethod(methodName, wrappedCallback);
    }

    /**
     * @brief Create a method server for a specific method
     * @param[in] methodName    The name of the method
     * @param[in] callback      The method callback function with E2E
     * @return The server of a specific method
     */
    template<class Request, class Reply>
    rtf::com::MethodServer RegisterMethodEx(const std::string& methodName,
                                            bool(*callback)(Request&, Reply&, MethodServerResult&))
    {
        std::function<bool(Request&, Reply&, MethodServerResult&)> wrappedCallback =
            std::bind(callback, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        return RegisterMethodEx(methodName, wrappedCallback);
    }

    /**
     * @brief Create a client for a specific method
     * @param[in] methodName    The name of method
     * @return The client of the method with the methodName
     */
    template<class MethodDataType>
    MethodClient CreateMethodClient(const std::string& methodName) noexcept
    {
        using namespace rtf::com::utils;

        if (IsInitialized() && isValidNamespace_) {
            if (adapter_.Initialization(AdapterType::METHOD, Role::CLIENT)) {
                return adapter_.CreateMethodClient<MethodDataType>(GetEntityURI(methodName));
            } else {
                /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
                logger_->Error() << "[RTFCOM] Initialize NodeHandle failed of " << GetEntityURI(methodName);
                return MethodClient();
            }
        } else {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Error() << "[RTFCOM] Node is not initialized, or using invalid NodeHandle's namespace";
            return MethodClient();
        }
    }

    /**
     * @brief Set the thread number that handle all callbacks and its queue length
     * @param[in] threadNumber    The number of thread, default is 1
     * @param[in] queueSize       The size of the queue, default is 1024
     * @return Operation result
     */
    bool SetMultiThread(std::uint16_t threadNumber = 1, std::uint16_t queueSize = 1024) noexcept;

    /**
     * @brief Close all connections within the node
     */
    void Shutdown(void) noexcept;
private:
    std::string nameSpace_;
    adapter::NodeHandleAdapter adapter_;
    bool isValidNamespace_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
    std::string GetEntityURI(const std::string& entityName) const noexcept;
    bool IsValidCreateSubscriber(const std::string& eventName, std::uint32_t queueSize) noexcept;
};
} // namespace com
} // namespace rtf

#endif // RTF_COM_NODE_HANDLE_H_
