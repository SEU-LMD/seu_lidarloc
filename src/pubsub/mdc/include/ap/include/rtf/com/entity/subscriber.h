/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Subscriber class
 * Create: 2020-04-22
 */
#ifndef RTF_COM_SUBSCRIBER_H_
#define RTF_COM_SUBSCRIBER_H_

#include "rtf/com/adapter/ros_proxy_adapter.h"
#include "rtf/com/utils/logger.h"
namespace rtf {
namespace com {
class Subscriber {
public:
    template<typename T> struct is_shared_ptr : std::false_type {};
    template<typename T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};
    /**
     * @brief Publisher default constructor
     */
    Subscriber(void) = default;

    /**
     * @brief Publisher constructor
     * @param[in] adapter    The actual adapter that handles this subscriber
     */
    Subscriber(const std::shared_ptr<rtf::com::adapter::RosProxyAdapter>& adapter);

    /**
     * @brief Publisher default destructor
     */
    ~Subscriber(void) = default;

    /**
     * @brief Subscriber copy constructor
     * @note deleted
     * @param[in] other    Other instance
     */
    Subscriber(const Subscriber& other) = delete;

    /**
     * @brief Subscriber move constructor
     * @param[in] other    Other instance
     */
    Subscriber(Subscriber && other);

    /**
     * @brief Subscriber copy assign operator
     * @note deleted
     * @param[in] other    Other instance
     */
    Subscriber& operator=(const Subscriber& other) = delete;

    /**
     * @brief Subscriber move assign operator
     * @param[in] other    Other instance
     */
    Subscriber& operator=(Subscriber && other);

    /**
     * @brief Returns whether of the subscriber is created correctly
     * @return Is the subscriber created correctly
     */
    operator bool() const noexcept;

    /**
     * @brief close the connection
     * @return void
     */
    void Shutdown(void) noexcept;

    /**
     * @brief Get the Event Data
     *
     * @tparam    EventDataType       The type of received data
     * @param[in] callback            The event callback function and other corresponding info
     * @return std::size_t            The number of received data
     */
    template<typename EventDataType>
    std::size_t GetEventData(std::function<void(EventDataType, const SampleInfo&)> callback) noexcept
    {
        using namespace rtf::com::utils;
        if ((callback == nullptr) || (!CheckSynchronousInterface())) {
            return 0;
        } else {
            return adapter_->GetEventData<EventDataType>(callback);
        }
    }

    /**
     * @brief Get the Event Data
     *
     * @tparam    EventDataType       The type of received data
     * @tparam    T                   The type of class
     * @param[in] callback            The event callback function and other corresponding info
     * @param[in] instance            The class instance of the event callback function
     * @return std::size_t            The number of received data
     */
    template<typename EventDataType, typename T>
    std::size_t GetEventData(void(T::*callback)(EventDataType, const SampleInfo&), T* instance) noexcept
    {
        using namespace rtf::com::utils;
        if ((callback == nullptr) || (!CheckSynchronousInterface())) {
            return 0;
        } else {
            return adapter_->GetEventData<EventDataType>(
                [callback, instance](EventDataType data, const SampleInfo& info) {
                    (instance->*callback)(std::move(data), info);
            });
        }
    }

    /**
     * @brief Get the Event Data
     *
     * @tparam    EventDataType       The type of received data
     * @tparam    T                   The type of class
     * @param[in] callback            The event callback function and other corresponding info
     * @param[in] instance            The class instance of the event callback function
     * @return std::size_t            The number of received data
     */
    template<typename EventDataType, typename T>
    std::size_t GetEventData(void(T::*callback)(EventDataType, const SampleInfo&) const, T* instance) noexcept
    {
        using namespace rtf::com::utils;
        const bool invalidCallback = (callback == nullptr) || (instance == nullptr);
        if ((invalidCallback) || (!CheckSynchronousInterface())) {
            return 0;
        } else {
            return adapter_->GetEventData<EventDataType>(
                [callback, instance](EventDataType data, const SampleInfo& info) {
                    (instance->*callback)(std::move(data), info);
            });
        }
    }

    /**
     * @brief Get the Event Data
     *
     * @tparam    EventDataType       The type of received data
     * @param[in] callback            The received function for getting data
     * @return std::size_t            The number of received data
     */
    template<typename EventDataType>
    std::size_t GetEventData(void(*callback)(EventDataType, const SampleInfo&)) noexcept
    {
        using namespace rtf::com::utils;
        if ((callback == nullptr) || (!CheckSynchronousInterface())) {
            return 0;
        } else {
            return adapter_->GetEventData<EventDataType>([callback](EventDataType data, const SampleInfo& info) {
                callback(std::move(data), info);
            });
        }
    }
private:
    std::shared_ptr<adapter::RosProxyAdapter> adapter_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;

    /**
     * @brief Check the valid of the Subscriber and its' received mode
     *
     * @retval true    It is valid subscriber and using correct received mode
     * @retval false   It is an invalid subscriber or using wrong received mode
     */
    bool CheckSynchronousInterface() const noexcept;
};
} // namespace com
} // namespace rtf
#endif // RTF_COM_SUBSCRIBER_H_
