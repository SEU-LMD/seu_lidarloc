/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: evet controller header
 * Create: 2021-04-02
 */
#ifndef VRTF_VCC_UTILS_EVENT_CONTROLLER_H
#define VRTF_VCC_UTILS_EVENT_CONTROLLER_H
#include <map>
#include <functional>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "ara/hwcommon/log/log.h"
namespace vrtf {
namespace vcc {
namespace utils {
constexpr std::uint32_t UNDEFINED_EVENT_UID = UINT32_MAX;
constexpr std::int64_t T_MILLISECOND = 1000000;
constexpr std::int64_t T_SECOND = 1000 * T_MILLISECOND;

enum class EventScheduleMode : uint32_t {
    ONE_TIME,
    PERIOD
};

int64_t GetCurMonoTime();

class EventHandle {
public:
    explicit EventHandle(std::uint32_t uid) : uid_(uid)
    {}

    ~EventHandle() = default;

    std::uint32_t GetUID() const
    {
        return uid_;
    }

    bool operator==(const EventHandle& other) const
    {
        return uid_ == other.GetUID();
    }

    bool operator<(const EventHandle& other) const
    {
        return uid_ < other.GetUID();
    }

private:
    std::uint32_t uid_;
};

struct Event {
    int64_t interval;
    int64_t scheduleTime;
    EventScheduleMode mode;
    std::function<void()> func;
    EventHandle handle;
};

class EventController {
public:
    /**
     * @brief EventController constructor
     */
    EventController();

    /**
     * @brief Get EventController singleton instance
     */
    static std::shared_ptr<EventController> GetInstance();

    /**
     * @brief EventController deconstructor
     */
    ~EventController();

    /**
     * @brief Create an event
     * @param[in] mode    the schedule mode of event
     * @param[in] func    the callback of event
     * @param[in] mode    the schedule time of event
     * @param[in] mode    the interval of event
     * @return A handle of the event
     */
    EventHandle NewEvent(EventScheduleMode mode, const std::function<void()> &func,
        int64_t scheduleTime = 0, int64_t interval = -1);

    /**
     * @brief Delete an event
     * @param[in] handle  the handle of event which is create by NewEvent
     */
    void DelEvent(const EventHandle &handle) noexcept;

    /**
     * @brief Insert an event
     * @param[in] handle  the handle of event which is create by NewEvent
     * @return whether insert is success or not
     */
    bool InsertEvent(const EventHandle &handle);

private:
    /**
     * @brief Start the event controller
     */
    void StartController();

    /**
     * @brief Stop the event controller
     */
    void StopController() noexcept;

    /**
     * @brief Reschedule the event which mode is PERIOD
     * @param[in] event  the event need to be reschedule
     */
    void RescheduleEvent(Event &event);

    /**
     * @brief the function worked by thread which contained in event controller
     */
    void TaskEntry();
    bool isStop_;
    bool isSelfThread_;
    /* Because of nanoseconds may same in different events, so make a pair(nanoseconds, eventID) as handlerMap key */
    std::map<std::pair<int64_t, std::uint32_t>, Event> handlerMap_;
    std::thread workThread_;
    std::mutex handlerMutex_;
    std::mutex handleMutex_;
    std::condition_variable cond_;
    std::map<EventHandle, Event> handleList_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    static std::shared_ptr<EventController> instance_;
    static std::mutex singletonMutex_;
};
}
}
}
#endif
