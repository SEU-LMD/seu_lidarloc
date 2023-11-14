/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: to manager Driver common api
 * Create: 2019-07-24
 */
#ifndef VRTF_VCC_API_INTERNAL_DRIVERMANAGER_H
#define VRTF_VCC_API_INTERNAL_DRIVERMANAGER_H
#include <map>
#include <memory>
#include <vector>
#include <mutex>
#include <algorithm>
#include "vrtf/vcc/api/types.h"
#include "ara/hwcommon/log/log.h"
#include "vrtf/vcc/driver/event_handler.h"
#include "vrtf/vcc/driver/method_handler.h"
#include "vrtf/vcc/driver/driver.h"
namespace vrtf {
namespace vcc {
namespace api {
namespace internal {
class DriverManager {
public:
    DriverManager();
    DriverManager(const DriverManager& drm) = delete;
    ~DriverManager();
    /**
     * @brief register dds/someip driver
     *
     * @param type[in]    the type of driver, dds or someip
     * @param driver[in]  the instance of driver will be stored
     */
    void RegisterDriver(vrtf::vcc::api::types::DriverType type,
                               std::shared_ptr<vrtf::vcc::driver::Driver> driver);
    void Clear();
    std::shared_ptr<vrtf::vcc::driver::Driver> GetDriver(const vrtf::vcc::api::types::DriverType& type);

    std::shared_ptr<vrtf::vcc::driver::EventHandler> CreateEvent(
        const vrtf::vcc::api::types::DriverType type, std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo);
    std::shared_ptr<vrtf::vcc::driver::EventHandler> SubscribeEvent(
        const vrtf::vcc::api::types::DriverType type,
        const std::shared_ptr<vrtf::vcc::api::types::EventInfo>& eventInfo,
        const vrtf::vcc::api::types::EventHandleReceiveHandler& handler);
    void SetSubscriptionStateChangeHandler(vrtf::vcc::api::types::SubscriptionStateChangeHandler handler,
                                                   const vrtf::vcc::api::types::EntityId& id);
    void UnsetSubscriptionStateChangeHandler(const vrtf::vcc::api::types::EntityId& id);

    int AllocateBuffer(vrtf::vcc::api::types::DriverType type, std::shared_ptr<uint8_t> &data, const uint32_t length);

    std::shared_ptr<vrtf::vcc::driver::MethodHandler> CreateMethodServer(
        const vrtf::vcc::api::types::DriverType type,
        const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& protocolData);

    std::shared_ptr<vrtf::vcc::driver::MethodHandler> CreateMethodClient(
        const vrtf::vcc::api::types::DriverType type,
        const std::shared_ptr<vrtf::vcc::api::types::MethodInfo>& protocolData);
    static std::shared_ptr<DriverManager>& GetInstance();

    /**
     * @brief check the type of driver if is initialized
     *
     * @param type[in]     the type of driver, dds/someip
     * @return true        the type of driver is initialized
     * @return false       the type of driver hasn't been initialized
     */
    bool IsInitialized(const vrtf::vcc::api::types::DriverType& type);
private:

    std::map<vrtf::vcc::api::types::DriverType, std::shared_ptr<vrtf::vcc::driver::Driver>> mapDriver_;
    std::map<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::driver::EventHandler>> mapEventHandler_;
    std::map<vrtf::vcc::api::types::EntityId, std::shared_ptr<vrtf::vcc::driver::MethodHandler>> mapMethodHandler_;
    std::mutex driverMapMutex_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
}
#endif
