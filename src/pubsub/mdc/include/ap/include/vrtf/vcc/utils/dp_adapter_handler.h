/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Implement dp adapter operations
 * Create: 2020-06-10
 */
#ifndef VRTF_DPADAPTERHANDLER_H
#define VRTF_DPADAPTERHANDLER_H
#include <memory>
#include <mutex>
#include "vrtf/driver/dds/mbuf.h"
#include "ara/hwcommon/log/log.h"

namespace vrtf {
namespace vcc {
namespace utils {
const std::size_t MBUF_PRIVATE_DATA_SIZE = (256);
const std::size_t MAX_E2E_HEADER_SIZE = (20);
/* Now use 4 types for record data size */
const std::size_t CM_RESERVE_SIZE = (24);
class DpAdapterHandler {
public:
    DpAdapterHandler()
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
        availableSize_ =
            (MBUF_PRIVATE_DATA_SIZE - DATAPLAIN_MBUF_PRIV_DDS_REGION_LENGTH - MAX_E2E_HEADER_SIZE - CM_RESERVE_SIZE);
    }
    ~DpAdapterHandler();
    static std::shared_ptr<DpAdapterHandler> GetInstance();
    int32_t MbufGetPrivInfo(Mbuf *mbuf,  void **priv, uint32_t *size);
    int32_t MbufFree(Mbuf *mbuf);
    std::size_t GetAvailabeLenth() const;
private:
    static const DPAdapterOps *dpAdapterOps_;
    static std::shared_ptr<DpAdapterHandler> instance_;
    static std::mutex mutex_;
    static std::shared_ptr<ara::godel::common::log::Log> logInstance_;
    std::size_t availableSize_ = 0;
};
}
}
}
#endif // VRTF_DPADAPTERHANDLER_H
