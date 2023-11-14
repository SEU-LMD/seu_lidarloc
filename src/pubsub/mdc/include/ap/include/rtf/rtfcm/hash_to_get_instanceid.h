/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Get InstanceId from hash local IP.
 * Create: 2020-02-07
 */
#ifndef RTF_HASH_TO_GET_INSTANCEID_H
#define RTF_HASH_TO_GET_INSTANCEID_H
#include "vrtf/vcc/api/param_struct_typse.h"
#include "ara/hwcommon/log/log.h"
namespace rtf {
namespace rtfcm {
namespace rtfmaintaind {
const static vrtf::vcc::api::types::InstanceId DEFAULT_LOCAL_INSTANCEID = "65534";
// In maintaind service 65534 is reserve, which will use if hash insatnce id fail.
constexpr static uint16_t MAX_UINT16_VALUE = 65534;
class HashToGetInstanceId {
public:
    HashToGetInstanceId()
    {
        using namespace ara::godel::common;
        logInstance_ = log::Log::GetLog("CM");
    }
    ~HashToGetInstanceId() = default;
    static vrtf::vcc::api::types::InstanceId GetHashInstanceId();
    static uint32_t HashStringU32(const std::string& str);
private:
    static bool GetLocalIp(std::string& ip);
    static std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
#endif
