/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: This provide the config to support rtfcom someip serialze config
 * Create: 2021-05-05
 */

#ifndef RTF_COM_CONFIG_SOMEIP_SERIALIZE_BASE_CONFIG_H_
#define RTF_COM_CONFIG_SOMEIP_SERIALIZE_BASE_CONFIG_H_

#include <unordered_map>

#include "rtf/com/types/ros_types.h"
#include "rtf/com/config/interface/config_info_interface.h"
#include "rtf/com/utils/logger.h"
namespace rtf {
namespace com {
namespace config {
class SOMEIPSerializeBaseConfig {
public:
    SOMEIPSerializeBaseConfig()
    {
        logger_ = rtf::com::utils::Logger::GetInstance();
    }
    /**
     * @brief Set impleLegacyStringSerialization
     * @param[in] isLegacy impleLegacyStringSerialization serialize config
     */
    inline void SetImplementsLegacyStringSerialization(const bool isLegacy)
    {
        isImplementsLegacyStringSerialization_ = isLegacy;
    }

    /**
     * @brief Return impleLegacyStringSerialization
     * @return impleLegacyStringSerialization serialize config
     */
    inline bool GetImplementsLegacyStringSerialization(void) const noexcept
    {
        return isImplementsLegacyStringSerialization_;
    }

    /**
     * @brief Set serialize byte order
     * @param[in] byteOrder serialize byte order config
     */
    inline void SetByteOrder(rtf::com::someip::serialize::ByteOrder const byteOrder)
    {
        using namespace rtf::com::utils;
        if ((byteOrder < rtf::com::someip::serialize::ByteOrder::BIGENDIAN) ||
            (byteOrder > rtf::com::someip::serialize::ByteOrder::LITTLEENDIAN)) {
            /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1: Logger records the log */
            logger_->Warn() << "Byte order is invalid, set byte order failed";
            return;
        }
        byteOrder_ = byteOrder;
    }

    /**
     * @brief Return byte order config
     * @return byte order config
     */
    inline rtf::com::someip::serialize::ByteOrder GetByteOrder() const noexcept
    {
        return byteOrder_;
    }

    /**
     * @brief Return serialize config is tlv config
     * @return serialize config is tlv config
     */
    inline bool GetSerializeTypeIsTLV() const noexcept
    {
        return isTlvSerialize_;
    }
protected:
    SOMEIPSerializeBaseConfig(const bool isTlvSerialize)
        : isImplementsLegacyStringSerialization_(false), isTlvSerialize_(isTlvSerialize),
          byteOrder_(someip::serialize::ByteOrder::BIGENDIAN)
    {
        logger_ = rtf::com::utils::Logger::GetInstance();
    }
    virtual ~SOMEIPSerializeBaseConfig(void) = default;
private:
    bool isImplementsLegacyStringSerialization_;
    bool isTlvSerialize_;
    someip::serialize::ByteOrder byteOrder_;
    std::shared_ptr<rtf::com::utils::Logger> logger_;
};
} // namespace config
} // namespace com
} // namespace rtf
#endif // RTF_COM_CONFIG_SOMEIP_SERIALIZE_BASE_CONFIG_H_
