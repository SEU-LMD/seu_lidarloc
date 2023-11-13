/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_usertype.h
 */

#ifndef RT_DDS_DDS_UTILITY_H
#define RT_DDS_DDS_UTILITY_H

#include "RT-DDS/dds_cpp/dds_cpp_common.h"
#include "RT-DDS/dds_cpp/dds_cpp_qos.h"

class DDS_Utility {
public:
    static void Sleep(const DDS_Duration &duration);
};

class DDS_ConfigHelper {
public:
    /**
     * @brief Assert the configuration identified by name in the input policy.
     * @details If the configuration already exists, this function replaces its
     * current value with the new one.
     * If the configuration identified by name does not exist, this function
     * adds it to the property set.
     * @safe UNSAFE. It is not safe to assert the same object DDS_ConfigQos while another thread may be simultaneously
     * calling DDS_ConfigHelper::AssertConfig, DDS_ConfigHelper::LookupConfig.
     * @param qos[in] input qos policy.
     * @param name[in] config name.
     * @param value[in] config value.
     * @return Standard DDS Return Code.
     * @retval DDS_RETCODE_OK
     */
    static DDS_ReturnCode AssertConfig(DDS_ConfigQos &qos, const std::string name, const std::string value);

    /**
     * @brief Searches for the configuration in the input policy given its name.
     * @safe UNSAFE. It is not safe to lookup the same object DDS_ConfigQos while another thread may be simultaneously
     * calling DDS_ConfigHelper::AssertConfig.
     * @param qos[in] input qos policy.
     * @param name[in] config name.
     * @return DDS_Config pointer.
     */
    static DDS_Config *LookupConfig(DDS_ConfigQos &qos, const std::string name);
};

#endif /* RT_DDS_DDS_UTILITY_H */

