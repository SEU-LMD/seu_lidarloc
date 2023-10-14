/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: This class provides hashing capability of the Ros-like interface.
 * Create: 2020-04-20
 */

#ifndef RTF_COM_UTILS_ROS_HASHER_H_
#define RTF_COM_UTILS_ROS_HASHER_H_

#include <cstdint>
#include <string>

#include "rtf/com/types/ros_types.h"

namespace rtf {
namespace com {
namespace utils {
class RosHasher {
public:
    static size_t Hash(const std::string& entityName) noexcept;
    static EntityId HashEntityId(const std::string& entityName) noexcept;
private:
    RosHasher(void) = delete;
    ~RosHasher(void) = delete;
};
} // namespace utils
} // namespace com
} // namespace rtf
#endif // RTF_COM_UTILS_ROS_HASHER_H_
