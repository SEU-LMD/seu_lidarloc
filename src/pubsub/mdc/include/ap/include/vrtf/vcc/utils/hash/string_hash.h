/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: define gcc string hash function
 * Create: 2021-05-22
 */
#ifndef VRTF_VCC_UTILS_HASH_STRING_HASH
#define VRTF_VCC_UTILS_HASH_STRING_HASH
#include <string>
#include <unistd.h>
namespace vrtf {
namespace vcc {
namespace utils {
namespace hash {
class StringHash {
public:
    static std::size_t Gcc64StrHash(std::string const &str);
};
}
}
}
}
#endif
