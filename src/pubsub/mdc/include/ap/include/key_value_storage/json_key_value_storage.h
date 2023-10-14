/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: JsonKeyValueStorage class header
 *              This class provides json key value storage related functions
 * Create: 2019-06-25
 */

#ifndef ARA_GODEL_COMMON_KVS_JSON_KEY_VALUE_STORAGE_H_
#define ARA_GODEL_COMMON_KVS_JSON_KEY_VALUE_STORAGE_H_

#include "key_value_storage.h"

namespace ara    {
namespace godel  {
namespace common {
namespace kvs    {
class JsonKeyValueStorage {
public:
    static bool LoadFromJson(std::string const &path, KeyValueStorage& kvsObj);

private:
};
} // namespace kvs
} // namespace common
} // namespace godel
} // namespace ara
#endif // ARA_GODEL_COMMON_KVS_JSON_KEY_VALUE_STORAGE_H_ */
