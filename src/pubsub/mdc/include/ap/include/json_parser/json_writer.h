/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: for generate json struct into a string
 * Create: 2019-6-25
 */

#ifndef JSONWRITER_H_
#define JSONWRITER_H_
#include<iostream>
#include<string>
#include<stack>
#include<cstdint>
#include<map>
namespace ara {
namespace godel {
namespace common {
namespace jsonParser {
enum class JsonContainerType: uint8_t {
    DEFAULT_TYPE = 0,
    OBJECT_TYPE,
    ARRAY_TYPE
};

class JsonWriter {
public:
    JsonWriter() = default;
    ~JsonWriter() = default;
    std::string GetString() const noexcept;
    void StartWriteJson(JsonContainerType) noexcept;
    void WriteKeyAndValue(std::string const &key, std::string const &value) noexcept;
    void WriteKey(std::string const &key) noexcept;
    void WriteValue(std::string const &value) noexcept;
    void WriteValue(bool value) noexcept;
    void WriteValue(int64_t value) noexcept;
    void WriteValue(uint64_t value) noexcept;
    void WriteValue(double value) noexcept;
    void WriteRawData(std::string const &value) noexcept;
    void EndArray() noexcept;
    void EndObject() noexcept;
private:
void EndContainer() noexcept;
void WriteNumber(std::string const &number) noexcept;
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : " \"" means left pad of string
std::string const LEFT_PAD {" \""};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : "\"" means right pad of string
std::string const RIGHT_PAD {"\""};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : " {" means begin pad of object
std::string const OBJECT_PAD {" {"};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 :  "} " means end pad of object
std::string const OBJECTEND_PAD {"} "};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : " [" means begin pad of array
std::string const ARRAY_PAD {" ["};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : "] " means end pad of array
std::string const ARRAREND_PAD {"] "};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : ":" means colon pad
std::string const COLON_PAD {":"};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : "," means comma pad
std::string const COMMA_PAD {","};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : "true" means true string
std::string const TRUE_STRING {"true"};
// AXIVION MAGIC AutosarC++19_03-A5.1.1 : "false" means false string
std::string const FALSE_STRING {"false"};

std::string jsonString_;
uint8_t depth {0};
std::map<uint8_t, bool> commaPadFlagMap_;
std::stack<JsonContainerType> jsonTypeContainer;
};
}
}
}
}
#endif
