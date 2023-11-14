/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description: Document value object class header
 * Create: 2019-6-25
 */

#ifndef ARA_GODEL_COMMON_JSONPARSER_DOCUMENT_H_
#define ARA_GODEL_COMMON_JSONPARSER_DOCUMENT_H_

#include "global.h"
#include <vector>
#include <string>
#include <map>
#include <cstdint>
#include <climits>
#include <memory>

namespace ara        {
namespace godel      {
namespace common     {
namespace jsonParser {
class Document {
public:
    Document();
    Document(std::nullptr_t doc);
    ~Document();
    JsonParseValue Parse(std::string const &path);
    JsonType GetType() const;
    bool HasParseError() const;
    Document operator[](int32_t index) const;
    Document operator[](std::string const &key) const;
    bool HasMember(std::string const &key) const;
    bool IsBool() const;
    bool GetBool() const;
    bool IsNull() const;
    bool IsNumber() const;
    double GetNumber() const;
    bool IsString() const;
    std::string GetString() const;
    bool IsArray() const;
    std::vector<Document> GetArray() const;
    bool IsObject() const;
    std::map<std::string, Document> GetObject() const;
    bool IsUint() const;
    unsigned GetUint() const;
    bool IsInt() const;
    int32_t GetInt() const;
    bool IsUint64() const;
    uint64_t GetUint64() const;
    bool IsInt64() const;
    int64_t GetInt64() const;
    bool IsDouble() const;
    double GetDouble() const;
    static std::string ParseFileToString(std::string const &path);
    JsonParseValue ParseStringToDocument(std::string const &jsonStr);
    void SetType(JsonType const &type);
    void SetNumber(double const &num);
    void SetString(std::string const &str);
    void AddVectorElement(Document const &doc);
    void MapEmplace(Document& doc);

private:
    double m_num = 0.0;
    std::string m_str = "";
    std::vector<Document> m_vec {};
    std::map<std::string, Document> m_map {};
    std::shared_ptr<Document> m_doc = nullptr;
    JsonType m_type = JsonType::JSON_UNDEFINED;
};
} // namespace jsonParser
} // namespace common
} // namespace godel
} // namespace ara
#endif // ARA_GODEL_COMMON_JSONPARSER_DOCUMENT_H_
