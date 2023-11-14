/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2021. All rights reserved.
 * Description: RtfConfigParser definition
 * Create: 2021-2-25
 */
#ifndef ARA_RTF_CONFIG_PARSER_H
#define ARA_RTF_CONFIG_PARSER_H

#include <string>

namespace ara {
namespace godel {
namespace common {
namespace rtfConfigParser {
class RtfConfigParser {
public:
    RtfConfigParser();
    explicit RtfConfigParser(std::string const &configPath);
    virtual ~RtfConfigParser() = default;
    std::string GetRtfConfigAddress() const;
    // module could be EM/PHM/TOOLS
    std::string GetRtfConfigInstanceId(std::string const &module) const;
private:
    struct PHMConfig {
        std::string phm_notifier_instance;
    };
    struct ToolsConfig {
        std::string tools_instance;
    };
    struct EMConfig {
        std::string em_instance;
    };

    void ParseRtfConfig();
    std::string Nic2Addr(std::string const &nicStr) const;
    bool IsValidNumberFormat(std::string const &str);

    std::string configPath_;
    std::string configAddress_;
    EMConfig emConfig_ {""};
    PHMConfig phmConfig_ {""};
    ToolsConfig toolsConfig_ {""};
};
} // namespace rtfConfigParser
} // namespace common
} // namespace rtf
} // namespace ara
#endif
