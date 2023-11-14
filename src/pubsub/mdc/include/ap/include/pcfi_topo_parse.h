/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI Topo parse
 *              User should not use the header file directly.
 * Author: c00527270
 * Create: 2020-04-16
 */

#ifndef _PCFI_TOPO_PARSE_H_
#define _PCFI_TOPO_PARSE_H_

#include "pcfi_base.h"

namespace mdc {
namespace pcfi {
struct EntityItem {
    String entityName;
    String confPath;
};

struct PluginItem {
    String pluginName;
    String pluginPath;
    Vector<EntityItem> entities;
};

struct ConfigParam {
    ConfigParam(const String& configPath, const String& libPrefix, const String& confPrefix)
        : configPath(configPath), libPrefix(libPrefix), confPrefix(confPrefix)
    {}
    String configPath {};
    String libPrefix {};
    String confPrefix {};
};

std::int32_t CanonicalPath(const String &filePath, String &newPath);
void StrReplace(String& str, const String& strOld, const String& strNew);
std::int32_t ParsePluginTopoFile(const ConfigParam &cfgParam, Vector<PluginItem> &plugins);
}
}
#endif