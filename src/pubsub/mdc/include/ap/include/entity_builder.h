/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI Entity Builder
 * Author: c00527270
 * Create: 2020-04-06
 */

#ifndef _PCFI_ENTITY_BUILDER_H_
#define _PCFI_ENTITY_BUILDER_H_

#include <memory>
#include <string>

#define PCFI_SETBUILDER(x) \
    extern "C" void* NewBuilder() { \
        return new (x)(); \
    }

namespace mdc {
namespace pcfi {
template<typename PluginBaseType>
class EntityBuilder {
public:
    virtual ~EntityBuilder() {}
    EntityBuilder& operator=(EntityBuilder const& rhs) = delete;
    virtual std::shared_ptr<PluginBaseType> Build(const std::string &pluginName, const std::string &entityName,
        const std::string &confPath) = 0;
};
}
}

#endif