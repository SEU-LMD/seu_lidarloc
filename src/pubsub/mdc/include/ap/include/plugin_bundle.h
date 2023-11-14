/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI Plugin Bundle
 *              User should not use the header file directly.
 * Author: c00527270
 * Create: 2020-04-06
 */

#ifndef _PCFI_PLUGIN_BUNDLE_H_
#define _PCFI_PLUGIN_BUNDLE_H_

#include <map>
#include <atomic>
#include <memory>
#include <cstdint>
#include <exception>
#include <dlfcn.h>
#include "pcfi_logger.h"
#include "entity_builder.h"

namespace mdc {
namespace pcfi {
template<typename PluginBaseType>
class PluginBundle {
public:
    PluginBundle(const String &pluginName, const String &pluginPath)
        : m_pluginName(pluginName), m_pluginPath(pluginPath)
    {}
    ~PluginBundle()
    {
        /* Attention: Derived objects must be destroyed before dynamic library unmapping(dlclose) */
        /*            or destruction functions will coredump because cannot find destructors      */
        m_entityMap.clear();
        m_builder.reset(nullptr);

        if (m_dlHandle != nullptr) {
            (void)dlclose(m_dlHandle);
        }
    }

    std::int32_t Init()
    {
        if (m_bInited.load()) {
            PCFI_LOG_WARN << "PluginBundle [" << m_pluginName << "] already inited";
            return PCFI_OK;
        }

        PCFI_LOG_DEBUG << "call dlopen() ->" << m_pluginPath;
        void* const handle = dlopen(m_pluginPath.c_str(), RTLD_LOCAL | RTLD_LAZY);
        PCFI_CHECK_RET(handle == nullptr, PCFI_ERROR, FmtStr("dlopen[%s] failed, dlerr(%s)",
            m_pluginPath.c_str(), dlerror()));
        m_dlHandle = handle;

        // 1. Get NewBuilder function from plugin library.
        using NewBuilderFuncT = EntityBuilder<PluginBaseType> *();
        PCFI_LOG_DEBUG << "call dlsym(" << m_dlHandle << ") to get symbol NewBuilder";
        NewBuilderFuncT *fNewBuilder = reinterpret_cast<NewBuilderFuncT *>(dlsym(m_dlHandle, "NewBuilder"));
        if (fNewBuilder == nullptr) {
            const char_t* const errMsg = dlerror();
            if (errMsg == nullptr) {
                PCFI_LOG_ERROR << "dlsym(" << m_dlHandle << ") failed";
            } else {
                PCFI_LOG_ERROR << "dlsym(" << m_dlHandle << ") failed, dlerror:" << errMsg;
            }
            FreeRes();
            return PCFI_ERROR;
        }
        // 2. Create(New) a plugin builder
        EntityBuilder<PluginBaseType> *ptrEntityBuilder = nullptr;
        try {
            ptrEntityBuilder = fNewBuilder();
        } catch (const std::exception& e) {
            PCFI_LOG_ERROR << "create " << m_pluginName << " NewBuilder() builder failed. exception: " << e.what();
            FreeRes(ptrEntityBuilder);
            return PCFI_ERROR;
        }

        if (ptrEntityBuilder == nullptr) {
            PCFI_LOG_ERROR << "create " << m_pluginName << " NewBuilder() builder failed.";
            FreeRes(ptrEntityBuilder);
            return PCFI_ERROR;
        }
        // 3. manage the plugin builder ptr using std::unique_ptr.
        try {
            m_builder = std::unique_ptr<EntityBuilder<PluginBaseType>>(ptrEntityBuilder);
        } catch (const std::exception& e) {
            PCFI_LOG_ERROR << "create " << m_pluginName << " builder unique_ptr failed. exception: " << e.what();
            FreeRes(ptrEntityBuilder);
            return PCFI_ERROR;
        }

        m_bInited.store(true);
        PCFI_LOG_WARN << "PluginBundle" << m_pluginName << "Init Success.";
        return PCFI_OK;
    }

    std::shared_ptr<PluginBaseType> GetEntity(const String &entityName)
    {
        PCFI_CHECK_RET(!m_bInited.load(), nullptr, FmtStr("PluginBundle[%s] isn't inited", m_pluginName.c_str()));
        const auto it = m_entityMap.find(entityName);
        if (it == m_entityMap.end()) {
            PCFI_LOG_WARN << "Entity[" << entityName << "] not found in Plugin[" << m_pluginName << "]";
            return nullptr;
        }
        return it->second;
    }

    bool IsEntityExist(const String &entityName)
    {
        PCFI_CHECK_RET(!m_bInited.load(), false, FmtStr("PluginBundle[%s] isn't inited", m_pluginName.c_str()));
        const auto it = m_entityMap.find(entityName);
        if (it == m_entityMap.end()) {
            return false;
        }
        return true;
    }

    Vector<String> GetEntityNameList() const
    {
        Vector<String> nameList;
        PCFI_CHECK_RET(!m_bInited.load(), nameList, FmtStr("PluginBundle[%s] isn't inited", m_pluginName.c_str()));
        for (const auto &p : m_entityMap) {
            nameList.emplace_back(p.first);
        }
        return nameList;
    }

    std::int32_t CreateEntity(const String &entityName, const String &confPath)
    {
        PCFI_CHECK_RET(!m_bInited.load(), PCFI_ERROR, FmtStr("PluginBundle[%s] isn't inited", m_pluginName.c_str()));
        if (IsEntityExist(entityName)) {
            PCFI_LOG_ERROR << "Entity Name already exist:" << entityName;
            return PCFI_OK;
        }
        PCFI_LOG_DEBUG << "Begin Create Entity [" << entityName << "], config path:" << confPath;

        PCFI_CHECK_RET((m_builder == nullptr), PCFI_ERROR, "m_builder is nullptr");
        std::shared_ptr<PluginBaseType> entity = m_builder->Build(m_pluginName, entityName, confPath);
        m_entityMap[entityName] = entity;

        PCFI_LOG_INFO << "Entity [" << entityName << "] created with conf [" << confPath << "]";
        return PCFI_OK;
    }
private:
    void FreeRes(const EntityBuilder<PluginBaseType> *ptrEntityBuilder = nullptr)
    {
        if (m_dlHandle != nullptr) {
            (void)dlclose(m_dlHandle);
            m_dlHandle = nullptr;
        }
        if (ptrEntityBuilder != nullptr) {
            delete ptrEntityBuilder;
            ptrEntityBuilder = nullptr;
        }
        return;
    }
private:
    String             m_pluginName;
    String             m_pluginPath;
    void              *m_dlHandle {nullptr};
    std::atomic<bool>  m_bInited {false};
    std::unique_ptr<EntityBuilder<PluginBaseType>>     m_builder;
    std::map<String, std::shared_ptr<PluginBaseType>>  m_entityMap;
};
}
}

#endif
