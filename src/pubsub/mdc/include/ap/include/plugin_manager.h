/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: PCFI Plugin Manager
 * Author: c00527270
 * Create: 2020-04-06
 */

#ifndef _PCFI_PLUGIN_MANAGER_H_
#define _PCFI_PLUGIN_MANAGER_H_

#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include "pcfi_base.h"
#include "pcfi_logger.h"
#include "plugin_bundle.h"
#include "pcfi_topo_parse.h"

namespace mdc {
namespace pcfi {
template<typename PluginBaseType>
class PluginManager {
public:
    static PluginManager &GetInstance()
    {
        static PluginManager mgr;
        return mgr;
    }

    /*
     * 1. PluginManager::Init will do some logging, So The Caller MUST
     * call ara::log::InitLogging before call PluginManager::Init
     * 2. The Caller MUST ensure the safety of input configPath.
     */
    std::int32_t Init(const String& configPath, const String& libPrefix = "", const String& confPrefix = "")
    {
        return AddPlugin(configPath, libPrefix, confPrefix);
    }

    std::int32_t AddPlugin(const String& configPath, const String& libPrefix = "", const String& confPrefix = "")
    {
        ConfigParam cfgParam(configPath, libPrefix, confPrefix);
        Vector<PluginItem> plugins;
        auto ret = ParsePluginTopoFile(cfgParam, plugins);
        PCFI_CHECK_RET(ret != PCFI_OK, ret, FmtStr("topo file(%s) parse failed", cfgParam.configPath.c_str()));

        ret = BuildAll(plugins);
        PCFI_CHECK_RET(ret != PCFI_OK, ret, "build all plugin entities NOT fully success");
        return PCFI_OK;
    }
    /*
     * 1. PluginManager::Init will do some logging, So The Caller MUST
     * call ara::log::InitLogging before call PluginManager::Init
     */
    std::int32_t Init(const Vector<PluginItem> &plugins)
    {
        const std::int32_t ret = BuildAll(plugins);
        PCFI_CHECK_RET(ret != PCFI_OK, ret, "build plugin entities failed");
        return PCFI_OK;
    }

    Vector<String> GetPluginNameList()
    {
        std::shared_lock<std::shared_timed_mutex> lock(m_pluginBundleMutex);
        Vector<String> nameList;
        for (const auto &p : m_pluginBundls) {
            nameList.emplace_back(p.first);
        }
        return nameList;
    }

    Vector<String> GetEntityNameList(const String &pluginName)
    {
        std::shared_ptr<PluginBundle<PluginBaseType>> plugin = GetPlugin(pluginName);
        if (plugin == nullptr) {
            PCFI_LOG_ERROR << "Plugin Name not registered: " << pluginName;
            return Vector<String>();
        }
        return plugin->GetEntityNameList();
    }

    std::shared_ptr<PluginBaseType> GetEntity(const String &pluginName, const String &entityName)
    {
        std::shared_ptr<PluginBundle<PluginBaseType>> plugin = GetPlugin(pluginName);
        if (plugin == nullptr) {
            PCFI_LOG_ERROR << "Plugin Name not registered: " << pluginName;
            return nullptr;
        }

        return plugin->GetEntity(entityName);
    }

    std::shared_ptr<PluginBaseType> GetEntity(const String &entityName)
    {
        const auto pluginNameList = GetPluginNameList();
        for (const auto& pluginName : pluginNameList) {
            std::shared_ptr<PluginBaseType> pluginEntity = GetEntity(pluginName, entityName);
            if (pluginEntity != nullptr) {
                PCFI_LOG_INFO << "get plugin succeed, plugin name:" << pluginName << ", entity name:" << entityName;
                return pluginEntity;
            }
        }
        PCFI_LOG_ERROR << "Entity Name not find" << entityName;
        return nullptr;
    }

    ~PluginManager() = default;

private:
    PluginManager() {
        CreateLogger("PCFI", "PCFI");
    }

    static void CreateLogger(const String ctxId, const String ctxDescription)
    {
        PcfiLogger::GetInstance().CreateLogger(ctxId, ctxDescription);
        return;
    }

    std::shared_ptr<PluginBundle<PluginBaseType>> GetPlugin(const String &pluginName)
    {
        std::shared_lock<std::shared_timed_mutex> lock(m_pluginBundleMutex);
        const auto it = m_pluginBundls.find(pluginName);
        if (it == m_pluginBundls.end()) {
            return nullptr;
        }
        return it->second;
    }

    std::uint32_t RegisterPlugin(const String &pluginName, const String &pluginPath)
    {
        if (GetPlugin(pluginName) != nullptr) {
            PCFI_LOG_WARN << "Plugin Name already exist: " << pluginName;
            return PCFI_OK;
        }

        std::shared_ptr<PluginBundle<PluginBaseType>> plugin =
            PcfiCreateObject<PluginBundle<PluginBaseType>>(pluginName, pluginPath);
        PCFI_CHECK_RET(!plugin, PCFI_ERROR, FmtStr("Create PluginBundle failed. pluginName=%s", pluginName.c_str()));

        const std::int32_t ret = plugin->Init();
        PCFI_CHECK_RET(ret != PCFI_OK, static_cast<std::uint32_t>(ret),
            FmtStr("Init plugin bundle[%s] with conf[%s] failed", pluginName.c_str(), pluginPath.c_str()));

        std::unique_lock<std::shared_timed_mutex> lock(m_pluginBundleMutex);
        m_pluginBundls[pluginName] = plugin;
        PCFI_LOG_INFO << "Plugin [" << pluginName << "] registered.";
        return PCFI_OK;
    }

    std::int32_t CreateEntity(const PluginItem &plugin)
    {
        PCFI_LOG_DEBUG << "Enter CreateEntity, Type =" << plugin.pluginName;
        std::shared_ptr<PluginBundle<PluginBaseType>> pluginIns = GetPlugin(plugin.pluginName);
        if (pluginIns == nullptr) {
            PCFI_LOG_ERROR << "Plugin Name not registered: " << plugin.pluginName;
            return PCFI_ERROR;
        }

        for (const EntityItem &entity : plugin.entities) {
            const auto ret = pluginIns->CreateEntity(entity.entityName, entity.confPath);
            PCFI_CHECK_RET(ret != PCFI_OK, ret, FmtStr("Create Plugin(%s)-Entity(%s)[%s] failed",
                plugin.pluginName.c_str(), entity.entityName.c_str(), entity.confPath.c_str()));
        }
        return PCFI_OK;
    }

    std::int32_t BuildAll(const Vector<PluginItem> &plugins)
    {
        std::lock_guard<std::mutex> lock(m_opMutex); // 添加操作互斥,保证每次的添加的插件初始化完
        std::int32_t buildResult = PCFI_OK;
        for (const PluginItem &plugin : plugins) {
            if (RegisterPlugin(plugin.pluginName, plugin.pluginPath) != PCFI_OK) {
                buildResult = PCFI_ERROR;
                continue;
            }

            if (CreateEntity(plugin) != PCFI_OK) {
                buildResult = PCFI_ERROR;
                continue;
            }
        }
        return buildResult;
    }

private:
    std::map<String, std::shared_ptr<PluginBundle<PluginBaseType>>> m_pluginBundls;
    std::shared_timed_mutex m_pluginBundleMutex;
    std::mutex              m_opMutex;
};
}
}

#endif
