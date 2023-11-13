/* *
 * FUNCTION: Define HafYamlNode Class
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 *    */
#ifndef ADSF_INTERFACES_YAML_HAFYAML_H
#define ADSF_INTERFACES_YAML_HAFYAML_H

#include "yaml-cpp/yaml.h"
#include <string>
#include <fstream>
#include <iostream>
#include "core/logger.h"
namespace Adsfi {
    using HafYamlIterator = YAML::const_iterator;
    class HafYamlNode {
    public:
        HafYamlNode(const std::string path)
        {
            std::ifstream fin(path);
            if (!fin) {
                std::clog << "Error: File corrupted or not exist.: " << path << std::endl;
            } else {
                fin.close();
            }
            node = YAML::LoadFile(path);
        }

        HafYamlNode() {}

        HafYamlNode(const YAML::Node child)
        {
            this->node = child;
        }
        ~HafYamlNode(){};

        // indexing
        template <typename Key> const HafYamlNode operator[](const Key &keyName) const
        {
            return HafYamlNode(node[keyName]);
        }

        template <typename Key> HafYamlNode operator[](const Key &keyName)
        {
            return HafYamlNode(node[keyName]);
        }

        // access
        template <typename T> T as() const
        {
            return node.as<T>();
        }

        template <typename T> bool GetValue(const std::string &keyName, T &value) const
        {
            if (node[keyName]) {
                value = node[keyName].as<T>();
                return true;
            }
            return false;
        }

        HafYamlIterator begin() const
        {
            return node.begin();
        }

        HafYamlIterator end() const
        {
            return node.end();
        }

        bool IsSequence() const
        {
            return node.IsSequence();
        }

        size_t NodeSize() const
        {
            return node.size();
        }

        template <typename Key, typename Value>void SetNodeValue(const Key &keyName, const Value &value)
        {
            node[keyName] = value;
            return;
        }

        YAML::Node GetNode() const
        {
            return node;
        }

        bool HasKeyValue(const std::string &keyName) const
        {
            return node[keyName];
        }

    private:
        YAML::Node node;
    };
}
#endif
