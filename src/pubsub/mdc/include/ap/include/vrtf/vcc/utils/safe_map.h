/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: thread safe unordered_map implementation
 * Create: 2019-11-19
 */
#ifndef INC_ARA_VCC_SAFEMAP_H
#define INC_ARA_VCC_SAFEMAP_H
#include <map>
#include "rw_lock.h"
namespace vrtf {
namespace vcc {
namespace utils {
template <class Key, class Value>
class SafeMap {
public:
    void Insert(const Key &key, const Value &value)
    {
        auto raii = tLock.WriteGuard();
        safeMap.emplace(key, value);
    }

    void Erase(const typename std::map<Key, Value>::iterator iter)
    {
        auto raii = tLock.WriteGuard();
        safeMap.erase(iter);
    }

    void Erase(const Key &key)
    {
        auto raii = tLock.WriteGuard();
        safeMap.erase(key);
    }

    size_t Size() const
    {
        auto raii = tLock.ReadGuard();
        return safeMap.size();
    }

    bool Empty() const
    {
        auto raii = tLock.ReadGuard();
        return safeMap.empty();
    }

    void Clear()
    {
        auto raii = tLock.WriteGuard();
        safeMap.clear();
    }

    bool Find(const Key &k, Value &v) const /* risk on ref to items destroyed */
    {
        auto raii = tLock.ReadGuard();
        auto it = safeMap.find(k);
        if (it == safeMap.end()) {
            return false;
        }
        v = it->second;
        return true;
    }

    bool Find(const Key &k) const /* risk on ref to items destroyed */
    {
        auto raii = tLock.ReadGuard();
        auto it = safeMap.find(k);
        if (it == safeMap.end()) {
            return false;
        }
        return true;
    }
    // user add lock to keep iterator is valid
    typename std::map<Key, Value>::iterator Begin()
    {
        auto raii = tLock.ReadGuard();
        return safeMap.begin();
    }

    typename std::map<Key, Value>::iterator End()
    {
        auto raii = tLock.ReadGuard();
        return safeMap.end();
    }

    Value& operator [](const Key& k)
    {
        auto raii = tLock.WriteGuard();
        return safeMap[k];
    }

private:
    std::map<Key, Value> safeMap; /* Fixme about support unordered_map */
    RWLock tLock;    /* thread lock */
};
}
}
}
#endif
