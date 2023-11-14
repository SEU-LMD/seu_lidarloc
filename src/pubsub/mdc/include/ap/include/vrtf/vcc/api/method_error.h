/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Define types in communication mannger
 * Create: 2019-07-24
*/
#ifndef VRTF_VCC_API_INTERNAL_METHOD_ERROR_H
#define VRTF_VCC_API_INTERNAL_METHOD_ERROR_H
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
struct MethodError {
    uint64_t domainValue = 0;
    int32_t errorCode = 0;
    static bool IsPlane()
    {
        return true;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(domainValue);
        fun(errorCode);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(domainValue);
        fun(errorCode);
    }

    bool operator == (const MethodError& t) const
    {
        return (domainValue == t.domainValue) && (errorCode == t.errorCode);
    }
};
}
}
}
}
#endif

