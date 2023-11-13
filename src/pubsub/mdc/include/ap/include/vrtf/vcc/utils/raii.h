/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: Raii in vcc
 * Create: 2019-11-19
 */
#ifndef INC_ARA_VCC_RAII_H
#define INC_ARA_VCC_RAII_H
#include <type_traits>
#include <functional>
namespace vrtf {
namespace vcc {
namespace utils {
template<typename T>
struct no_const {
    using type = typename std::conditional<std::is_const<T>::value, typename std::remove_const<T>::type, T>::type;
};
/*
 Resource Acquisition Is Initialization
 */
class Raii {
public:
    using fun_type = std::function<int()>;
    Raii (const Raii &) = delete;
    Raii& operator= (const Raii &) = delete;
    Raii (Raii && r) noexcept : release(r.release)
    {
    }
    Raii (const fun_type& relFun, const fun_type& acqFun = [] { return 0;}) noexcept :release(relFun)
    {
        (void)acqFun();
    }
    ~Raii () noexcept
    {
        release();
    }
private:
    fun_type release;
};

template<typename RES, typename REL, typename ACQ>
Raii MakeRaii (RES& res, REL rel, ACQ acq) noexcept
{
    auto pres = std::addressof (const_cast<typename no_const<RES>::type&>(res));
    return Raii(std::bind(rel, pres), std::bind(acq, pres));
}
}
}
}
#endif

