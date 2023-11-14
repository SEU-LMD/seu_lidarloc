/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of String class according to AutoSAR standard core type
 * Create: 2019-07-24
 */
#ifndef ARA_CORE_STRING_H
#define ARA_CORE_STRING_H

#include "ara/core/string_view.h"
#include <type_traits>
#include <functional>
#ifdef AOS_TAINT
#ifndef COVERITY_TAINT_SET_DEFINITION
#define COVERITY_TAINT_SET_DEFINITION
/**
 * @brief Function for Stain Modeling
 * @details The function is used only when the compilation macro AOS_TAINT is enabled.
 */
static void Coverity_Tainted_Set(void *buf){}
#endif
#endif
namespace ara {
namespace core {
namespace internal {
template <typename CharT = char, typename Traits = std::char_traits<char>, typename Allocator = std::allocator<char>>
class BasicString : public std::basic_string<CharT, Traits, Allocator> {
public:
    using StdString = std::basic_string<CharT, Traits, Allocator>;
    using StdString::StdString;
    using size_type = typename StdString::size_type;
    using const_iterator = typename StdString::const_iterator;
    using StdString::npos;

    BasicString() noexcept(noexcept(Allocator()))
        : StdString(Allocator())
    {}

    BasicString(StdString const & s)
        : StdString(s)
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&s);
#endif
    }

    BasicString(StdString && s) noexcept
        : StdString(std::move(s))
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&s);
#endif
    }

    BasicString(BasicString const & s)
        : StdString(s)
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&s);
#endif
    }

    BasicString(BasicString && s) noexcept
        : StdString(std::move(s))
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&s);
#endif
    }

    explicit BasicString(StringView<CharT, Traits> sv) // [SWS_CORE_03302]
        : StdString(sv.data(), sv.size())
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
    }

    template <typename T,
        typename = typename std::enable_if<std::is_convertible<T const&,
            StringView<CharT, Traits>>::value>::type>
    BasicString(T const & t, size_type pos, size_type n) // [SWS_CORE_03303]
        : BasicString(StringView<CharT, Traits>(t).substr(pos, n))
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&t);
        Coverity_Tainted_Set((void *)&pos);
        Coverity_Tainted_Set((void *)&n);
#endif
    }

    ~BasicString() = default;

    using StdString::operator=;
    BasicString& operator=(BasicString const & other)
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&other);
#endif
        if (&other != this) {
            assign(other);
        }
        return *this;
    }

    BasicString& operator=(BasicString && other)
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&other);
#endif
        if (&other != this) {
            assign(std::move(other));
        }
        return *this;
    }

    BasicString& operator=(StringView<CharT, Traits> sv) // [SWS_CORE_03304]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
        return assign(sv);
    }

    using StdString::assign;
    BasicString& assign(StringView<CharT, Traits> sv) // [SWS_CORE_03305]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
        StdString::assign(sv.data(), sv.size());
        return *this;
    }

    template <typename T,
        typename = typename std::enable_if<std::is_convertible<T const&,
            StringView<CharT, Traits>>::value>::type>
    BasicString& assign(T const & t, size_type pos, size_type n = npos) // [SWS_CORE_03306]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&t);
        Coverity_Tainted_Set((void *)&pos);
        Coverity_Tainted_Set((void *)&n);
#endif
        StringView<CharT, Traits> const sv = t;
        if (pos >= sv.size()) {
            throw std::out_of_range("position is out of range");
        }
        size_type const rcount = std::min(n, sv.size() - pos);
        StdString::assign(sv.data() + pos, rcount);
        return *this;
    }

    using StdString::append;
    BasicString& append(StringView<CharT, Traits> sv) // [SWS_CORE_03308]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
        StdString::append(sv.data(), sv.size());
        return *this;
    }

    template <typename T,
        typename = typename std::enable_if<std::is_convertible<T const&,
            StringView<CharT, Traits>>::value>::type>
    BasicString& append(T const & t, size_type pos, size_type n = npos) // [SWS_CORE_03309]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&t);
        Coverity_Tainted_Set((void *)&pos);
        Coverity_Tainted_Set((void *)&n);
#endif
        StringView<CharT, Traits> const sv = t;
        if (pos >= sv.size()) {
            throw std::out_of_range("position is out of range");
        }
        size_type const rcount = std::min(n, sv.size() - pos);
        StdString::append(sv.data() + pos, rcount);
        return *this;
    }

    using StdString::operator+=;
    BasicString& operator+=(StringView<CharT, Traits> sv) // [SWS_CORE_03307]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
        return append(sv);
    }

    using StdString::insert;
    BasicString& insert(size_type pos, StringView<CharT, Traits> sv) // [SWS_CORE_03310]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos);
        Coverity_Tainted_Set((void *)&sv);
#endif
        StdString::insert(pos, sv.data(), sv.size());
        return *this;
    }

    template <typename T,
        typename = typename std::enable_if<std::is_convertible<T const&,
            StringView<CharT, Traits>>::value>::type>
    BasicString& insert(size_type pos1, T const & t, size_type pos2, size_type n = npos) // [SWS_CORE_03311]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos1);
        Coverity_Tainted_Set((void *)&pos2);
        Coverity_Tainted_Set((void *)&n);
#endif
        StringView<CharT, Traits> sv = t;
        if (pos1 > StdString::size() || pos2 >= sv.size()) {
            throw std::out_of_range("position is out of range");
        }
        size_type const rcount = std::min(n, sv.size() - pos2);
        StdString::insert(pos1, sv.data() + pos2, rcount);
        return *this;
    }

    using StdString::replace;
    BasicString& replace(size_type pos1, size_type n1, StringView<CharT, Traits> sv) // [SWS_CORE_03312]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos1);
        Coverity_Tainted_Set((void *)&n1);
        Coverity_Tainted_Set((void *)&sv);
#endif
        StdString::replace(pos1, n1, sv.data(), sv.size());
        return *this;
    }

    // [SWS_CORE_03313]
    template <typename T,
        typename = typename std::enable_if<std::is_convertible<T const&,
            StringView<CharT, Traits>>::value>::type>
    BasicString& replace(size_type pos1, size_type n1, T const & t, size_type pos2, size_type n2 = npos)
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos1);
        Coverity_Tainted_Set((void *)&n1);
        Coverity_Tainted_Set((void *)&t);
        Coverity_Tainted_Set((void *)&pos2);
        Coverity_Tainted_Set((void *)&n2);
#endif
        StringView<CharT, Traits> sv = t;
        if (pos1 >= StdString::size() || pos2 >= sv.size()) {
            throw std::out_of_range("position is out of range");
        }
        size_type const rcount = std::min(n2, sv.size() - pos2);
        StdString::replace(pos1, n1, sv.data() + pos2, rcount);
        return *this;
    }

    BasicString& replace(const_iterator i1, const_iterator i2, StringView<CharT, Traits> sv) // [SWS_CORE_03314]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&i1);
        Coverity_Tainted_Set((void *)&i2);
        Coverity_Tainted_Set((void *)&sv);
#endif
        return replace(i1 - StdString::begin(), i2 - i1, sv);
    }

    operator StringView<CharT, Traits>() const noexcept // [SWS_CORE_03301]
    {
        return StringView<CharT, Traits>(StdString::data(), StdString::size());
    }

    using StdString::find;
    using StdString::rfind;
    using StdString::find_first_of;
    using StdString::find_last_of;
    using StdString::find_first_not_of;
    using StdString::find_last_not_of;
    size_type find(StringView<CharT, Traits> sv, size_type pos = 0) const noexcept // [SWS_CORE_03315]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::find(sv.data(), pos, sv.size());
    }

    size_type rfind(StringView<CharT, Traits> sv, size_type pos = npos) const noexcept // [SWS_CORE_03316]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::rfind(sv.data(), pos, sv.size());
    }

    size_type find_first_of(StringView<CharT, Traits> sv, size_type pos = 0) const noexcept // [SWS_CORE_03317]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::find_first_of(sv.data(), pos, sv.size());
    }

    size_type find_last_of(StringView<CharT, Traits> sv, size_type pos = npos) const noexcept // [SWS_CORE_03318]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::find_last_of(sv.data(), pos, sv.size());
    }

    size_type find_first_not_of(StringView<CharT, Traits> sv, size_type pos = 0) const noexcept // [SWS_CORE_03319]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::find_first_not_of(sv.data(), pos, sv.size());
    }

    size_type find_last_not_of(StringView<CharT, Traits> sv, size_type pos = npos) const noexcept // [SWS_CORE_03320]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
        Coverity_Tainted_Set((void *)&pos);
#endif
        return StdString::find_last_not_of(sv.data(), pos, sv.size());
    }

    using StdString::compare;
    int compare(StringView<CharT, Traits> sv) const noexcept // [SWS_CORE_03321]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&sv);
#endif
        size_type stringSize = StdString::size();
        size_type stringViewSize = sv.size();
        size_type const rcount = std::min(stringSize, stringViewSize);
        int const ret = Traits::compare(StdString::data(), sv.data(), rcount);
        if (ret == 0) {
            if (stringSize == stringViewSize) {
                return 0;
            }
            return (stringSize < stringViewSize) ? -1 : 1;
        }
        return ret;
    }

    int compare(size_type pos1, size_type n1, StringView<CharT, Traits> sv) const // [SWS_CORE_03322]
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos1);
        Coverity_Tainted_Set((void *)&n1);
        Coverity_Tainted_Set((void *)&sv);
#endif
        return StringView<CharT, Traits>(StdString::data(), StdString::size()).substr(pos1, n1).compare(sv);
    }

    // [SWS_CORE_03323]
    template <typename T>
    int compare(size_type pos1, size_type n1, T const & t, size_type pos2, size_type n2 = npos) const
    {
#ifdef AOS_TAINT
        Coverity_Tainted_Set((void *)&pos1);
        Coverity_Tainted_Set((void *)&n1);
        Coverity_Tainted_Set((void *)&t);
        Coverity_Tainted_Set((void *)&pos2);
        Coverity_Tainted_Set((void *)&n2);
#endif
        StringView<CharT, Traits> const sv = t;
        return StringView<CharT, Traits>(StdString::data(),
            StdString::size()).substr(pos1, n1).compare(sv.substr(pos2, n2));
    }
};
} // End of internal

template<typename Allocator = std::allocator<char>>
using BasicString = internal::BasicString<char, std::char_traits<char>, Allocator>;

template <typename Allocator>
void swap(BasicString<Allocator>& lhs, BasicString<Allocator>& rhs) // [SWS_CORE_03296]
{
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&lhs);
    Coverity_Tainted_Set((void *)&rhs);
#endif
    lhs.swap(rhs);
}

using String = internal::BasicString<>;
} // End of namespace core
} // End of namespace ara

namespace std {
template <>
struct hash<ara::core::String> {
    size_t operator()(ara::core::String const & s) const noexcept
    {
	return std::hash<std::string>()(s);
    }
};
}

#endif
