/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: the implementation of StringView class according to AutoSAR standard core type
 * Create: 2019-07-19
 */
#ifndef ARA_CORE_STRING_VIEW_H
#define ARA_CORE_STRING_VIEW_H

#include <string>
#include <limits>
#include <iterator>
#include <iostream>
#include <utility>
#include <type_traits>
#include <stdexcept>
#include <cstddef>

namespace ara {
namespace core {
namespace internal {
template <typename CharT, typename Traits = std::char_traits<CharT>>
class StringView {
public:
    using traits_type = Traits;
    using value_type = CharT;
    using pointer = CharT *;
    using const_pointer = const CharT *;
    using reference = CharT &;
    using const_reference = const CharT &;
    using const_iterator = const CharT *;
    using iterator = const_iterator;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;
    using reverse_iterator = const_reverse_iterator;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    static constexpr size_type npos = size_type(-1);

    constexpr StringView() noexcept
        : dataPtr_(nullptr), strSize_(0)
    {
    }

    constexpr StringView(const_pointer ptr, size_type len)
        : dataPtr_(ptr), strSize_(len)
    {
    }

    constexpr StringView(const_pointer ptr)
        : dataPtr_(ptr), strSize_(ptr == nullptr ? 0 : traits_type::length(ptr))
    {
    }

    constexpr StringView(StringView const &) noexcept = default;

    ~StringView() = default;

    StringView& operator=(StringView const &) noexcept = default;

    constexpr const_iterator begin() const noexcept
    {
        return dataPtr_;
    }
    constexpr const_iterator cbegin() const noexcept
    {
        return dataPtr_;
    }
    constexpr const_iterator end() const noexcept
    {
        return dataPtr_ + strSize_;
    }
    constexpr const_iterator cend() const noexcept
    {
        return dataPtr_ + strSize_;
    }
    constexpr const_reverse_iterator rbegin() const noexcept
    {
        return const_reverse_iterator(end());
    }
    constexpr const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(end());
    }
    constexpr const_reverse_iterator rend() const noexcept
    {
        return const_reverse_iterator(begin());
    }
    constexpr const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(begin());
    }

    constexpr size_type size() const noexcept
    {
        return strSize_;
    }
    constexpr size_type length() const noexcept
    {
        return strSize_;
    }
    constexpr size_type max_size() const noexcept
    {
        return std::numeric_limits<size_type>::max();
    }
    constexpr bool empty() const noexcept
    {
        return strSize_ == 0;
    }

    constexpr const_reference operator[](size_type pos) const
    {
        return dataPtr_[pos];
    }
    constexpr const_reference at(size_type pos) const
    {
        if (pos >= strSize_) {
            throw std::out_of_range("position is out of range");
        }
        return dataPtr_[pos];
    }
    constexpr const_reference front() const
    {
        return dataPtr_[0];
    }
    constexpr const_reference back() const
    {
        return dataPtr_[strSize_ - 1];
    }
    constexpr const_pointer data() const noexcept
    {
        return dataPtr_;
    }

    void remove_prefix(size_type n)
    {
        dataPtr_ += n;
        strSize_ -= n;
    }
    void remove_suffix(size_type n)
    {
        strSize_ -= n;
    }

    void swap(StringView& sv) noexcept
    {
        std::swap(dataPtr_, sv.dataPtr_);
        std::swap(strSize_, sv.strSize_);
    }

    size_type copy(CharT* dest, size_type count, size_type pos = 0) const
    {
        if (dest == nullptr) {
            return 0;
        }
        if (pos > strSize_) {
            throw std::out_of_range("position is out of range");
        }

        size_type const rcount = std::min(count, strSize_ - pos);
        auto begin = dataPtr_ + pos;
        auto end = begin + rcount;
        for (; begin != end;) {
            *dest++ = *begin++;
        }
        return rcount;
    }
    constexpr StringView substr(size_type pos = 0, size_type count = npos) const
    {
        if (pos > strSize_) {
            throw std::out_of_range("position is out of range");
        }

        size_type const rcount = std::min(count, strSize_ - pos);
        return StringView(dataPtr_ + pos, rcount);
    }
    constexpr int compare(StringView v) const noexcept
    {
        int ret = Traits::compare(dataPtr_, v.dataPtr_, std::min(strSize_, v.strSize_));
        if (ret == 0) {
            if (strSize_ == v.strSize_) {
                return 0;
            }
            return (strSize_ < v.strSize_) ? -1 : 1;
        }
        return ret;
    }
    constexpr int compare(size_type pos, size_type count, StringView v) const
    {
        return substr(pos, count).compare(v);
    }
    constexpr int compare(size_type pos1, size_type count1, StringView v, size_type pos2, size_type count2) const
    {
        return substr(pos1, count1).compare(v.substr(pos2, count2));
    }
    constexpr int compare(const_pointer s) const
    {
        return compare(StringView(s));
    }
    constexpr int compare(size_type pos, size_type count, const_pointer s) const
    {
        return substr(pos, count).compare(StringView(s));
    }
    constexpr int compare(size_type pos, size_type count1, const_pointer s, size_type count2) const
    {
        return substr(pos, count1).compare(StringView(s, count2));
    }

    size_type find(StringView v, size_type pos = 0) const noexcept
    {
        return find(v.data(), pos, v.size());
    }
    size_type find(CharT c, size_type pos = 0) const noexcept
    {
        size_type ret = npos;
        if (pos < strSize_) {
            size_type const rcount = strSize_ - pos;
            const_pointer ptr = traits_type::find(dataPtr_ + pos, rcount, c);
            if (ptr) {
                ret = ptr - dataPtr_;
            }
        }
        return ret;
    }
    size_type find(const_pointer s, size_type pos, size_type count) const
    {
        if (s == nullptr || traits_type::length(s) == 0) {
            std::cout << "[CORETYPE string_view], Error, invalid object pointer." << std::endl;
            return npos;
        }
        size_type rcount = std::min(count, traits_type::length(s));
        if (rcount == 0)
            return pos <= strSize_ ? pos : npos;
        if (rcount > strSize_) {
            return npos;
        }
        for (; pos <= strSize_ - rcount; ++pos) {
            if (traits_type::compare(dataPtr_ + pos, s, rcount) == 0) {
                return pos;
            }
        }
        return npos;
    }
    size_type find(const_pointer s, size_type pos = 0) const
    {
        return find(s, pos, traits_type::length(s));
    }

    size_type rfind(StringView v, size_type pos = npos) const noexcept
    {
        return rfind(v.data(), pos, v.size());
    }
    size_type rfind(CharT c, size_type pos = npos) const noexcept
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).rfind(c, pos);
    }
    size_type rfind(const_pointer s, size_type pos, size_type count) const
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).rfind(s, pos, count);
    }
    size_type rfind(const_pointer s, size_type pos = npos) const
    {
        return rfind(s, pos, Traits::length(s));
    }

    size_type find_first_of(StringView v, size_type pos = 0) const noexcept
    {
        return find_first_of(v.data(), pos, v.size());
    }
    size_type find_first_of(CharT c, size_type pos = 0) const noexcept
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_first_of(c, pos);
    }
    size_type find_first_of(const_pointer s, size_type pos, size_type count) const
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_first_of(s, pos, count);
    }
    size_type find_first_of(const_pointer s, size_type pos = 0) const
    {
        return find_first_of(s, pos, Traits::length(s));
    }

    size_type find_last_of(StringView v, size_type pos = npos) const noexcept
    {
        return find_last_of(v.data(), pos, v.size());
    }
    size_type find_last_of(CharT c, size_type pos = npos) const noexcept
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_last_of(c, pos);
    }
    size_type find_last_of(const_pointer s, size_type pos, size_type count) const
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_last_of(s, pos, count);
    }
    size_type find_last_of(const_pointer s, size_type pos = npos) const
    {
        return find_last_of(s, pos, Traits::length(s));
    }

    size_type find_first_not_of(StringView v, size_type pos = 0) const noexcept
    {
        return find_first_not_of(v.data(), pos, v.size());
    }
    size_type find_first_not_of(CharT c, size_type pos = 0) const noexcept
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_first_not_of(c, pos);
    }
    size_type find_first_not_of(const_pointer s, size_type pos, size_type count) const
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_first_not_of(s, pos, count);
    }
    size_type find_first_not_of(const_pointer s, size_type pos = 0) const
    {
        return find_first_not_of(s, pos, Traits::length(s));
    }

    size_type find_last_not_of(StringView v, size_type pos = npos) const noexcept
    {
        return find_last_not_of(v.data(), pos, v.size());
    }
    size_type find_last_not_of(CharT c, size_type pos = npos) const noexcept
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_last_not_of(c, pos);
    }
    size_type find_last_not_of(const_pointer s, size_type pos, size_type count) const
    {
        return std::basic_string<CharT, Traits>(dataPtr_, strSize_).find_last_not_of(s, pos, count);
    }
    size_type find_last_not_of(const_pointer s, size_type pos = npos) const
    {
        return find_last_not_of(s, pos, Traits::length(s));
    }

private:
    const CharT *dataPtr_;
    size_type strSize_;
};

template <typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os,
    StringView<CharT, Traits> const & v)
{
    os << std::basic_string<CharT, Traits>(v.data(), v.size());
    return os;
}

template <typename T>
using Identity = typename std::decay<T>::type;

template <typename CharT, typename Traits>
constexpr bool operator==(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) == 0;
}
template <typename CharT, typename Traits>
constexpr bool operator==(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) == 0;
}
template <typename CharT, typename Traits>
constexpr bool operator==(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) == 0;
}

template <typename CharT, typename Traits>
constexpr bool operator!=(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) != 0;
}
template <typename CharT, typename Traits>
constexpr bool operator!=(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) != 0;
}
template <typename CharT, typename Traits>
constexpr bool operator!=(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) != 0;
}

template <typename CharT, typename Traits>
constexpr bool operator<(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) < 0;
}
template <typename CharT, typename Traits>
constexpr bool operator<(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) < 0;
}
template <typename CharT, typename Traits>
constexpr bool operator<(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) < 0;
}

template <typename CharT, typename Traits>
constexpr bool operator>(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) > 0;
}
template <typename CharT, typename Traits>
constexpr bool operator>(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) > 0;
}
template <typename CharT, typename Traits>
constexpr bool operator>(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) > 0;
}

template <typename CharT, typename Traits>
constexpr bool operator<=(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) <= 0;
}
template <typename CharT, typename Traits>
constexpr bool operator<=(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) <= 0;
}
template <typename CharT, typename Traits>
constexpr bool operator<=(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) <= 0;
}

template <typename CharT, typename Traits>
constexpr bool operator>=(StringView<CharT, Traits> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) >= 0;
}
template <typename CharT, typename Traits>
constexpr bool operator>=(StringView<CharT, Traits> x, Identity<StringView<CharT, Traits>> y) noexcept
{
    return x.compare(y) >= 0;
}
template <typename CharT, typename Traits>
constexpr bool operator>=(Identity<StringView<CharT, Traits>> x, StringView<CharT, Traits> y) noexcept
{
    return x.compare(y) >= 0;
}
} // End of namespace internal

using StringView = internal::StringView<char>;
} // End of namespace core
} // End of namespace ara

namespace std {
template <>
struct hash<ara::core::StringView> {
    size_t operator()(ara::core::StringView const & sv) const noexcept
    {
        size_t hashVal = 0;
        hashVal = std::hash<std::string> {}(std::string(sv.data(), sv.length()));
        return hashVal;
    }
};
}
#endif
