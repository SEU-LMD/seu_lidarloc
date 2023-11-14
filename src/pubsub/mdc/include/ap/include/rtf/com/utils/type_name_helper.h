/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The template to check if the function GetTypeName() in data definiation is exist
 * Create: 2020-07-31
 */
#ifndef RTF_COM_UTILS_DATA_TYPE_HELPER_H_
#define RTF_COM_UTILS_DATA_TYPE_HELPER_H_

#include <string>
#include <type_traits>
#include <typeinfo>
#include <cxxabi.h>
namespace ros {
namespace message_traits {
    template<class T>
    struct DataType;

    template<class T>
    struct IsMessage;
}
}
namespace rtf      {
namespace com      {
namespace utils    {
namespace internal {
template<typename T>
struct has_get_type_name {
// Define a helper to detect function
template<typename U, std::string(*)()>
struct Helper;

// Try to match template Helper to detect expected function
// The function is just used for providing return type
// The reason why the function parameter is 'Helper*' is
// we need an expression to satisfy decltype(...)
template<typename U>
static std::true_type HasFunction(Helper<U, &U::GetTypeName>*);

// This matches all cases, which is the fallback
// The function is just used for providing return type
template<typename U>
static std::false_type HasFunction(...);

// Calucate the return type of HasFunction by decltype(...)
// and set the type of value
static const decltype(HasFunction<T>(nullptr)) value;
};
} // namespace internal

template<typename T>
struct HasFunctionValue {
public:
    template<typename U>
    static auto Check(int) -> decltype(ros::message_traits::IsMessage<U>::value, std::true_type());
    template<typename U>
    static std::false_type Check(...);
    static const bool value = std::is_same<decltype(Check<T>(0)), std::true_type>::value;
};


template<typename T>
typename std::enable_if<
    HasFunctionValue<typename std::decay<T>::type>::value && ros::message_traits::IsMessage<T>::value,
    std::string>::type GetTypeNameByRos(void)
{
    std::string name(ros::message_traits::DataType<typename std::decay<T>::type>::value());
    std::string headSymbol("/");
    return headSymbol + name;
}

template<typename T>
typename std::enable_if<
    HasFunctionValue<typename std::decay<T>::type>::value && !ros::message_traits::IsMessage<T>::value,
    std::string>::type GetTypeNameByRos(void)
{
    return "unknow";
}

template<typename T>
constexpr typename std::enable_if<
    !HasFunctionValue<typename std::decay<T>::type>::value, std::string>::type GetTypeNameByRos(void)
{
    return "unknow";
}

template<typename T>
constexpr typename std::enable_if<internal::has_get_type_name<T>::value, std::string>::type GetTypeName(void)
{
    return T::GetTypeName();
}

template<typename T>
typename std::enable_if<!internal::has_get_type_name<T>::value, std::string>::type GetTypeName(void)
{
    auto demangledName = abi::__cxa_demangle(typeid(T).name(), 0, 0, nullptr);
    std::string result(demangledName);
    free(demangledName);
    return result;
}
} // namespace utils
} // namespace com
} // namespace rtf

#endif
