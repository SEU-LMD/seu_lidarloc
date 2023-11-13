/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: 持久化模块的错误类型
 * Author: c00507212
 * Create: 2019-12-10
 * Notes: 无
 */

#ifndef ARA_PER_ERROR_DOMAIN_H_
#define ARA_PER_ERROR_DOMAIN_H_

#include "ara/core/error_domain.h"
#include "ara/core/error_code.h"
#include "ara/core/exception.h"
#include "ara/per/per_base_type.h"
namespace ara {
namespace per {
enum class PerErrc : ara::core::ErrorDomain::CodeType {
    // Requested storage location is not found or not configured in the AUTOSAR model.
    kStorageLocationNotFoundError = 1,
    // The key was not found.
    kKeyNotFoundError = 2,
    // Opening the resource for writing failed because it is configured read-only.
    kIllegalWriteAccessError = 3,
    // A severe error which might happen during the operation,
    // such as out of memory or writing/reading to the storage return an error.
    kPhysicalStorageError = 4,
    // The integrity of the storage could not be established.
    // This can happen when the structure of a key value database is corrupted, or a read-only file has no content.
    kIntegrityError = 5,
    // The validation of redundancy measures failed for a single key, for the whole key value data base, or for a file.
    kValidationError = 6,
    // The encryption or decryption failed for a single key, for the whole key value data base, or for a file.
    kEncryptionError = 7,
    // The provided data type does not match the stored data type.
    kDataTypeMismatchError = 8,
    // The operation could not be performed because no initial value is available.
    kInitValueNotAvailableError = 9,
    // The operation could not be performed because the resource is currently busy.
    kResourceBusyError = 10,
    // Undefined error, implementation specific.
    kInternalError = 11,
    // The allocated storage quota was exceeded, or memory could not be allocated.
    kOutOfMemoryError = 12,
    // The file was not found.
    kFileNotFoundError = 13,
    // The path does not exist
    kPathNotFoundError = 14,
    // The extracted target file does not exist
    kExtractTargetFileNotExistError = 15
};
class PerErrorDomain : public ara::core::ErrorDomain {
    static const ErrorDomain::IdType kId {101U};
public:
    PerErrorDomain() noexcept
        : ErrorDomain(kId)
    {
    }

    char_t const* Name() const noexcept override
    {
        return "Per";
    }

    char_t const* Message(CodeType errorCode) const noexcept override
    {
        const auto code = static_cast<ara::per::PerErrc>(errorCode);
        const auto iter = errorMessageMap.find(code);
        return (iter != errorMessageMap.end()) ? (iter->second).c_str() : "null";
    }
    void ThrowAsException(ara::core::ErrorCode const& errorCode) const noexcept(false) override
    {
        ara::core::ThrowOrTerminate<ara::core::Exception>(errorCode);
    }

    virtual ~PerErrorDomain()
    {}
private:
    ara::core::map<ara::per::PerErrc, ara::core::String> errorMessageMap {};
};

namespace internal {
    static const PerErrorDomain g_perErrorDomain;
}

inline constexpr ara::core::ErrorDomain const& GetPerDomain()
{
    return internal::g_perErrorDomain;
}

inline constexpr ara::core::ErrorCode MakeErrorCode(const PerErrc code,
    const ara::core::ErrorDomain::SupportDataType dataType)
{
    return ara::core::ErrorCode(static_cast<ara::core::ErrorDomain::CodeType>(code), GetPerDomain(), dataType);
}
}  // namespace per
}  // namespace ara

#endif
