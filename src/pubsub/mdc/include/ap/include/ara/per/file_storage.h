/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 普通文件代理的接口
 * Author: s00515168
 * Create: 2020-05-13
 * Modify: 2020-06-10
 * Notes: 无
 */

#ifndef ARA_PER_FILE_STORAGE_H_
#define ARA_PER_FILE_STORAGE_H_
#include <memory>
#include "ara/per/per_error_domain.h"
#include "ara/per/basic_operations.h"
#include "ara/per/read_write_accessor.h"
#include "ara/per/unique_handle.h"

namespace ara {
namespace per {
class FileStorage {
public:
    FileStorage() = default;
    virtual ~FileStorage() = default;

    virtual ara::core::Result<ara::core::Vector<ara::core::String>> GetAllFileNames() const noexcept = 0;
    virtual ara::core::Result<void> DeleteFile(const ara::core::StringView& fileName) noexcept = 0;
    virtual ara::core::Result<bool> FileExists(const ara::core::StringView& fileName) const noexcept = 0;
    virtual ara::core::Result<void> RecoverFile(const ara::core::StringView& fileName) const noexcept = 0;

    virtual ara::core::Result<ara::per::UniqueHandle<ReadWriteAccessor>> OpenFileReadWrite(
        const ara::core::StringView& fileName,
        BasicOperations::OpenMode const mode = static_cast<BasicOperations::OpenMode>(0)) const noexcept = 0;
    virtual ara::core::Result<ara::per::UniqueHandle<ReadAccessor>> OpenFileReadOnly(
        const ara::core::StringView& fileName,
        BasicOperations::OpenMode const mode = static_cast<BasicOperations::OpenMode>(0)) const noexcept = 0;

    virtual ara::core::Result<ara::per::UniqueHandle<ReadWriteAccessor>> OpenFileWriteOnly(
        const ara::core::StringView& fileName,
        BasicOperations::OpenMode const mode = static_cast<BasicOperations::OpenMode>(0)) const noexcept = 0;
private:
    FileStorage(const FileStorage& obj) = delete;
    FileStorage& operator=(const FileStorage& obj) = delete;
};

ara::core::Result<ara::per::UniqueHandle<FileStorage>> OpenFileStorage(const ara::core::StringView& fs) noexcept;

ara::core::Result<void> RecoverAllFiles(const ara::core::StringView& fs) noexcept;
}  // namespace per
}  // namespace ara

#endif  // ARA_PER_FILE_STORAGE_H_
