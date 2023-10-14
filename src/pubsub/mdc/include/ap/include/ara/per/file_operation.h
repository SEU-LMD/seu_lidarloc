/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 压缩、解压缩、拷贝等对外接口
 * Create: 2020-07-02
 */

#ifndef ARA_PER_FILE_OPERATION_H_
#define ARA_PER_FILE_OPERATION_H_

#include <sys/stat.h>
#include "ara/core/result.h"
#include "ara/core/string.h"
#include "ara/per/per_error_domain.h"
#include "ara/per/unique_handle.h"

namespace ara {
namespace per {
enum class PackageType : uint8_t {
    TAR_GZ = 0U,
    ZIP
};
class FileOperation {
public:
    FileOperation() = default;
    virtual ~FileOperation() = default;

    virtual ara::core::Result<void> ExtractAllFile(const ara::core::String& tarFile,
        const ara::core::String& extractionDirectory, const PackageType type = PackageType::TAR_GZ) const noexcept = 0;

    virtual ara::core::Result<void> ExtractSingleFile(const ara::core::String& tarFile,
        const ara::core::String& extractionDirectory, const ara::core::String& targetSingleFileName,
        const PackageType type = PackageType::TAR_GZ) const noexcept = 0;

    virtual ara::core::Result<void> RemovePath(const ara::core::String& pathName) const noexcept = 0;

    virtual ara::core::Result<void> CreateFile(const ara::core::String& pathName,
        const mode_t& mode) const noexcept = 0;

    virtual ara::core::Result<void> MakeDirectory(const ara::core::String& pathName,
        const mode_t& mode) const noexcept = 0;

    virtual ara::core::Result<void> MovePath(const ara::core::String& from,
        const ara::core::String& to) const noexcept = 0;

    virtual ara::core::Result<void> RenamePath(const ara::core::String& from,
        const ara::core::String& to) const noexcept = 0;

    virtual ara::core::Result<void> CopyPath(const ara::core::String& from,
        const ara::core::String& to) const noexcept = 0;
private:
    FileOperation(const FileOperation& obj) = delete;
    FileOperation& operator=(const FileOperation& obj) = delete;
};

ara::core::Result<ara::per::UniqueHandle<FileOperation>> CreateFileOperation() noexcept;
}
}

#endif  // ARA_PER_FILE_OPERATION_H
