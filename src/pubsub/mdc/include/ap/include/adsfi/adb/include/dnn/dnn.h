/*
 * Description: NN模块，提供NN相关API
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */

#ifndef HAF_DNN_DNN_H
#define HAF_DNN_DNN_H

#include <string>
#include <mutex>
#include <map>
#include "core/core.h"
#include "core/stream.h"
#include "acl/acl.h"

namespace Adsfi {
using HafCallback = aclrtCallback;
using HafModelDataset = aclmdlDataset;
struct HafDNNHandle {
    uint32_t id{};
    aclmdlDesc* modelDesc{};
    aclmdlDataset* inData{};
    aclmdlDataset* outData{};

    uint32_t memoryPoolIdx{};
    uint32_t addMemoryPoolIdx{};
    std::map<aclmdlDataset*, aclmdlDataset*> memoryPool;
    std::map<const aclmdlDataset*, uint32_t> memoryPoolStatus;
    std::map<uint32_t, aclmdlDataset*> memoryPoolIdxMap;
    std::recursive_mutex memoryPoolMtx;
};

HafStatus HafDNNModelInitialize(HafDNNHandle &network, const std::string modelFilename);
HafStatus HafDNNRelease(HafDNNHandle &network);
HafStatus HafDNNModelCreateInput(HafDNNHandle &network);
HafStatus HafDNNModelAddInput(const HafDNNHandle &network, void const * const buffer, const size_t bufferSize,
    const size_t index);
HafStatus HafDNNModelCreateOutput(HafDNNHandle &network);
HafStatus HafDNNModelGetInputNum(const HafDNNHandle &network, size_t &num);
HafStatus HafDNNModelGetOutputNum(const HafDNNHandle &network, size_t &num);
HafStatus HafDNNModelGetOutputBuffer(const HafDNNHandle &network, const size_t index, void ** const buffer,
    size_t &bufferSize);
HafStatus HafDNNModelInputDataDestroy(HafDNNHandle &network);
HafStatus HafDNNModelOutputDataDestroy(HafDNNHandle &network);
HafStatus HafDNNModelProcess(const HafDNNHandle &network);

#if defined(MDC_PRODUCTION_CORE)
HafStatus HafDNNModelProcessAsync(HafDNNHandle &network, const HafCallback function, const HafStream stream);
HafStatus HafDNNInitMemoryPool(HafDNNHandle &network, const uint32_t memoryPoolSize = 1U);
HafStatus HafDNNDestroyMemoryPool(HafDNNHandle &network);
HafStatus HafDNNSetMempoolStatus(HafDNNHandle &network, const HafModelDataset * const input,
    const HafMempoolStatus &status);

HafStatus HafDNNSubscribeReport(const HafStream &stream, const uint64_t threadId);
HafStatus HafDNNUnSubscribeReport(const HafStream &stream, const uint64_t threadId);
HafStatus HafDNNProcessReport(const int32_t &timeout);

HafStatus HafDNNMemoryPoolAddInput(HafDNNHandle &network, std::vector<void *> buffer,
    const std::vector<size_t> bufferSize);
#endif
HafStatus HafDNNModelGetOutputBuffer(const HafModelDataset * const output, const size_t index, void ** const buffer,
    size_t &bufferSize);
}
#endif
