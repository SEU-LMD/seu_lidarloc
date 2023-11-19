/**
 * Copyright (C) 2018. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef DVPP_PNG_H
#define DVPP_PNG_H

#include "DvppCommon.h"
#include "ExportMacro.h"
// pngd input param
struct PngInputInfoAPI {
    void* inputData; // input data image
    uint64_t inputSize; // buf size for verifying input data
    void* address; // Memory address
    uint64_t size; // Memory size
    int32_t transformFlag; // Conversion flag, 1 means RGBA to RGB. 0 means keep the original format.

    PngInputInfoAPI() :
        inputData(nullptr),
        inputSize(0),
        address(nullptr),
        size(0),
        transformFlag(0) {}
};

// pngd output param
struct PngOutputInfoAPI {
    void* outputData; //output data image.
    uint64_t outputSize; // output data size.
    void* address; // Memory address.
    uint64_t size; // Memory size.
    int32_t format; // Output format, 2 means rgb output, 6 means rgba output.
    uint32_t width; // output width.
    uint32_t high; // output high.
    uint32_t widthAlign; // Width alignment.
    uint32_t highAlign; // High alignment.

    PngOutputInfoAPI() :
        outputData(nullptr),
        outputSize(0),
        address(nullptr),
        size(0),
        format(0),
        width(0),
        high(0),
        widthAlign(0),
        highAlign(0) {}

    void FreeOutputMemory();
};
#endif // DVPP_PNG_H