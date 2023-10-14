/*
 * Description: Memory模块，提供Memory相关功能
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */
#ifndef HAF_CORE_MEMORY_H
#define HAF_CORE_MEMORY_H

#include "core/core.h"
#include "core/status.h"
#include "core/stream.h"
namespace Adsfi {
    enum class HafMemMallocPolicy {
        HAF_MEM_HUGE_FIRST = 0,  // 优先申请大页内存，如果大页内存不够，则使用普通页的内存
        HAF_MEM_HUGE_ONLY = 1,   // 仅申请大页，如果大页内存不够，则返回错误
        HAF_MEM_NORMAL_ONLY = 2  // 仅申请普通页
    };

    enum class HafMemcpyKind {
        HAF_MEM_COPY_HOST_TO_HOST = 0,      // Host内的内存复制
        HAF_MEM_COPY_HOST_TO_DEVICE = 1,    // Host到Device的内存复制
        HAF_MEM_COPY_DEVICE_TO_HOST = 2,    // Device到Host的内存复制
        HAF_MEM_COPY_DEVICE_TO_DEVICE = 3,  // Device内的内存复制
        HAF_MEM_COPY_MANAGED = 4
    };

    /* ******************************************************************************
      函 数 名		:  HafMallocDevice
      功能描述		:  申请Device上的内存。在Device上申请size大小的线性内存，通过*devPtr返
                    回已分配内存的指针。使用aclrtMalloc接口申请的内存，需要通过aclrtFree
                    接口释放内存。调用媒体数据处理的接口前，若需要申请Device上的内存存放输
                    入或输出数据，需调用aclrtDvppMalloc申请内存。
      输入参数		:  devPtr : 指向Device上已分配内存的指针的指针
                    theSize : 申请内存的大小，单位Byte; policy : 内存分配规则。
      返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafMallocDevice(void **devPtr, const size_t theSize, const HafMemMallocPolicy policy);

    /* ******************************************************************************
      函 数 名              :  HafMallocDvpp
      功能描述              :  申请dvpp相关函数(主要是图像处理函数)使用的内存。
      输入参数              :  dvppPtr : 返回“dvpp上已分配内存的指针”的指针; theSize : 申请内存的大小，单位Byte。
      返 回 值              :  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafMallocDvpp(void **dvppPtr, const size_t theSize);

    /* ******************************************************************************
      函 数 名		:  HafFreeDevice
      功能描述		:  释放Device上的内存。只能释放通过HafMallocDevice接口申请的内存。
      输入参数		:  devPtr : 待释放内存的指针。
      返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafFreeDevice(void **devPtr);

    /* ******************************************************************************
      函 数 名		:  HafFreeDvpp
      功能描述		:  释放dvpp申请的内存。只能释放通过HafMallocDvpp接口申请的内存。
      输入参数		:  dvppPtr : 待释放内存的指针。
      返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafFreeDvpp(void **dvppPtr);

    /* ******************************************************************************
      函 数 名		:  HafMemset
      功能描述		:  初始化内存，将内存中的内容设置为指定的值。要初始化的内存都在Host侧或Device侧，系统根据地址判定是Host还是Device。
      输入参数		:  ptr : 内存的起始地址; value : 设置的值; count : 需要设置为指定值的内存长度，单位Byte。
      返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafMemset(void *ptr, uint64_t maxCount, int32_t value, uint64_t count);

    /* ******************************************************************************
      函 数 名		:  HafMemcpy
      功能描述		:  实现Host和Device间的同步内存复制。
      输入参数		:  dst : 目的内存地址指针; src : 源内存地址指针; count: 内存复制的长度，单位Byte; kind :内存复制的类型。
      返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafMemcpy(void *dst, uint64_t maxCount, const void *src, uint64_t count, const HafMemcpyKind kind);
}

#endif // HAF_CORE_MEMORY_H
