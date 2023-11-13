/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: PCIe transmission channel interface
 * Author: g00544770
 * Create: 2020-05-12
 */

#ifndef PCIETRANSAPI_H
#define PCIETRANSAPI_H
#include <memory>

namespace PCIeTrans {
/* 描述: 内存拷贝接口，把待发送数据拷入对应传输通道的内存中
   入参: addr: 发送通道的首地址，需与AllocMemory接口返回的地址一致;
         pData: 待发送数据的首地址;
         length: 待发送数据的长度，需与传入AllocMemory接口的长度一致 */
int MemcpyDMA(uintptr_t dstAddr, uintptr_t srcAddr, size_t length);

/*------------------ 发送接口 ------------------*/
/* 描述: Host侧申请内存接口，成功返回0，失败返回-1
   入参: channelId: 取值0 ~ 3，对应发送至Mini0 ~ Mini3;
         length: 待发送数据长度;
         addr: 可传入任意值，函数返回发送通道首地址，用户需之后把待发送数据拷入该地址 */
int AllocMemory(const int channelId, const int length, uintptr_t &addr);

/* 描述: Host侧向Mini侧发送接口，成功返回0，失败返回-1
   入参: channelId: 取值0 ~ 3，对应发送至Mini0 ~ Mini3;
         length: 待发送数据长度，需与传给AllocMemory的length一致;
         addr: 待发送数据首地址，需与AllocMemory返回的地址一致 */
int Send(const int channelId, const int length, const uintptr_t addr);

/* 描述: Host侧丢弃掉当前数据接口，成功返回0，失败返回-1
   入参: channelId: 取值0 ~ 3，对应发送至Mini0 ~ Mini3;
         length: 待发送数据长度，需与传给AllocMemory的length一致;
         addr: 待发送数据首地址，需与AllocMemory返回的地址一致 */
int DiscardData(const int channelId, const int length, const uintptr_t addr);

/*------------------ 接收接口 ------------------*/
/* 描述: Mini侧等待接收数据接口，成功返回0，失败返回-1
   入参: addr: 可传入任意值，函数返回接收通道首地址，用户之后可从该地址把数据拷走;
         length: 已接收的数据长度;
         timeout: 等待超时时间，默认为0，永不超时 */
int WaitForData(const int channelId, int &length, uintptr_t &addr, const int timeout = 0);

/* 描述: Mini侧取走数据后清空内存接口，成功返回0，失败返回-1
   入参: addr: 接收通道首地址，需与WaitForData接口返回的地址一致;
         length: 已接收数据长度，需与WaitForData接口返回的长度一致 */
int FreeMemory(const int channelId, const int length, const uintptr_t addr);
}
#endif
