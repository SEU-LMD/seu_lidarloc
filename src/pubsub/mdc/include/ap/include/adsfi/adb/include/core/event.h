/*
 * Description:  Event模块，提供Event相关功能
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 */

#ifndef HAF_CORE_EVENT_H
#define HAF_CORE_EVENT_H

#include "core/core.h"
#include "core/status.h"
#include "core/stream.h"
namespace Adsfi {
    /* ******************************************************************************
        函 数 名		:  HafEventCreate
        功能描述		:  创建一个Event
        输入参数		:  新创建的event的指针
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventCreate(HafEvent &event);

    /* ******************************************************************************
        函 数 名		:  HafEventDestroy
        功能描述		:  销毁一个Event，只能销毁通过HafEventCreate接口创建的Event。销毁某个
                    Event时，用户需确保等待HafEventSynchronize接口或HafStreamWaitEvent
                    接口涉及的任务都结束后，再销毁。
        输入参数		:  待销毁的event
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventDestroy(HafEvent &event);

    /* ******************************************************************************
        函 数 名		:  HafEventRecord
        功能描述		:  在Stream中记录一个Event。
        输入参数		:  event : 待记录的event
                    stream : 将该event记录在指定stream中，若使用默认Stream，此处设为NULL
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventRecord(const HafEvent event, const HafStream stream);

    /* ******************************************************************************
        函 数 名		:  HafEventReset
        功能描述		:  复位一个Event。用户需确保等待Stream中的任务都完成后，再复位Event。
        输入参数		:  event : 待复位的event; stream : 指定event所在的stream。
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventReset(const HafEvent event, const HafStream stream);

    /* ******************************************************************************
        函 数 名		:  HafEventQuery
        功能描述		:  查询指定Event的状态。
        输入参数		:  event : 指定待查询的event; status : event的状态。
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventQuery(const HafEvent event, HafEventStatus &status);

    /* ******************************************************************************
        函 数 名		:  HafEventSynchronize
        功能描述		:  阻塞Host运行，等待Event完成。
        输入参数		:  指定需等待的event
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventSynchronize(const HafEvent event);

    /* ******************************************************************************
        函 数 名		:  HafEventElapsedTime
        功能描述		:  统计两个Event之间的耗时，同步接口。接口调用顺序：调用HafEventCreate
                    接口创建Event-->调用HafEventRecord接口在同一个Stream中记录起始Event
                    、结尾Event-->调用HafStreamSynchronize接口阻塞应用程序运行，直到指定
                    Stream中的所有任务都完成-->调用HafEventElapsedTime接口统计两个Event之间的耗时
        输入参数		:  ms : 表示两个Event之间的耗时，单位为毫秒; start : 起始Event; end : 结尾Event。
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafEventElapsedTime(float32_t& ms, const HafEvent start, const HafEvent end);

    /* ******************************************************************************
        函 数 名		:  HafStreamWaitEvent
        功能描述		:  阻塞指定Stream的运行，直到指定的Event完成。支持多个Stream等待同一个Event的场景。
        输入参数		:  event : 需等待的event; stream : 指定需要等待event完成的stream，如果使用默认Stream，此处设置为NULL。
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafStreamWaitEvent(const HafEvent event, const HafStream stream);

    /* ******************************************************************************
        函 数 名		:  HafDeviceSynchronize
        功能描述		:  阻塞Host运行，直到正在运算中的Device完成运算。多Device场景下，调用该
                    接口等待的是当前Context对应的Device。
        输入参数		:  无
        返 回 值		:  0表示成功，其它值表示失败
    ****************************************************************************** */
    HafStatus HafDeviceSynchronize();
}

#endif // HAF_CORE_EVENT_H__
