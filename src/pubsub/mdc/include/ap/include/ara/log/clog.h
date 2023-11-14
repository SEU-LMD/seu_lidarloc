/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Created by l00519163 on 2020/5/23.
 */

#ifndef LOG_C_LOG_H
#define LOG_C_LOG_H

#ifdef __cplusplus
extern "C" {
#endif
// Loglevel Option
#define LOG_MDC_OFF 0
#define LOG_MDC_FATAL 1
#define LOG_MDC_ERROR 2
#define LOG_MDC_WARN 3
#define LOG_MDC_INFO 4
#define LOG_MDC_DEBUG 5
#define LOG_MDC_VERBOSE 6

/******************************************************************************
函数名称: InitLogging
功能说明: C语言日志初始化接口
输入参数：appId，appDescription
返 回 值：无
注意事项：一个进程注册一次，appid四个字节内
******************************************************************************/
// @uptrace{LOG-LOG-N01-F04}
void InitLogging(const char * const appId, const char * const appDescription);

/******************************************************************************
函数名称: Mlog
功能说明: C语言应用日志打印接口
输入参数：ctxId, level（日志级别CLogLevel枚举）, appDescription, fmt(打印内容)
返 回 值：无
注意事项：进程内区分不同的功能模块，ctxid四个字节内
******************************************************************************/
// C语言应用日志打印接口
// @uptrace{LOG-LOG-N01-F16}
void Mlog(const char * const ctxId, int level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif
#endif