/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: plog API for C
 */

#ifndef PLOG_CPLOG_H
#define PLOG_CPLOG_H

#include "time.h"
#include "stdint.h"

/**
 * @brief This is API for C lib, for more detail, please go to the C++ API,
 * MsgTimelineRecorder.hpp and ProfileLogWriter.hpp
 */
#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t PlogMsgUid;
typedef uint8_t PlogMoudleID;
typedef uint8_t PlogStageID;

static const PlogMoudleID PLOG_MOUDLE_ID_CM_SEND = 1;
static const PlogMoudleID PLOG_MOUDLE_ID_DDS_SEND = 5;
static const PlogMoudleID PLOG_MOUDLE_ID_DDS_RECV = 6;
static const uint64_t PLOG_TIMESTAMP_CURRENT = 0;

typedef struct {
    /** the stage ID, SHALL be less than MOUDLE_MAX_NUM */
    PlogStageID stageID;
    /** the stage's name, can only contain printable chars, and CANT have ',' '|' '{' and '}' */
    const char* stageName;
    /** the stage name's len, not including the \0 end, limit is STAGE_NAME_LEN_MAX */
    uint64_t stageNameLen;
} PlogStageName;

/**
 * @brief plog recorder handle, contains a void ptr, which SHALL NEVER be edited by the user
 */
typedef struct {
    void* recorderHandle;
} PlogRecorderHandle;

typedef struct {
    void* theSingleton;
} PlogWriterSingletonHandle;

/**
 * @brief This API is for helping controlling the life circle of the Singleton
 */
PlogWriterSingletonHandle PlogGetWriterSingleton(PlogMoudleID selfMoudleId);

/**
 * @brief This API is for helping controlling the death of the Singleton's life circle
 */
void PlogReleaseWriterSingleton(PlogWriterSingletonHandle* handle);

/**
 * @brief init the writer, then you can write to the log using the given mouldID,
 * for other detail, @see ProfileLogWriter::Init()
 * @param selfMoudleId the moudleID, @note if exceeds the limit: MOUDLE_NUM_MAX, the reserve moudle 0 will be inited
 * @param versionStr version's c stryle str, can be null or empty,
 * @note which must be printable and can't contain ',' or new line marker
 * @param verStrLen the len of version Str, not including the \0 end, limit is VERSION_STR_LEN_MAX
 * @param names the head ptr to a array of PlogStageName, can be null
 * @param nameLen the len of the PlogStageName array
 * @return 0 success
 */
int PlogInitWriter(PlogMoudleID selfMoudleId,
                   const char* versionStr, uint64_t verStrLen,
                   const PlogStageName* names, uint64_t nameLen);

/**
 * @brief get a recorder handle of the target moudle
 * @param selfMoudleId the moudle the recorder belonged to
 * @return the Handle, which shall never be editted
 */
PlogRecorderHandle PlogAcquireRecorder(PlogMoudleID selfMoudleId);

/**
 * @brief write the records in the recorder handle and reset the handle safely
 * @param recorderHandle ptr to the recorder
 * @return 0 success
 */
int PlogFiniAndWriteRecord(PlogRecorderHandle* recorderHandle);

/**
 * @brief record the user guid for make relation between different process, for now, this should only be called by DDS
 * @param recorderHandle the recorderhandle
 * @param guid the guid string, should could be null or empty, which can only contain printable chars and can't have ','
 * @param guidLen the str len of the guid, not including the \0 null end, limit is USER_GUID_LEN_MAX
 * @return 0 success
 */
int PlogRecordUserGuid(PlogRecorderHandle recorderHandle, const char* guid, uint64_t guidLen);

/**
 * @brief Record the reationship between different moudles in a same process
 * @param recorderHandle the recorder to recrod, of this moudle
 * @param relatedMsgUid the related msg's uid, this should from PlogGetMsgUid or
 * @param relatedMoudleID the id of the moudle that the msg comes from
 * @return 0 success
 */
int PlogRecordRelatedMsgGuid(PlogRecorderHandle recorderHandle, PlogMsgUid relatedMsgUid, PlogMoudleID relatedMoudleID);

/**
 * @brief record the time stamp at stage n
 * @param recorderHandle the recorder handle
 * @param stage the stage at
 * @param timeStamp the timestamp, in nsec, you can get this from PlogConvertSysTime2NsecU64,
 * or input the PLOG_TIMESTAMP_CURRENT let the moudle calculate for you
 * @param userInfo a user string, can be null or empty, can only have printable chars, CANT contain ',' '|' and '-'
 * @param userInfoLen the len of the user info, limit is USER_DEFINED_DATA_LEN_MAX
 * @return 0 success
 */
int PlogRecordTimestamp(
    PlogRecorderHandle recorderHandle,
    PlogStageID stage, uint64_t timeStamp,
    const char* userInfo, uint64_t userInfoLen);

/**
 * @brief Get the msg uid of this recorder, which is unique in one moudle of a process
 * @param recorderHandle the handle
 * @return PLOG_UID_MAX of handle is not vaild
 */
uint64_t PlogGetMsgUid(PlogRecorderHandle recorderHandle);

/**
 * @brief convert the sys time struct to u64 representing u64
 * @param sysTime got from 'clock_gettime(CLOCK_REALTIME, &t)'
 * @return
 */
uint64_t PlogConvertSysTime2NsecU64(const struct timespec* sysTime);

#ifdef __cplusplus
}
#endif

#endif // PLOG_CPLOG_H
