/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: A writer class to manage writing in profile log
 */

#ifndef PLOG_PROFILELOGWRITER_HPP
#define PLOG_PROFILELOGWRITER_HPP

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <mutex>
#include <atomic>

#include "PLogDefsAndLimits.hpp"

namespace rbs {
namespace plog {
class ShmLogSpaceWriter;
class MsgTimelineRecorder;
class ProfileLogWriterImpl;
struct ProfileRecord;

class ProfileLogWriter : public std::enable_shared_from_this<ProfileLogWriter> {
public:
    /**
     * @brief Get the Writer Instance related to the moudleID
     * @param moudleID the id of the moudle (the id should be < MOUDLE_MAX,
     * otherwise we will return the instance to moudle0)
     * @return the instance (it's a ref, so if you dont want to control the lifecycle,
     * you can use a ref to get the rtn value, which is faster)
     * @example const auto& ins = GetInstance(MOIDLE_CM_SEND);
     */
    static const std::shared_ptr<ProfileLogWriter>& GetInstance(MoudleID moudleID);

    /**
     * @brief Init the instance, where we init the shm, set the version Str and stage lists
     * @param versionStr version str, whose len should be <= VERSION_STR_LEN_MAX
     * @param stageList a list of the name of the stages used,
     * the stage id should be less than STAGE_NUM_MAX(16),
     * the stage's name should be shorter than STAGE_NAME_LEN_MAX (not including the null end '\0')
     * @return the inited Instance (which will exsist for the process's whole lifetime)
     */
    PlogReturnValue Init(std::string versionStr, const StageNameList& stageList);

    /**
     * @brief Get a recorder, which is unqiue for one moudle's writer
     * @return a Recorder @see MsgTimelineRecorder
     * @note if the Writer is not ready, a zombie recorder will still be made,
     * and when you use the zombie recorder, nothing happend
     */
    std::unique_ptr<MsgTimelineRecorder> AcquireRecorder();

    MsgTimelineRecorder AcquireRecorderObj();

    /**
     * @brief write the msg recordered in the recorder to the shmMem log
     * @return the rtn code, success -> PlogReturnValue::OK
     */
    PlogReturnValue WriteMsgTimeLine(const std::unique_ptr<MsgTimelineRecorder>);
    PlogReturnValue WriteMsgTimeLine(MsgTimelineRecorder);

    ProfileLogWriter(const ProfileLogWriter&) = delete;
    ProfileLogWriter(ProfileLogWriter&&) = delete;
    ProfileLogWriter operator=(const ProfileLogWriter&)& = delete;
    ProfileLogWriter operator=(ProfileLogWriter&&)& = delete;

    virtual ~ProfileLogWriter() = default;

private:
    ProfileLogWriter() = default;

    static std::array<std::shared_ptr<ProfileLogWriter>, MOUDLE_MAX_NUM>& GetInstanceTable();

    std::unique_ptr<ProfileLogWriterImpl> impl_;

    friend class MsgTimelineRecorderImpl;
    friend class ProfileLogWriterImpl;
    friend class MsgTimelineRecorder;
};
}
}


#endif // PLOG_PROFILELOGWRITER_HPP
