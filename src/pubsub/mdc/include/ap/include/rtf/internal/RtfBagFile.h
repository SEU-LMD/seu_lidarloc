/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description:
 *      This file is the implement of class BagFile.
 *      BagFile will create a bag file and provide read or write operation
 * Author: xujunbin 00514454
 * Create: 2019-11-30
 * Notes: NA
 * History: 2019-11-30 xujunbin 00514454 revised this file.
 */
#ifndef RTF_BAG_FILE_H
#define RTF_BAG_FILE_H

#include <iostream>
#include <set>

#include "ara/core/string.h"
#include "ara/core/map.h"
#include "ara/core/vector.h"
#include "rtf/internal/RtfBagStructs.h"
#include "rtf/internal/RtfHeader.h"
#include "rtf/internal/RtfBuffer.h"
#include "rtf/maintaind/impl_type_eventinfo.h"
#include "rtf/rtfbag/RtfBagInfo.h"

namespace rtf {
namespace rtfbag {
class RtfBagFile {
    friend class RtfView;
public:
    RtfBagFile();
    ~RtfBagFile();

    /*
     * @Description: 打开文件并写入文件头相关信息
     * @param: input ara::core::String const& fileName
     * @return: true: 成功 false: 失败
     */
    bool OpenWrite(ara::core::String const& fileName, ara::core::String const& filePath, ara::core::String const& file);

    /*
     * @Description: 关闭文件写入，此时会写入ConnetionInfo，ChunkInfo等信息
     * @param: none
     * @return: true: 成功 false: 失败
     */
    bool CloseWrite();

    bool Write(rtf::rtfbag::EventMsg const& eventMsg, uint64_t const& time, uint8_t const *msgBuff, uint32_t buffLen);

    bool OpenRead(ara::core::String const& fileName);
    bool CloseRead();
    bool ReadMsgDataIntoStream(MessageIndex const& msgIndex, RtfBuffer& buffer) const;
    bool ReadMsgDataSize(MessageIndex const& msgIndex, uint32_t& size) const;
    /*
     * @Description: 获取当前bag文件的文件大小
     * @param: none
     * @return: uint64_t 文件大小
     */
    uint64_t GetFileSize() const;

    /*
     * @Description: 获取当前打开文件的文件名称
     * @param: none
     * @return: ara::core::String 文件名称
     */
    ara::core::String GetFileName() const;
    void SetMaxChunkSize (uint32_t chunkSize);

    /*
     * @Description: 获取bag文件头信息
     * @param:
     * @return: true: 成功 false： 失败
     */
    bool GetFileHeaderInfo(BagFileHeadInfo& headerInfo);

    void SetStartTime();
    void SetStopTime();

private:
    bool CloseFile();
    // write chunk
    bool StartChunkWriting(uint64_t time);
    bool StopChunkWriting();
    void ResetBagFile();
    // write record
    bool WriteVersion() const;
    bool WriteFileHeaderRecord();
    bool WriteRecordHeader(ara::core::Vector<ara::core::String> const& fieldName,
        ara::core::Vector<ara::core::String> const& fieldValue) const;
    bool WriteChunkInfoRecords() const;
    bool WriteChunkInfoRecord(ChunkInfo chunkInfoRecord) const;
    bool WriteConnectionRecords() const;
    bool WriteConnectionRecord(Connection connRecord) const;
    bool WriteChunkHeaderRecord(uint32_t currChunkSize) const;
    bool WriteMessageRecord(uint32_t connnectionId,
        uint64_t time, uint8_t const *msgBuff, uint32_t msgLen);
    bool WriteChunkIndexRecords() const;
    bool WriteChunkIndexRecord(uint32_t id, const std::multiset<MessageIndex> &indexSet) const;
    uint32_t GetRecordHeaderLen(ara::core::Vector<ara::core::String> const& fieldName,
        ara::core::Vector<ara::core::String> const& fieldValue) const;
    uint32_t GetConnectionId(rtf::rtfbag::EventMsg const& eventMsg);

    // read record
    bool StartReading();
    bool ReadVersion();
    bool ReadFileHeaderRecord();
    bool ReadConnectionRecord();
    bool ReadConnectionFeild(const ReadMap &readMap, Connection &connection);

    bool ReadChunkInfoRecord();
    bool ReadChunkIndexRecord();
    bool ReadChunkHeader(ChunkHeader& chunkHeader) const;
    bool OptionMatch(const ReadMap& readMap, const uint8_t& opt) const;
    // read header
    template<typename T>
    bool ReadField(const ReadMap& readMap, const ara::core::String& field, T* data) const;
    bool ReadField(const ReadMap& readMap, const ara::core::String& field, ara::core::String& data) const;
    bool ReadHeader(RtfHeader& header) const;
    bool ReadMsgDataFromBuffer(RtfBuffer& buffer, const uint64_t& offset) const;
    void UpdateVersion(BagFileHeadInfo& headerInfo) const;

    bool CheckSystemIsAos() const;
    void GetFileHeaderRecordTimeInfo(BagFileHeadInfo& headerInfo) const;

private:
    ara::core::String fileName_;
    FILE* file_;
    uint32_t maxChunkSize_;
    uint64_t fileSize_;
    uint32_t chunkCount_;
    uint32_t connCount_;
    uint64_t connPos_;
    BagFileHeader bagFileHeader_;
    uint64_t bagFileHeaderPos_;
    uint32_t version_;
    ara::core::String BAG_VERSION_;  // current bag version
    bool fileWriteOpen_;
    bool fileReadOpen_;

    // Current chunk
    bool chunkOpen_;
    uint64_t currChunkDataStartPos_;
    uint64_t currChunkDataEndPos_;
    ChunkInfo currChunkInfo_;
    ara::core::Vector<ChunkInfo> chunks_;

    ara::core::Map<ara::core::String, uint32_t> eventConnectionId_;
    ara::core::Map<ara::core::String, Connection> eventConnection_;
    ara::core::Map<uint32_t, Connection> connections_;

    ara::core::Map<uint32_t, std::multiset<MessageIndex>> connectionIdIndex_;
    ara::core::Map<uint32_t, std::multiset<MessageIndex>> currChunkIndex_;

    uint64_t startRecordRealTime_;
    uint64_t startRecordVirtualTime_;
    uint64_t stopRecordRealTime_;
    uint64_t stopRecordVirtualTime_;
};
}  // namespace rtfbag
}  // namespace rtf
#endif  // RTF_BAG_FILE_H
