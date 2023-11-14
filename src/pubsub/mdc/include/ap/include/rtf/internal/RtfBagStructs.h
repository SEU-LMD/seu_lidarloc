/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: Structs definition of RtfBagFile class
 * Create: 2019-12-3
 */
#ifndef RTF_BAG_STRUCTS_H
#define RTF_BAG_STRUCTS_H

#include "ara/core/map.h"
#include "ara/core/vector.h"
#include "ara/core/string.h"
#include "rtf/maintaind/impl_type_eventinfo.h"
#include "rtf/maintaind/impl_type_drivertype.h"

namespace rtf {
namespace rtfbag {
struct BagFileHeader {
    BagFileHeader() : connectionCount(0), chunkCount(0), connectionPos(0)
    {}
    uint32_t connectionCount;
    uint32_t chunkCount;
    uint64_t connectionPos;
};

struct ChunkHeader {
    ChunkHeader(): chunkSize(0)
    {}
    uint32_t chunkSize;  // size of the chunk in bytes
};

struct ConnectionHeader {
    ConnectionHeader() : id(0)
    {}
    uint32_t id;
    ara::core::String event;
};

struct Connection {
    Connection() : driverType(rtf::maintaind::DriverType::DDS) {}
    ConnectionHeader header;
    ara::core::String type;
    ara::core::String msgDef;
    rtf::maintaind::DriverType driverType;
    rtf::maintaind::EventInfo eventInfo;
};

struct MessageIndex {
    MessageIndex() : time(0), msgPos(0)
    {}
    uint64_t time;    // timestamp of the message
    uint64_t msgPos;  // absolute pos (byte) of the message record
    bool operator<(MessageIndex const& msgIndex) const
    {
        return time < msgIndex.time;
    }
};

struct ChunkInfoHeader {
    ChunkInfoHeader() : connectionCount(0), startTime(0), endTime(0), chunkPos(0)
    {}
    uint32_t connectionCount;
    uint64_t startTime;
    uint64_t endTime;
    uint64_t chunkPos;
};

struct ChunkInfo {
    ChunkInfo() {}
    ChunkInfoHeader header;
    ara::core::Map<uint32_t, uint32_t> connectionIdCount;
};

struct EventMsg {
    EventMsg() : driverType(rtf::maintaind::DriverType::DDS) {}
    ara::core::String eventName;
    ara::core::String dataType;
    ara::core::String msgDef;
    rtf::maintaind::DriverType driverType;
    rtf::maintaind::EventInfo eventInfo;
};

const uint8_t MESSAGE_OP = 0x02;
const uint8_t FLIE_HEADER_OP = 0x03;
const uint8_t CHUNK_INDEX_OP = 0x04;
const uint8_t CHUNK_HEADER_OP = 0x05;
const uint8_t CHUNK_INFO_OP = 0x06;
const uint8_t CONNECTION_OP = 0x07;
const uint32_t RATE_OF_BIT = 1024;
const uint32_t INIT_BAG_VERSION = 100;
}  // namespace rtfbag
}  // namespace rtf
#endif