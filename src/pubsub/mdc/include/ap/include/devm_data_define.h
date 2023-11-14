/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 设备管理数据类型定义
 * Author: o00288819
 * Create: 2020-03-26
 */
#ifndef DEVM_DATA_DEFINE_H
#define DEVM_DATA_DEFINE_H

#include <functional>
#include "devm_base_type.h"

namespace mdc {
namespace devm {
using LogHandler = std::function<void(std::uint8_t, const String&)>;
using CameraFaultNotifyHandler = std::function<std::uint32_t(Vector<std::uint8_t>&, Vector<std::uint8_t>&)>;

enum class DevmAttr : std::uint8_t {
    CPU_USAGE = 0U,
    MEM_USAGE = 1U,
    ESN_LABLE = 2U,
    FANDUTY = 3U,
    VOLTAGE = 4U,
    TEMPERATURE = 5U,
    NET_CARD = 6U,
    VERSION_HW = 7U,
    VERSION_SW = 8U,
    SOC_NET_INFO = 9U,
    DEVICE_STATE = 10U,
    SENSOR_TIME_SYNC = 106U,
    SENSOR_STATE = 109U,
};

enum DevmLogLevel {
    D_LOG_LEVEL_BEGIN,
    D_LOG_LEVEL_FATAL,
    D_LOG_LEVEL_ERROR,
    D_LOG_LEVEL_WARN,
    D_LOG_LEVEL_INFO,
    D_LOG_LEVEL_DEBUG
};

enum class DevmWorkState : std::uint8_t {
    STATE_OK,
    STATE_ERROR
};

enum class DevmDevState : std::uint8_t {
    DEV_STATE_OK,
    DEV_STATE_ERROR,
    DEV_STATE_UNKNOW // not use
};

enum class DevmVerSionState : std::uint8_t {
    STATE_OK,
    STATE_ABNORMAL
};

enum DevmResultCode : std::int32_t {
    DEVM_OK = 0,
    DEVM_ERROR,
    DEVM_NULL_POINTER,
    DEVM_UNSUPPORT,
    DEVM_TIME_OUT,
    DEVM_INVALID_STATUS,
    DEVM_NO_VALUE,
    DEVM_UNKOWN
};

enum class UpdatePackgType : std::uint8_t {
    MCU_FIRMWARE = 1U,
    MCU_APP_FIRMWARE,
    MCU_BASE_FIRMWARE,
    CPLD_FIRMWARE,
    UNKOWN_FIRMWARE
};

enum class UpdateRetCode : std::uint8_t {
    UPDATE_OK = 0U,
    UPDATE_ERROR,
    UPDATE_NONEED,
    UPDATE_RUNNING,
    UPDATE_UPGRADING,
    UPDATE_TIMEOUT,
    UPDATE_OFFLINE,
    UPDATE_RETURNEMPTY,
    UPDATE_UNKOWN
};

enum class SystemActionType : std::uint8_t {
    TYPE_SOCA = 0U,
    TYPE_SOCB = 1U,
    TYPE_MCU = 7U,
    TYPE_SOC_ALL = 8U,
    TYPE_SOC_MCU_ALL = 9U,
    TYPE_UNKOWN = 10U
};

enum class SystemAction : std::uint8_t {
    ACT_SLEEP = 0U,
    ACT_POWER_OFF,
    ACT_HOT_REBOOT,
    ACT_COLD_REBOOT,
    ACT_UNKOWN
};

enum class SensorAction : std::uint8_t {
    ACT_SLEEP = 0U,
    ACT_WAKEUP,
    ACT_REBOOT,
    ACT_UNKOWN
};

enum class MdcDeviceType : std::uint8_t {
    DEVICE_TYPE_BEGIN = 0U,
    DEVICE_TYPE_SOC = 1U,
    DEVICE_TYPE_MCU = 2U,
    DEVICE_TYPE_LANSW = 3U,
    DEVICE_TYPE_CPLD = 4U,
    DEVICE_TYPE_UFS = 5U,
    DEVICE_TYPE_PORT = 6U,
    DEVICE_TYPE_CAN = 7U,
    DEVICE_TYPE_NETCARD = 8U,
    DEVICE_TYPE_SERDES = 9U,
    DEVICE_TYPE_ISP = 10U,
    DEVICE_TYPE_MINI = 11U,
    DEVICE_TYPE_HOST = 12U,
    DEVICE_TYPE_SSD = 13U,
    DEVICE_TYPE_PCIE = 14U,
    DEVICE_TYPE_LIDAR = 101U,
    DEVICE_TYPE_RADAR = 102U,
    DEVICE_TYPE_INS = 103U,
    DEVICE_TYPE_USS = 104U,
    DEVICE_TYPE_CAMERA = 105U,
    DEVICE_TYPE_UNKNOW = 199U,
};

struct MdcSensorMsg {
    std::uint32_t msgId;
    Vector<std::uint8_t> msgData;
    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(msgId);
        fun(msgData);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(msgId);
        fun(msgData);
    }

    bool operator ==(const mdc::devm::MdcSensorMsg& t) const
    {
        return ((msgId == t.msgId) && (msgData == t.msgData));
    }
};

struct MdcSensorInMsg {
    std::uint32_t sendMsgId;
    std::uint32_t recvMsgId;
    Vector<std::uint8_t> sendMsgData;
    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(sendMsgId);
        fun(recvMsgId);
        fun(sendMsgData);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(sendMsgId);
        fun(recvMsgId);
        fun(sendMsgData);
    }

    bool operator ==(const mdc::devm::MdcSensorInMsg& t) const
    {
        return ((sendMsgId == t.sendMsgId) && (recvMsgId == t.recvMsgId) && (sendMsgData == t.sendMsgData));
    }
};

struct MdcConfigItem {
    String key;
    String value;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(key);
        fun(value);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(key);
        fun(value);
    }

    bool operator ==(const mdc::devm::MdcConfigItem& t) const
    {
        return ((key == t.key) && (value == t.value));
    }
};

struct MdcUpdateInfo {
    String version;
    String packgName;                // 区分同一个型号，不同的软件包
    String deviceType;               // 区分不同的型号
    std::uint8_t versionStatus;      // 版本号状态,获取版本号异常错误信息
    std::uint8_t mainSub;            // 主备

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(version);
        fun(packgName);
        fun(deviceType);
        fun(versionStatus);
        fun(mainSub);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(version);
        fun(packgName);
        fun(deviceType);
        fun(versionStatus);
        fun(mainSub);
    }

    bool operator ==(const mdc::devm::MdcUpdateInfo& t) const
    {
        return ((version == t.version) && (packgName == t.packgName) &&
                (deviceType == t.deviceType) && (versionStatus == t.versionStatus) &&
                (mainSub == t.mainSub));
    }
};

struct MdcCanIdInfo {
    std::uint32_t canId {0U};
    std::uint32_t canIdType {0U};

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(canId);
        fun(canIdType);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(canId);
        fun(canIdType);
    }

    bool operator ==(const mdc::devm::MdcCanIdInfo& t) const
    {
        return ((canId == t.canId) && (canIdType == t.canIdType));
    }
};

struct MdcCanCheckList {
    std::uint8_t idNumber {0U};
    std::uint8_t controlMethod {0U};
    Vector<MdcCanIdInfo> canIdInfos;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(idNumber);
        fun(controlMethod);
        fun(canIdInfos);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(idNumber);
        fun(controlMethod);
        fun(canIdInfos);
    }

    bool operator ==(const mdc::devm::MdcCanCheckList& t) const
    {
        return ((idNumber == t.idNumber) && (controlMethod == t.controlMethod) &&
                (canIdInfos == t.canIdInfos));
    }
};

struct ComCtrlConfig {
    std::uint8_t channelId {0U};
    MdcCanCheckList upStream;
    MdcCanCheckList downStream;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(channelId);
        fun(upStream);
        fun(downStream);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(channelId);
        fun(upStream);
        fun(downStream);
    }

    bool operator ==(const mdc::devm::ComCtrlConfig& t) const
    {
        return ((channelId == t.channelId) && (upStream == t.upStream) &&
                (downStream == t.downStream));
    }
};

struct MdcDeviceInfo {
    String deviceName;
    String swVersion;
    String hwVersion;
    String chipName;
    String chipType;
    std::uint32_t deviceId {0U};
    std::uint8_t deviceType {0U};
    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(deviceName);
        fun(swVersion);
        fun(hwVersion);
        fun(chipName);
        fun(chipType);
        fun(deviceId);
        fun(deviceType);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(deviceName);
        fun(swVersion);
        fun(hwVersion);
        fun(chipName);
        fun(chipType);
        fun(deviceId);
        fun(deviceType);
    }

    bool operator ==(const mdc::devm::MdcDeviceInfo& t) const
    {
        return ((deviceName == t.deviceName) && (swVersion == t.swVersion) &&
                (hwVersion == t.hwVersion) && (chipName == t.chipName) &&
                (chipType == t.chipType) && (deviceId == t.deviceId) &&
                (deviceType == t.deviceType));
    }
};

struct DevmUpdateInfo {
    String deviceName;
    std::uint32_t deviceId {0U};
    std::uint8_t deviceType {0U};
    std::uint8_t upgradable {0U};
    Vector<MdcUpdateInfo> updateList;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(deviceName);
        fun(deviceId);
        fun(deviceType);
        fun(upgradable);
        fun(updateList);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(deviceName);
        fun(deviceId);
        fun(deviceType);
        fun(upgradable);
        fun(updateList);
    }

    bool operator ==(const mdc::devm::DevmUpdateInfo& t) const
    {
        return ((deviceName == t.deviceName) && (deviceId == t.deviceId) &&
                (deviceType == t.deviceType) && (upgradable == t.upgradable) &&
                (updateList == t.updateList));
    }
};

struct MdcDevObj {
    String          deviceName;
    std::uint32_t   deviceId {0U};
    std::uint8_t    deviceType {0U};
    std::uint8_t    upgradable {0U};

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(deviceName);
        fun(deviceId);
        fun(deviceType);
        fun(upgradable);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(deviceName);
        fun(deviceId);
        fun(deviceType);
        fun(upgradable);
    }

    bool operator ==(const mdc::devm::MdcDevObj& t) const
    {
        return ((deviceName == t.deviceName) && (deviceId == t.deviceId) &&
                (deviceType == t.deviceType) && (upgradable == t.upgradable));
    }
};

struct UpgradableDevObj {
    String deviceName;
    std::uint8_t deviceStatus {static_cast<std::uint8_t>(DevmDevState::DEV_STATE_ERROR)};
    Vector<MdcUpdateInfo> upgradeList;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(deviceName);
        fun(deviceStatus);
        fun(upgradeList);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(deviceName);
        fun(deviceStatus);
        fun(upgradeList);
    }

    bool operator ==(const mdc::devm::UpgradableDevObj& t) const
    {
        return ((deviceName == t.deviceName) && (deviceStatus == t.deviceStatus) &&
                (upgradeList == t.upgradeList));
    }
};

struct DidInfoItem {
    uint16_t did;
    Vector<uint8_t> value;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun) const // duplication code with diag
    {
        fun(did);
        fun(value);
    }

    template<typename F>
    void enumerate(F& fun)
    {
        fun(did);
        fun(value);
    }

    bool operator == (const ::mdc::devm::DidInfoItem& t) const
    {
        return (did == t.did) && (value == t.value);
    }
};

struct PowerOffInfo {
    std::uint64_t utcTime {0U}; // 单位ms
    std::uint8_t  reason {0U};

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun) const // duplication code with diag
    {
        fun(utcTime);
        fun(reason);
    }

    template<typename F>
    void enumerate(F& fun)
    {
        fun(utcTime);
        fun(reason);
    }

    bool operator == (const ::mdc::devm::PowerOffInfo& t) const
    {
        return (utcTime == t.utcTime) && (reason == t.reason);
    }
};

struct BoardInfoCfg {
    std::uint32_t   boardId {0U};
    std::uint32_t   slotId {0U};
    std::uint32_t   pcbId {0U};
    std::uint32_t   bomId {0U};
    std::uint32_t   phId {0U};
    std::uint32_t   hkId {0U};
};
}
}
#endif