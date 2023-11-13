/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: 设备管理客户端对外相关接口定义
 * Author: o00288819
 * Create: 2020-03-04
 */
#ifndef DEVM_CLIENT_H
#define DEVM_CLIENT_H

#include <memory>
#include <atomic>
#include "devm_data_define.h"

namespace mdc {
namespace devm {
class DevmClientImpl;
class DevmClient {
    DevmClient();
    ~DevmClient();
public:
    static DevmClient& GetInstance()
    {
        static DevmClient devmClient;
        return devmClient;
    }

    std::int32_t Init();
    void SetLogHanlder(const LogHandler devmLogHandler);
    std::int32_t GetDeviceState(const String& devName, std::uint8_t& status);
    std::int32_t GetState(std::uint8_t& status);

    /*
     * 函数功能    部署模式读取
     * 输入说明    std::uint8_t mode: 0 单盒
                                     1 双盒
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t GetDeployMode(std::uint8_t& mode);
    std::int32_t SetDeployMode(const std::uint8_t mode);

    /*
     * 函数功能    主备模式读取
     * 输入说明    std::uint8_t mode: 0 主
                                     1 从
     *            std::uint8_t type: 0 当前主从模式
                                     1 初始主从模式
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t GetMasterMode(std::uint8_t& mode, const std::uint8_t type);
    std::int32_t SetMasterMode(const std::uint8_t mode, const std::uint8_t type);

    /*
    函数功能    获取设备的信息
    输入说明    String& devName：设备名称
    输出说明    MdcDeviceInfo& devInfo  设备信息
    返回值说明  0   成功，其他值失败
    */
    std::int32_t GetDevInfo(const String& devName, MdcDeviceInfo& mdcDevInfo);

    /*
    函数功能    获取设备的信息
    输入说明    String& devName：设备名称
    输出说明    DevmUpdateInfo& devmUpdateInfo  设备升级信息
    返回值说明  0   成功，其他值失败
    */
    std::int32_t GetUpgradeInfo(const String& devName, DevmUpdateInfo& devmUpdateInfo);

    /*
    函数功能    获取设备的信息
    输入说明    无
    输出说明    Vector<MdcDevObj>& devList  设备列表
    返回值说明  0   成功，其他值失败
    */
    std::int32_t GetDeviceList(Vector<MdcDevObj>& devList);

     /*
    函数功能    获取设备的信息
    输入说明    无
    输出说明    Vector<MdcDevObj>& devList  升级设备列表
    返回值说明  0   成功，其他值失败
    */
    std::int32_t GetUpgradableDeviceList(Vector<UpgradableDevObj>& devList);

    /*
     * 函数功能    升级设备
     * 输入说明    String& devName：设备名称
     *            String& packgName 升级包名
     *            String& filePath 升级包路径
     * 返回值说明  0   成功，其他值失败
     */
    std::int32_t Update(const String& devName, const String& pkgName, const String& filePath);

    /*
     * 函数功能    获取设备的升级状态
     * 输入说明    String& devName：设备名称
     *            String& packgName 升级包名
     * 输出说明    std::uint8_t& progress 升级进度
     *            std::int32_t& errCode  升级状态
     * 返回值说明  0   成功，其他值失败
     */
    std::int32_t GetUpdateState(const String& devName, const String& pkgName,
        std::uint8_t& progress, std::int32_t& errCode);
    std::int32_t UpdateSyncStart(const String& devName, const String& pkgName);
    std::int32_t SetSensorAction(const Vector<String>& devNameList, const std::uint8_t action);
    std::int32_t SetSystemAction(const std::uint8_t action);
    /*
     * 函数功能    设置系统操作
     * 输入说明
     * std::uint8_t type：操作对象
     * 0：SocA
     * 1: SocB (单芯场景不支持)
     * 3-6：预留位
     * 7：MCU
     * 8：整Soc
     * 9：整系统（整Soc+Mcu）
     * std::uint8_t action：具体操作
     * 0：休眠（整系统进入低功耗，此时type参数不生效）
     * 1：下电
     * 2: 热复位
     * 3：冷复位
     * 返回值说明  0   成功，其他值失败
     */
    std::int32_t DoSystemAction(const std::uint8_t type, const std::uint8_t action);
    String GetTemperature();
    String GetVoltage();
    String GetFanduty();
    String GetCpuUsage();
    String GetMemUsage();
    std::int32_t GetStatisticsInfo(const String& devName, String& statisticsInfo);
    std::int32_t SetConfig(const String& devName, const Vector<MdcConfigItem>& configItems);
    Vector<std::uint8_t> GetConfig(const String& devName, const String& cfgKey);
    /*
     * 函数功能    读写did
     * 输入说明    String& devName：设备名称
     *             Vector<std::uint8_t> data: 读写内容
     *             std::uint8_t opNum: 0 read
     *                                 1 write
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t DidOperate(const String& devName, const std::uint16_t did,
        const std::uint8_t opNum, Vector<std::uint8_t>& data);
    /*
     * 函数功能    获取电子标签（参考《华为公司电子标签规范V4.01 第三部分》）
     * 输入说明    String& item：获取项目
     *                Raw 获取整块原始电子标签（透传未解析）
     *                BoardType 对内型号
     *                BarCode SN条码
     *                Item BBOM编码
     *                Description 英文描述
     *                Manufactured 生产日期
     *                VendorName 厂家名称
     *                IssueNumber 发行号
     *                CLEICode CLEI码
     *                BOM BOM编码
     *                Model 对外型号
     *                ExInfo 扩展区内容
     * 输出说明    String& value 获取结果
     * 返回值说明  0   成功，其他值失败
     */
    std::int32_t GetElectronicLable(const String& item, String& value);
    /*
     * 函数功能    获取MDC BoardInfo信息
     * 输入说明    无
     * 输出说明    BoardInfoCfg& boardInfo
     *             boardId            slotId                            pcbId    bomId    phId    hkId
     *   Pro A板     0          0(paln A)/1(plan B)                       NA       NA       NA     0
     *   Pro R板     1          0(paln A)/1(plan B)                       NA       NA       NA     1
     *   Pro C1板    2          0(paln A)/1(plan B)                        0       NA        0     2
     *   Pro C2板    3          0(paln A)/1(plan B)                        0       NA        0     3
     *   610 C板     0                   0                                 0       NA       16     55
     *   300         1  0(mini0)/1(mini1)/2(mini2)/3(mini3),host(other)   NA       NA       NA     NA
     *   300F        3  0(mini0)/1(mini1)/2(mini2)/3(mini3),host(other)   NA       NA       NA     NA
     * 返回值说明  0   成功，其他值失败
     */
    std::int32_t GetBoardInfo(BoardInfoCfg& boardInfo);
    std::int32_t SetWorkMode(const std::uint8_t& workMode);
    std::int32_t GetWorkMode(std::uint8_t& workMode);

    std::int32_t SetCanControlConfig(const DevmCtrlConfig& ctrlConfig); // 新接口，OM切换

    std::int32_t SetSoftVersion(const String& softVersion);
    String GetSoftVersion();
    String GetDevAttribute(const String& devName, const DevmAttr& param);

    std::int32_t SetLicenseFile(const Vector<std::uint8_t>& licenseFile);
    std::int32_t SetPublicKey(const Vector<std::uint8_t>& publicKey);
    std::int32_t GetLicenseFile(Vector<std::uint8_t>& licenseFile);
    std::int32_t GetPublicKey(Vector<std::uint8_t>& publicKey);

    /*
     * 函数功能    传感器消息代理
     * 输入说明    String& devName：设备名称
     *             MdcSensorMsg& reqMsg: 发送信息
     *             MdcSensorMsg& respMsg: 接受信息, msg id 大于 0x7FF，则只发送信息，不接收
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t SensorMsgProxy(const String& devName, const MdcSensorMsg& reqMsg, MdcSensorMsg& respMsg);
    /*
     * 函数功能    设置系统从浅休眠进入深度休眠的时间
     * 输入说明    std::uint8_t time 进入深度休眠的时间，单位小时，范围[1-240]
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t SetEnterDeepSleepTime(const std::uint8_t time);

    /*
     * 函数功能    设置CAMERA故障处理回调函数
     * 输入说明    CameraHandler: std::uint32_t(Vector<std::uint8_t>, Vector<std::uint8_t>)
     * 返回值说明  0  成功，其他值失败
     */
    static std::int32_t SetCameraFaultNotifyHandler(const CameraFaultNotifyHandler handler);

    /*
     * 函数功能    查询MCU异常下电原因，下电时间(ms)
     * 输入说明    PowerOffInfo
     * 返回值说明  0  查询成功，其他值失败
     */
    std::int32_t GetSysPowerOffInfo(PowerOffInfo& powerOffInfo);

    /*
     * 函数功能    标记已读取MCU异常下电原因
     * 输入说明    无
     * 返回值说明  0  标记成功，其他值失败
     */
    std::int32_t SetSysPowerOffInfoReaded();

    /*
     * 函数功能    打开系统硬件狗，只能打开，不支持关闭
     * 输入说明    timeout, 狗超时时间，单位秒
     * 返回值说明  0  标记成功，其他值失败
     */
    std::int32_t OpenWatchDog(const std::uint32_t timeout);

    /*
     * 函数功能    设置MCU Recovery状态
     * 输入说明    state ： 状态值
     * 返回值说明  0  标记成功，其他值失败
     */
    std::int32_t SetMcuRecoveryState(const std::uint8_t state);

    /*
     * 函数功能    获取ufs容量
     * 输入说明    capacity ： 容量值，单位 512byte
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t GetUfsCapacity(std::uint32_t& capacity);

    /*
     * 函数功能    校验分区一致性
     * 输入说明    configFile ： 配置文件全路径
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t VerifyDiskPartition(const std::string& configFile, std::int32_t& result);

    /*
     * 函数功能    设置/清除 recovery标识，获取进入recovery的方式
     * 输入说明    operateType ：
                        0：设置recovery标识
                        1：获取进入recovery的方式recoveryType (0 - BISO启动异常进入  1 - 调用接口强制进入  other: 非法)
                        2：清除recovery标识
     * 返回值说明  0  标记成功，其他值失败
     */
    std::int32_t RecoveryFlagOperate(const std::uint8_t operateType, std::uint32_t& recoveryType);

    /*
     * 函数功能    重置BIOS recovery启动计数
     * 输入说明    无
     * 返回值说明  0  成功，其他值失败
     */
    std::int32_t ResetRecoveryCount();

    std::int32_t SetMcuSystemStatus(const std::uint8_t type);

    /*
     * 函数功能    先打开系统硬件狗，后清除BIOS启动计数 （喂狗失败仍会清除计数）
     * 输入说明    无
     * 返回值说明  0  喂狗及清计数成功，其他值失败
     */
    std::int32_t SetDevReady();

private:
    static std::int32_t ParseElectronicLable(const String& eLable, const String& item, String& value);
private:
    std::unique_ptr<DevmClientImpl> m_devmImpl;
    std::atomic<uint32_t> m_callCnt {};
};
}
}
#endif
