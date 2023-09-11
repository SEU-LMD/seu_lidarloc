//
// Created by fyy on 23-9-9.
//

#include "utils/config_helper.h"
#include "easylogging++.h"
#include "./loc_manager.h"
#include "utils/filesys.h"
//选择中间件
//#ifdef X86
#include "pubsub/ros/ros_pubsub.h"
//#endif

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {

    CreateDirWithDelete("./log");

    //1.初始化 log
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::Format, "%level %file %line : %msg");
#ifdef ELPP_THREAD_SAFE
    EZLOG(INFO) << "easylogging++ thread safe!";
#else
    EZLOG(INFO) << "easylogging++ thread unsafe";
#endif

    //2.初始化中间件
    PubSubInterface* pubsub;
    //#ifdef X86
    pubsub = new ROSPubSub();
    //#endif

    pubsub->initPubSub(argc, argv, "mapping");

    //3.初始化配置参数
    Load_Sensor_YAML("./config/sensor.yaml");
    Load_Mapping_YAML("./config/mapping.yaml");

    //4.启动多个线程
    LocManager loc_manager;
    loc_manager.Init(pubsub);

    //5.设置mapping manager的回调函数
    auto cloud_callback = std::bind(&LocManager::CloudCallback, &loc_manager,std::placeholders::_1);
    auto gnssins_callback = std::bind(&LocManager::GNSSINSCallback, &loc_manager,std::placeholders::_1);

    pubsub->addSubscriber(SensorConfig::pointCloudTopic, DataType::LIDAR, cloud_callback);
    pubsub->addSubscriber(SensorConfig::gpsTopic, DataType::GNSS_INS, gnssins_callback);

    //开始运行程序
    pubsub->run();
}