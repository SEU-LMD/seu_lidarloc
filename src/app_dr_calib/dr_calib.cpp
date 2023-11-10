//
// Created by fyy on 23-9-9.
//

#include "utils/config_helper.h"
#include "easylogging++.h"
#include "./dr_calib_manager.h"
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

    pubsub->initPubSub(argc, argv, "dr_calib");

    //3.初始化配置参数
    Load_Sensor_YAML("./config/sensor.yaml");

    //4.启动多个线程
    DRCalibManager dr_calib_manager;
    dr_calib_manager.Init(pubsub);
