//
// Created by slam on 23-10-21.
//

#ifndef SEU_LIDARLOC_UTP_THREAD_H
#define SEU_LIDARLOC_UTP_THREAD_H

#include <thread>
#include <string>
//udp 通信部分
#include <sys/types.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <mutex>
#include <cstring>
#include <deque>
//#include "udp_helper_h"

class UDP_THREAD{

public:


    std::thread udp_thread;
    std::mutex udp_mutex;
    std::deque<std::string> send_msgs;
    const char*  server_ip;
    const char*  client_ip;
    int server_port;
    int client_port;
    int serv_sock;
    struct sockaddr_in serv_addr,client_addr;
    std::string send_msg_test;//just test

    const int BUFFUR_SIZE=1024;

    void DoWork(){
        while(1){
            std::string send_msg;
            bool msg_ready = false;
            ssize_t data_size;
            udp_mutex.lock();
            if(send_msgs.size()!=0){
                send_msg = send_msgs.front();
                msg_ready = true;
                send_msgs.pop_front();
            }
            udp_mutex.unlock();

            if(msg_ready){
                char buffer[BUFFUR_SIZE];
                std::strcpy(buffer,send_msg.c_str());
                sendto(serv_sock,buffer,std::strlen(buffer),0,(struct sockaddr*)&client_addr,sizeof(client_addr));
            }
            else{
                sleep(0.01);
            }
        }
    }



    void init(const std::string &ip ,const int &port,const int &servport ){
        serv_sock = socket(AF_INET, SOCK_DGRAM, 0);


        client_ip = ip.c_str();
        client_port = port;
        server_port=servport;
        memset(&serv_addr, 0, sizeof(serv_addr));  //每个字节都用0填充
        serv_addr.sin_family = AF_INET;  //使用IPv4地址 (地址族，IPv4 or IPv6：AF_INET6)
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);  //具体的32位IP地址，inet_addr将一个IP字符串转化为一个网络字节序的整数值,反向操作inet_ntoa()
        serv_addr.sin_port = htons(server_port);  //端口号
        bind(serv_sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr));

        memset(&client_addr,0,sizeof (client_addr));
        client_addr.sin_family = AF_INET;  //使用IPv4地址 (地址族，IPv4 or IPv6：AF_INET6)
        client_addr.sin_addr.s_addr = inet_addr(client_ip);  //具体的32位IP地址，inet_addr将一个IP字符串转化为一个网络字节序的整数值,反向操作inet_ntoa()
        client_addr.sin_port = htons(client_port);  //端口号
        udp_thread = std::thread(&UDP_THREAD::DoWork,this);

    }

    void SendUdpMSg(const std::string& msg){
        udp_mutex.lock();
        send_msgs.push_back(msg);
        udp_mutex.unlock();
    }
    ~UDP_THREAD(){
        close(serv_sock);
    }
};
#endif //SEU_LIDARLOC_UTP_THREAD_H
