//
// Created by slam on 23-10-24.
//

#ifndef REBUILD_UDP_CLIENT_H
#define REBUILD_UDP_CLIENT_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>

class UDP_CLENT{

public:
    int client_sock;
    const char* server_ip;
    char rev_buff[1024];
    std::string rcv_msg;
    struct sockaddr_in server_addr,client_addr;



    void init(const int &port){
        client_sock = socket(PF_INET, SOCK_DGRAM, 0);
        if(client_sock==-1){
            std::cout<<"udp server erro"<<std::endl;
        }
        memset(&client_addr, 0, sizeof(client_addr));  //每个字节都用0填充

        client_addr.sin_family = AF_INET;
        client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        client_addr.sin_port=htons(port);
        if(bind(client_sock,(struct sockaddr*)&client_addr,sizeof (client_addr))==-1){
            std::cout<<"bind erro!"<<std::endl;
        }
        memset(&server_addr, 0, sizeof(server_addr));//每个字节都用0填充
    }

    void recvProcess(){
        socklen_t addrLen=sizeof (server_addr);
        //block mode!!!!
        int recv_len = recvfrom(client_sock, rev_buff,sizeof (rev_buff), 0,(struct sockaddr*)&server_addr,&addrLen);
        std::cout<<"ok"<<recv_len<<std::endl;
        if (recv_len > 0){
            rev_buff[recv_len] = '\0';
            std::string rcv_msgs(rev_buff);
            rcv_msg=rcv_msgs;
        }
    }

    ~UDP_CLENT(){
        close(client_sock);
    }

};








#endif //REBUILD_UDP_CLIENT_H