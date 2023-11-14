/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */

#ifndef SOMEIP_SSLINTERFACE_H
#define SOMEIP_SSLINTERFACE_H
#include <cstdint>
#include <sys/socket.h>

namespace Someip {
enum class SSLError : std::int32_t {
    SSL_OK,                    // not an error
    SSL_WANT_READ,             // not an error, need more read
    SSL_WANT_WRITE,            // not an error, need more write
    SSL_SYSCALL_ERROR,         // posix socket error
    SSL_IGNORE,                // ignore this error, do nothing
    SSL_CLOSE_CONNECTION       // close this connection, can not recover
};

class SSLInterface {
public:
    virtual ~SSLInterface() = default;
    virtual bool Enabled() const = 0;
    virtual int Socket() const = 0;
    virtual int HandShake(SSLError &err) const = 0;
    virtual int Read(void *buf, int num, SSLError &err) const = 0;
    virtual int Write(const void *buf, int num, SSLError &err) const = 0;
    virtual int Listen(struct sockaddr *clientAddr) const { return -1; }
    virtual int Accept(std::int32_t connfd, struct sockaddr *clientAddr) { return -1; }
};
}

#endif // SOMEIP_SSLINTERFACE_H

