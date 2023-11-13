/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 */

#ifndef SOMEIP_SSLCTXINTERFACE_H
#define SOMEIP_SSLCTXINTERFACE_H

#include <memory>

namespace Someip {
class SSLInterface;
class SSLCTXInterface {
public:
    virtual ~SSLCTXInterface() = default;
    virtual std::unique_ptr<SSLInterface> CreateSSL(int socket, bool isClient) const = 0;
};
}

#endif // SOMEIP_SSLCTXINTERFACE_H
