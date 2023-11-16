#ifndef LOCATION_RECEIVE_BASE_H
#define LOCATION_RECEIVE_BASE_H

#include <shared_mutex>
#include "mdc/location/locationserviceinterface_proxy.h"
#include "data_receive_base_v1.h"
#include "core/types.h"
#include "location/location.h"
#include "sa_types.h"

namespace Adsfi {

class LocationReceiveBase: public DataReceiveBaseV1<mdc::location::proxy::LocationServiceInterfaceProxy,
    mdc::location::proxy::LocationServiceInterfaceProxy::HandleType, HafLocationV2> {
public:
    explicit LocationReceiveBase(const uint32_t instanceID)
        : DataReceiveBaseV1(instanceID) {};
    virtual ~LocationReceiveBase();
    void RegisterHandle() override;
    void OnDataReceive();
};

}
#endif
