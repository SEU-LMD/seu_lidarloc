
#ifndef FUSION_LOCATION_RESULT_SEND_BASE_H
#define FUSION_LOCATION_RESULT_SEND_BASE_H

#include <shared_mutex>
#include "data_send_base_v1.h"
#include "core/types.h"
#include "location/location.h"
#include "mdc/location/locationserviceinterface_skeleton.h"
#include "sa_types.h"

namespace Adsfi {

class LocationSendBase
    : public DataSendBaseV1<mdc::location::skeleton::LocationServiceInterfaceSkeleton,
                        HafLocationV2> {
public:
    explicit LocationSendBase(const uint32_t instanceIdxIn):DataSendBaseV1(instanceIdxIn){};
    virtual ~LocationSendBase();
    void SendingData() override;
};

}

#endif
