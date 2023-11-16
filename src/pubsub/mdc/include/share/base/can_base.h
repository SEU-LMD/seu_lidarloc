#ifndef CAN_BASE_H
#define CAN_BASE_H

#include <shared_mutex>
#include "mdc/mdccanrx/canrxserviceinterface_proxy.h"
#include "mdc/mdccan/impl_type_canfdbusdataparams.h"
#include "core/types.h"
#include "sa_types.h"
#include "data_receive_base_v1.h"

using namespace Adsfi;

namespace shared {
class CanBase : public DataReceiveBaseV1<mdc::mdccanrx::proxy::CanRxServiceInterfaceProxy,
                                         mdc::mdccanrx::proxy::CanRxServiceInterfaceProxy::HandleType, HafCanRxFrame> {
public:
    explicit CanBase(const uint32_t instanceID) : DataReceiveBaseV1(instanceID){};
    void RegisterHandle() override;
    void OnDataReceiveS();
    void OnDataReceiveL();
    virtual ~CanBase();
    void SendingData();

    void Stop()
    {
        DataReceiveBaseV1::Stop();
        sendCv_.notify_all();
    }
    HafStatus SendOneData(std::shared_ptr<HafCanTxFrame>& data)
    {
        if (data == nullptr) {
            HAF_LOG_ERROR << "The data ptr for sending is nullptr";
            return HAF_ERROR;
        }
        if (proxyPtr_ == nullptr) {
            HAF_LOG_ERROR << "proxy_ptr_ is nullptr";
            return HAF_ERROR;
        }
        std::lock_guard<std::mutex> lk(sendMtx_);
        sendDataContainer_.Push(data);
        sendCv_.notify_one();
        return HAF_SUCCESS;
    }

private:
    std::mutex sendMtx_;
    std::condition_variable sendCv_;
    ThreadSafeStack<std::shared_ptr<HafCanTxFrame>> sendDataContainer_;
};
}  // namespace shared
#endif
