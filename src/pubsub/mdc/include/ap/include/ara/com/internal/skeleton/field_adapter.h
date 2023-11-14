/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: This file provides an interface related to communication management.
 * Create: 2019-07-01
 */
#ifndef ARA_COM_SKELETON_FIELD_ADAPTER_H_
#define ARA_COM_SKELETON_FIELD_ADAPTER_H_
#include "ara/com/internal/skeleton/skeleton_adapter.h"
#include "ara/core/future.h"
namespace ara {
namespace com {
namespace internal {
namespace skeleton {
namespace field {
namespace impl {
// Internal class !!! Prohibit to use by Application!!!!
class FieldAdapterImpl {
public:
    FieldAdapterImpl(void)
    {
        logInstance_ = ara::godel::common::log::Log::GetLog("CM");
    }

    FieldAdapterImpl(const std::shared_ptr<vrtf::vcc::Skeleton>& skeleton, EntityId entityId)
        : skeleton_(skeleton), entityId_(entityId)
    {
        logInstance_ = ara::godel::common::log::Log::GetLog("CM");
    }

    virtual ~FieldAdapterImpl(void)
    {
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetEntityId(void) const
    {
        return entityId_;
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    void SetSetterEntityId(const EntityId& id)
    {
        setterEntityId_ = id;
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    void SetGetterEntityId(const EntityId& id)
    {
        getterEntityId_ = id;
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetSetterEntityId(void) const
    {
        return setterEntityId_;
    }

    // Internal interface!!! Prohibit to use by Application!!!!
    EntityId GetGetterEntityId(void) const
    {
        return getterEntityId_;
    }

protected:
        /**
     * @brief update field value
     * @details if update before offerservice, update initdata and store
     *          if update after offerservice, the data will be send by this interface
     *          filed not use plog to calculate time delay, pass a nullptr
     *
     * @param[in] data the data which will be sent
     */
    template<class SampleType>
    void Update(const SampleType& data)
    {
        std::shared_ptr<SampleInfoImpl> sampleInfo = std::make_shared<SampleInfoImpl>();
        sampleInfo->isEvent_ = false;
        skeleton_->Send<SampleType>(data, entityId_, sampleInfo);
    }
        /**
     * @brief register get callback
     * @details register get callback
     *
     * @param[in] getHandler the callback will be invoke when receive one get message
     */
    template<class SampleType>
    void RegisterGetHandler(std::function<ara::core::Future<SampleType>()> getHandler)
    {
        skeleton_->RegisterGetMethod<SampleType>(getHandler, entityId_);
    }
        /**
     * @brief register set callback
     * @details register set callback
     *
     * @param[in] setHandler the callback will be invoke when receive one set message
     */
    template<class SampleType>
    void RegisterSetHandler(std::function<ara::core::Future<SampleType>(const SampleType& value)> setHandler)
    {
        skeleton_->RegisterSetMethod<SampleType>(setHandler, entityId_);
    }

    /**
     * @brief Verify the validity of the field
     * @details 1. To ensure the existence of valid values for these fields:
     *              1) hasNotifier = true;
     *              2) hasGetter = true and a GetHandler has not yet been registered.
     *          2. To ensure the existence of setHandler if hasSetter == true.
     *          If the above conditions are not met, the program will abort.
     */
    template<class SampleType>
    void VerifyValidity()
    {
        // get field info from VCC
        std::shared_ptr<vrtf::vcc::api::types::FieldInfo> info;
        skeleton_->QueryFieldInfo(entityId_, info);

        std::function<ara::core::Future<SampleType>(void)> getHandler = nullptr;
        std::function<ara::core::Future<SampleType>(const SampleType& value)> setHandler = nullptr;
        skeleton_->QueryGetMethod<SampleType>(entityId_, getHandler);
        skeleton_->QuerySetMethod<SampleType>(entityId_, setHandler);

        bool isInitialized = skeleton_->IsFieldInitialized<SampleType>(entityId_);
        if (info == nullptr) {
            ara::core::Abort("Not find field Info!");
        }
        // [SWS_CM_00128]Verify the existence of valid values
        if (info->IsHasNotifier() || (info->IsHasGetter() && getHandler == nullptr)) {
            if (!isInitialized) {
                ara::core::Abort("No init data for the field!");
            }
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose()<< "Verify valid values successfully for the field: " << entityId_;

        // [SWS_CM_00128]Verify the existence of SetHandler
        if (info->IsHasSetter() && setHandler == nullptr) {
            ara::core::Abort("No setHandler for the field!");
        }
        /* AXIVION Next Line AutosarC++19_03-A5.1.1, AutosarC++19_03-A5.0.1 : Records the log */
        logInstance_->verbose()<< "Verify SetHandler successfully for the field: " << entityId_;
        skeleton_->RegisterFieldHandler<SampleType>(info);
    }

    /**
     * @brief Set field to uninitialized state
     * @details 1. Set field isInitialized_ to false
     */
    template<class SampleType>
    void ResetInitState()
    {
        skeleton_->ResetInitState<SampleType>(entityId_);
    }

private:
    std::shared_ptr<vrtf::vcc::Skeleton> skeleton_;
    EntityId entityId_ = UNDEFINED_ENTITYID;
    EntityId setterEntityId_ = UNDEFINED_ENTITYID;
    EntityId getterEntityId_ = UNDEFINED_ENTITYID;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}

template<class SampleType>
class FieldAdapter : public impl::FieldAdapterImpl {
public:
    using value_type = typename std::decay<SampleType>::type;
    FieldAdapter(const std::shared_ptr<vrtf::vcc::Skeleton>& skeleton, EntityId entityId)
        : FieldAdapterImpl(skeleton, entityId)
    {
    }

    FieldAdapter(void)
    {
    }
    ~FieldAdapter() = default;
    void Update(const SampleType& data)
    {
        FieldAdapterImpl::Update<SampleType>(data);
    }

    void RegisterGetHandler(std::function<ara::core::Future<SampleType>()> getHandler)
    {
        FieldAdapterImpl::RegisterGetHandler<SampleType>(getHandler);
    }

    void RegisterSetHandler(std::function<ara::core::Future<SampleType>(const SampleType& value)> setHandler)
    {
        FieldAdapterImpl::RegisterSetHandler<SampleType>(setHandler);
    }

    void VerifyValidity()
    {
        FieldAdapterImpl::VerifyValidity<SampleType>();
    }

    void ResetInitState()
    {
        FieldAdapterImpl::ResetInitState<SampleType>();
    }
};
}
}
}
}
}

#endif
