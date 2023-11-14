/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2019. All rights reserved.
 * Description: dds_cpp_infrastruture.h
 */

#ifndef RT_DDS_DDS_CPP_INFRASTRUCTURE_H
#define RT_DDS_DDS_CPP_INFRASTRUCTURE_H

#include "RT-DDS/dds_cpp/dds_cpp_fooseq.h"

class DDS_Listener {
public:
    virtual ~DDS_Listener() = default;
};

class DDS_ConditionImpl;

class DDS_Condition {
public:
    /**
     * @brief Get DDS_ConditionImpl pointer.
     * @return the DDS_ConditionImpl pointer.
     */
    virtual DDS_ConditionImpl *GetImplConditionI() = 0;

    /**
     * @brief Retrieve the trigger value.
     * @return the trigger value.
     */
    virtual DDS_Boolean GetTriggerValue() = 0;

protected:
    virtual ~DDS_Condition()
    {}
};

DDS_SEQUENCE(DDS_ConditionSeq, DDS_Condition *);

class DDS_DomainParticipantFactory;

class DDS_Entity;

class DDS_WaitSetImpl;

class DDS_StatusCondition : virtual public DDS_Condition {
public:
    /**
     * @brief Get the list of statuses enabled on an DDS_Entity.
     * @return list of enabled statuses.
     */
    virtual DDS_StatusMask GetEnabledStatuses() = 0;

    /**
     * @brief This operation defines the list of communication statuses that
     *        determine the trigger_value of the DDS_StatusCondition.
     * @param mask The list of enables statuses.
     * @return Standard DDS_ReturnCode.
     * @retval DDS_RETCODE_OK
     */
    virtual DDS_ReturnCode SetEnabledStatuses(DDS_StatusMask mask) = 0;

    /**
     * @brief Get the DDS_Entity associated with the DDS_StatusCondition.
     * @return DDS_Entity associated with the DDS_StatusCondition.
     */
    virtual DDS_Entity *GetEntity() = 0;

protected:
    ~DDS_StatusCondition() override = default;
};

class DDS_GuardConditionImpl;

class DDS_GuardCondition : public DDS_Condition {
public:
    /**
     * @brief Constructor.
     * @details No argument constructor.<br/>
     *          The default constructor initializes the guard condition with
     *          trigger value DDS_BOOLEAN_FALSE.
     */
    DDS_GuardCondition();

    /**
     * @brief Destructor.
     */
    ~DDS_GuardCondition() override;

    /**
     * @brief Get DDS_ConditionImpl pointer.
     * @return the DDS_ConditionImpl pointer.
     */
    DDS_ConditionImpl *GetImplConditionI() override;

    /**
     * @brief Get the trigger value.
     * @return the trigger value.
     */
    DDS_Boolean GetTriggerValue() override;

    /**
     * @brief Set the trigger value.
     * @param[in] value the new trigger value.
     * @return DDS Standard Return Code.
     * @retval DDS_RETCODE_OK
     */
    DDS_ReturnCode SetTriggerValue(DDS_Boolean value);

private:
    DDS_GuardConditionImpl *conditionImpl_;
};

class DDS_WaitSet {
public:
    /**
     * @brief Constructor.
     */
    DDS_WaitSet();

    /**
     * @brief Destructor.
     */
    virtual ~DDS_WaitSet();

    /**
     * @brief Allows an application thread to wait for the occurrence of certain
     *        conditions.
     * @param[in,out] activeConditions A valid DDSConditionSeq object.
     * @param[in] timeout Wait timeout duration.
     * @return DDS Standard Return Code.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     * @retval DDS_RETCODE_TIMEOUT
     */
    virtual DDS_ReturnCode Wait(DDS_ConditionSeq &activeConditions, const DDS_Duration &timeout);

    /**
     * @brief Attaches a DDS_Condition to the DDS_WaitSet.
     * @param[in] condition Condition to be attached.
     * @return DDS Standard Return Code.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     */
    virtual DDS_ReturnCode AttachCondition(DDS_Condition *condition);

    /**
     * @brief Detaches a DDS_Condition from the DDS_WaitSet.
     * @param[in] condition Condition to be detached.
     * @return DDS Standard Return Code.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_BAD_PARAMETER
     */
    virtual DDS_ReturnCode DetachCondition(DDS_Condition *condition);

    /**
     * @brief Retrieves the list of attached DDS_Condition(s).
     * @param[in,out] attachedConditions a DDSConditionSeq object where the list
     *                                   of attached conditions will be returned.
     * @return DDS Standard Return Code.
     * @retval DDS_RETCODE_OK
     * @retval DDS_RETCODE_ERROR
     */
    virtual DDS_ReturnCode GetConditions(DDS_ConditionSeq &attachedConditions);

private:
    DDS_WaitSetImpl *wsImpl_;
};

class DDS_Entity {
public:
    /**
     * @brief Allows access to the DDS_StatusCondition associated with the
     *        DDS_Entity.
     * @return The status condition associated with this entity.
     */
    virtual DDS_StatusCondition *GetStatusCondition() = 0;

    /**
     * @brief Retrieves the list of communication statuses in the DDS_Entity
     *        that are triggered.
     * @return List of communication statuses in the DDSEntity that are
     *         triggered.
     */
    virtual DDS_StatusMask GetStatusChanges() = 0;

    /**
     * @brief Allows access to the DDS_InstanceHandle associated with the
     *        DDSEntity.
     * @return The instance handle associated with this entity.
     */
    virtual DDS_InstanceHandle GetInstanceHandle() = 0;

protected:
    virtual ~DDS_Entity()
    {}
};

#endif /* RT_DDS_DDS_CPP_INFRASTRUCTURE_H */

