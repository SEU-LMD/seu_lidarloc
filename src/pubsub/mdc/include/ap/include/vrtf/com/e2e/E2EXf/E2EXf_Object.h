/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of E2EXf_CM.h
 * Create: 2020-11-10
 */

#ifndef VRTF_COM_E2EXF_OBJECT_H
#define VRTF_COM_E2EXF_OBJECT_H

#include <string>
#include <array>
#include "vrtf/com/e2e/E2EXf/E2EXf_ConfigIndexImpl.h"

namespace vrtf {
namespace com {
namespace e2e {
class E2EXf_Object final {
public:
    /**
     * @brief Delete using default constructor
     *
     */
    E2EXf_Object() = delete;

    /**
     * @brief Construct a new e2exf object object
     *
     * @param[in] DataID        The dataId of corresponding E2E configuration
     * @param[in] NetworkName   The network name of the dataId belongs to
     */
    E2EXf_Object(std::uint32_t DataID, std::string NetworkName)
        : E2EXf_Object(DataID, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, true, 0, std::move(NetworkName)) {}

    /**
     * @brief Construct a new e2exf object object
     *
     * @param[in] DataIDList     The dataIdList of corresponding E2E configuration
     * @param[in] NetworkName    The network name of the dataIdList belongs to
     */
    E2EXf_Object(std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList, std::string NetworkName)
        : E2EXf_Object(UNDEFINED_DATAID, DataIDList, false, 0, std::move(NetworkName)) {}

    /**
     * @brief Construct a new e2exf object object
     *
     * @param[in] Other   the other instance
     */
    E2EXf_Object(const E2EXf_Object& Other) = default;

    /**
     * @brief Construct a new e2exf object object using assignment operator
     *
     * @param[in] Other   the other instance
     * @return E2EXf_Object&   a new e2exf object
     */
    E2EXf_Object& operator = (const E2EXf_Object& Other) & = default;

    /**
     * @brief Destroy the e2exf object object
     *
     */
    ~E2EXf_Object() = default;

    /**
     * @brief Get the DataID object
     *
     * @return std::uint32_t  the value of configured dataId
     */
    std::uint32_t GetDataID() const noexcept { return DataID_; }

    /**
     * @brief Get the DataID List object
     *
     * @return std::array<std::uint8_t, DATAIDLIST_LENGTH>  the value of configured dataIdList
     */
    std::array<std::uint8_t, DATAIDLIST_LENGTH> GetDataIDList() const noexcept { return DataIDList_; }

    /**
     * @brief  If it is using dataId
     *
     * @return bool
     *      @retval true    Using dataId
     *      @retval false   Using dataIdList
     */
    bool IsUsingDataID() const noexcept { return IsUsingDataID_; }

    /**
     * @brief Get the Protect Counter object will be used in protection or after protection
     *
     * @return std::uint32_t  The protect counter will be used or after protection
     */
    std::uint32_t GetProtectCounter() const noexcept { return Counter_; }

    /**
     * @brief Set the Protect Counter object will be used in protection, or after protection
     *
     * @param[in] Counter  The protect counter will be used in protection or after protection
     */
    void SetProtectCounter(std::uint32_t Counter) { Counter_ = Counter; }

    /**
     * @brief Get the Network Name object
     *
     * @return std::string      The network of the configured E2E Id
     */
    std::string GetNetworkName() const noexcept { return NetworkName_; }

    /**
     * @brief Set the Incorrect DataIDList object
     *
     * @param[in] Incorrect   The incorrect dataIdList will be used in protection
     */
    void SetIncorrectDataIDList(std::array<std::uint8_t, DATAIDLIST_LENGTH> Incorrect) { IncorrectIDList_ = Incorrect; }

    /**
     * @brief Get the Incorrect DataIDList object
     *
     * @return std::array<std::uint8_t, DATAIDLIST_LENGTH>   The incorrect dataIdList will be used in protection
     */
    std::array<std::uint8_t, DATAIDLIST_LENGTH> GetIncorrectDataIDList() const { return IncorrectIDList_; }

    /**
     * @brief Set the Incorrect DataID object
     *
     * @param[in] Incorrect  The incorrect dataIdList will be used in protection
     */
    void SetIncorrectDataID(std::uint32_t Incorrect) { IncorrectID_ = Incorrect; }

    /**
     * @brief Get the Incorrect DataID object
     *
     * @return std::uint32_t  The incorrect dataIdList will be used in protection
     */
    std::uint32_t GetIncorrectDataID() const { return IncorrectID_; }
private:
    std::uint32_t DataID_;
    std::uint32_t IncorrectID_;
    std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList_;
    std::array<std::uint8_t, DATAIDLIST_LENGTH> IncorrectIDList_;
    bool IsUsingDataID_;
    std::uint32_t Counter_;
    std::string NetworkName_;

    E2EXf_Object(std::uint32_t DataID, std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList,
             bool IsUsingDataID, std::uint32_t Counter, std::string NetworkName)
        : DataID_(DataID), IncorrectID_(DataID), DataIDList_(DataIDList), IncorrectIDList_(DataIDList),
          IsUsingDataID_(IsUsingDataID), Counter_(Counter), NetworkName_(NetworkName)
    {
    }
};
} /* End e2e namespace */
} /* End com namespace */
} /* End ara namespace */

#endif

