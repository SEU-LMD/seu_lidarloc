/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2020. All rights reserved.
 * Description: The declarations of E2EXf_ConfigIndexImpl
 * Create: 2019-06-17
 */
/**
* @file
*
* @brief The declarations of E2EXf_ConfigIndexImpl
*/
#ifndef E2EXF_CONFIG_INDEX_IMPL_H
#define E2EXF_CONFIG_INDEX_IMPL_H

#include <array>
#include <cstdint>
#include <string>

namespace vrtf {
namespace com {
namespace e2e {
constexpr std::uint8_t DATAIDLIST_LENGTH {16U};
/* AXIVION Next Line AutosarC++19_03-M0.1.4, AutosarC++19_03-A2.10.5: Convert user configuration to c var */
constexpr std::uint32_t UNDEFINED_HEADER_SIZE {0xFFFEU};
/* AXIVION Next Line AutosarC++19_03-A2.10.5: Convert user configuration to c var */
constexpr std::uint32_t UNDEFINED_DATAID {0xFFFFFFFEU};
namespace impl {
/**
 * @brief Mapping DataID or DataIDList for each configuration.
 * @req{ AR-iAOS-E2EXf_Config-00003,
 * E2EXf_Config shall provide E2E ConfigIndex class,
 * ASIL-D,
 * DR-iAOS-RTF-RTFE2E-00005,
 */
class E2EXf_ConfigIndexImpl final {
public:
    E2EXf_ConfigIndexImpl() = delete;
    ~E2EXf_ConfigIndexImpl() = default;
    E2EXf_ConfigIndexImpl(const E2EXf_ConfigIndexImpl&) = default;
    E2EXf_ConfigIndexImpl& operator = (const E2EXf_ConfigIndexImpl&) = delete;
    explicit E2EXf_ConfigIndexImpl(const std::uint32_t& DataID, const std::string& NetworkName = "");
    explicit E2EXf_ConfigIndexImpl(const std::array <std::uint8_t, DATAIDLIST_LENGTH>& DataIDList,
                                   const std::string& NetworkName = "");
    /**
     * @brief Set DataID for each configuration.
     * @param[in] DataID DataID for mapping configuration.
     * @req{ AR-iAOS-E2EXf_Config-00003,
     * E2EXf_Config shall provide E2E ConfigIndex class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     */
    void SetDataID(const std::uint32_t& DataID)
    {
        DataID_ = DataID;
        IsUsingDataID_ = true;
    }
    /**
     * @brief Set DataIDList for each configuration.
     * @param[in] DataIDList DataIDList for mapping configuration.
     * @req{ AR-iAOS-E2EXf_Config-00003,
     * E2EXf_Config shall provide E2E ConfigIndex class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     */
    void SetDataIDList(const std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList);
    /**
     * @brief Get DataIDList for this configuration.
     * @return std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList.
     * @req{ AR-iAOS-E2EXf_Config-00003,
     * E2EXf_Config shall provide E2E ConfigIndex class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     */
    const std::array<std::uint8_t, DATAIDLIST_LENGTH>& GetDataIDList() const
    {
        return DataIDList_;
    }
    /**
     * @brief Get DataID for this configuration.
     * @return std::uint32_t DataID.
     * @req{ AR-iAOS-E2EXf_Config-00003,
     * E2EXf_Config shall provide E2E ConfigIndex class,
     * ASIL-D,
     * DR-iAOS-RTF-RTFE2E-00005,
     */
    const std::uint32_t& GetDataID() const
    {
        return DataID_;
    }

    std::string GetNetworkName() const
    {
        return NetworkName_;
    }
    bool operator<(const E2EXf_ConfigIndexImpl& Index) const;
    explicit operator bool() const;
private:
    std::uint32_t DataID_;
    std::array<std::uint8_t, DATAIDLIST_LENGTH> DataIDList_;
    bool IsUsingDataID_;
    std::string NetworkName_;

    E2EXf_ConfigIndexImpl(const std::uint32_t& DataID, const std::array<std::uint8_t, DATAIDLIST_LENGTH>& DataIDList,
                          bool IsUsingDataID, const std::string& NetworkName = "")
        : DataID_(DataID),
          DataIDList_ {DataIDList},
          IsUsingDataID_ {IsUsingDataID},
          NetworkName_ {NetworkName}
    {
    }
};
} /* End namespace impl */
} /* End namesapce e2e */
} /* End namesapce com */
} /* End namespace vrtf */
#endif
