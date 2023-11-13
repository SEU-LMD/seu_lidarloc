/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: the declaration of instanceSpecifier class according to AutoSAR standard core type
 * Create: 2020-03-21
 */
#ifndef ARA_CORE_INSTANCE_SPECIFIER_H
#define ARA_CORE_INSTANCE_SPECIFIER_H

#include "ara/core/string_view.h"
#include "ara/core/result.h"
#include "ara/core/string.h"
#ifdef AOS_TAINT
#ifndef COVERITY_TAINT_SET_DEFINITION
#define COVERITY_TAINT_SET_DEFINITION
/**
 * @brief Function for Stain Modeling
 * @details The function is used only when the compilation macro AOS_TAINT is enabled.
 */
static void Coverity_Tainted_Set(void *buf){}
#endif
#endif
namespace ara {
namespace core {
/**
 * @brief class representing an AUTOSAR Instance Specifier, which is basically an AUTOSAR shortname-path wrapper.
 *
 */
class InstanceSpecifier final {
public:
    /**
     * @brief Construct a new Instance Specifier object
     *
     * @param metaModelIdentifier stringified meta model identifier (short name path) where path separator is ’/’.
     */
    explicit InstanceSpecifier(StringView metaModelIdentifier)
    {
#ifdef AOS_TAINT
    Coverity_Tainted_Set((void *)&metaModelIdentifier);
#endif
        if (metaModelIdentifier.find("/") == String::npos) {
#ifndef NOT_SUPPORT_EXCEPTIONS
        throw Exception(ErrorCode(CoreErrc::kInvalidMetaModelPath));
#endif
        } else {
            bool validFlag = true;
            for (auto c : metaModelIdentifier) {
                bool checkRst = ((c >= '0') && (c <= '9')) || ((c >= 'a') && (c <= 'z')) ||
                                ((c >= 'A') && (c <= 'Z')) || (c == '/') || (c == '_');
                if (!checkRst) {
                    validFlag = false;
#ifndef NOT_SUPPORT_EXCEPTIONS
                    throw Exception(ErrorCode(CoreErrc::kInvalidMetaModelShortname));
#else
                    break;
#endif
                }
            }
            if (validFlag) {
                myIndentifier_ = metaModelIdentifier;
            }
        }
    }

    /**
     * @brief Destroy the Instance Specifier object
     *
     */
    ~InstanceSpecifier () noexcept;

    /**
     * @brief Create a new instance of this class.
     *
     * @param metaModelIdentifier stringified form of InstanceSpecifier
     * @return Result<InstanceSpecifier> a Result, containing either a syntactically valid InstanceSpecifier,
     *                                   or an ErrorCode
     */
    static Result<InstanceSpecifier> Create(StringView metaModelIdentifier);

    /**
     * @brief eq operator to compare with other InstanceSpecifier instance.
     *
     * @param other InstanceSpecifier instance to compare this one with.
     * @return bool true in case both InstanceSpecifiers are denoting exactly the same model element, false else.
     */
    bool operator== (InstanceSpecifier const &other) const noexcept;

    /**
     * @brief eq operator to compare with other InstanceSpecifier instance.
     *
     * @param other string representation to compare this one with.
     * @return bool true in case this InstanceSpecifiers is denoting exactly the same model element as other,
     *              false else.
     */
    bool operator== (StringView other) const noexcept;

    /**
     * @brief uneq operator to compare with other InstanceSpecifier instance.
     *
     * @param other InstanceSpecifier instance to compare this one with.
     * @return bool false in case both InstanceSpecifiers are denoting exactly the same model element, true else.
     */
    bool operator!= (InstanceSpecifier const &other) const noexcept;

    /**
     * @brief uneq operator to compare with other InstanceSpecifier string representation.
     *
     * @param other string representation to compare this one with.
     * @return bool false in case this InstanceSpecifiers is denoting exactly the same model element as other,
     *              true else.
     */
    bool operator!= (StringView other) const noexcept;

    /**
     * @brief lower than operator to compare with other InstanceSpecifier for ordering purposes.
     *
     * @param other InstanceSpecifier instance to compare this one with.
     * @return bool true in case this InstanceSpecifiers is lexically lower than other, false else.
     */
    bool operator< (InstanceSpecifier const &other) const noexcept;

    /**
     * @brief method to return the stringified form of InstanceSpecifier.
     *
     * @return StringView stringified form of InstanceSpecifier.
     */
    StringView ToString () const noexcept;
private:
    StringView myIndentifier_;
};
} // End of namespace core
} // End of namespace ara

#endif
