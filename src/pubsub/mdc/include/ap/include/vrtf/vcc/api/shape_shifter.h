/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of ShapeShifter
 * Create: 2020-12-02
 */
#ifndef RTF_COM_SHAPE_SHIFTER_H_
#define RTF_COM_SHAPE_SHIFTER_H_

#include <memory>
#include <securec.h>
#include "vrtf/vcc/api/stream.h"
#include "ara/hwcommon/log/log.h"
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
class ShapeShifter {
public:
    /**
     * @brief Construct a new shape shifter object
     */
    ShapeShifter();

    /**
     * @brief ShapeShifter copy constructor
     *
     * @param[in] other The other instance will be copy
     */
    ShapeShifter(const ShapeShifter &other);

    /**
     * @brief ShapeShifter copy assign operator
     *
     * @param[in] other The other instance will be copy
     */
    ShapeShifter& operator=(const ShapeShifter &other);

    /**
     * @brief ShapeShifter move constructor
     *
     * @param[in] other The other instance will be move
     */
    ShapeShifter(ShapeShifter &&other);

    /**
     * @brief ShapeShifter move assign operator
     *
     * @param[in] other The other instance will be move
     */
    ShapeShifter& operator=(ShapeShifter &&other);

    /**
     * @brief Destroy the Thread Group object
     */
    ~ShapeShifter();

    /**
     * @brief ShapeShifter write msg to stream
     *
     * @param[in] stream The stream to be writed
     */
    void Write(vrtf::vcc::api::types::Stream& stream) const noexcept;

    /**
     * @brief ShapeShifter read msg from stream
     *
     * @param[in] stream The stream to be read from
     */
    void Read(vrtf::vcc::api::types::Stream& stream) noexcept;

    /**
     * @brief Get ShapeShifter used size
     *
     * @return ShapeShifter used size
     */
    uint32_t Size() const noexcept;
private:
    uint8_t *msgBuf_;
    uint32_t msgBufUsed_;
    uint32_t msgBufAlloc_;
    std::shared_ptr<ara::godel::common::log::Log> logInstance_;
};
}
}
}
}
#endif
