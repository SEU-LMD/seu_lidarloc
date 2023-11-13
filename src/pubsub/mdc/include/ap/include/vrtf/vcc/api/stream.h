/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: The declaration of Stream
 * Create: 2020-12-02
 */
#ifndef RTF_COM_STREAM_H_
#define RTF_COM_STREAM_H_
#include <cstdint>
namespace vrtf {
namespace vcc {
namespace api {
namespace types {
class Stream {
public:
    /**
     * @brief Construct a new stream object
     *
     * @param[in] data Data address
     * @param[in] length The length of data
     */
    Stream(uint8_t* data, uint32_t length);

    /**
     * @brief Destroy the stream object
     */
    ~Stream() = default;

    /**
     * @brief Get stream data address
     *
     * @return Stream data address
     */
    uint8_t* GetData() noexcept;

    /**
     * @brief Get stream data length
     *
     * @return Stream data length
     */
    uint32_t GetLength() const noexcept;
private:
    uint8_t* data_;
    uint32_t length_;
};
}
}
}
}
#endif
