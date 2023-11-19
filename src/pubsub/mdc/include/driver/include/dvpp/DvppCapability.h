#ifndef DVPP_DVPPCAPABILITY_H
#define DVPP_DVPPCAPABILITY_H

#include "DvppCommon.h"
#include "ExportMacro.h"
#include <cstdint>

struct dvpp_capability_ioctl_msg {
    int32_t chan_num;
    int32_t in_size;
    int32_t out_size;
    void* in;
    void* out;
};

// Modified for 64-bit platform
struct compat_ioctl_msg {
    int32_t chan_num;
    int32_t in_size;
    int32_t out_size;
};


struct dvpp_request_msg_stru {
    uint32_t opt;
    void* in;
    void* out;
};

struct power_control_req_stru {
    uint32_t module_id;
    uint32_t enigne_type;
    uint32_t enigne_id;
    uint32_t ops;
    uint32_t state;
};

struct power_control_res_stru {
    uint32_t state;
};

struct device_query_req_stru {
    uint32_t module_id; // thr module id.
    uint32_t engine_type; // dvpp engine type.
};

struct module_feature_stru {
    uint32_t module_id;
    uint32_t max_input_chan;
    uint32_t support_type;
    unsigned char engine_count[16];
};


struct device_query_res_stru {
    uint32_t module_count;
    struct module_feature_stru module_info[16];
};

// capability capability info

// SP:Semi-Planar
// P: Packed
// 8/10BIT: bit depth
// LIN : linear
enum dvpp_color_format {
    // YUV444 in different ordering of YUV Semi-Planar/Packed,8 bit,Linear.
    DVPP_COLOR_YUV444_YUV_P_8BIT_LIN,
    DVPP_COLOR_YUV444_YVU_P_8BIT_LIN,
    DVPP_COLOR_YUV444_UYV_P_8BIT_LIN,
    DVPP_COLOR_YUV444_UVY_P_8BIT_LIN,
    DVPP_COLOR_YUV444_VYU_P_8BIT_LIN,
    DVPP_COLOR_YUV444_VUY_P_8BIT_LIN,

    DVPP_COLOR_YUV444_UV_SP_8BIT_LIN,
    DVPP_COLOR_YUV444_VU_SP_8BIT_LIN,

    // 422
    DVPP_COLOR_YUYV422_YUYV_P_8BIT_LIN,
    DVPP_COLOR_YUYV422_YVYU_P_8BIT_LIN,
    DVPP_COLOR_YUYV422_UYVY_P_8BIT_LIN,
    DVPP_COLOR_YUYV422_VYUY_P_8BIT_LIN,


    DVPP_COLOR_YUV422_UV_SP_8BIT_LIN,
    DVPP_COLOR_YUV422_VU_SP_8BIT_LIN,


    DVPP_COLOR_YUV420_SP_8BIT_LIN,
    DVPP_COLOR_YVU420_SP_8BIT_LIN,

    DVPP_COLOR_YUV420_SP_8BIT_HFBC,
    DVPP_COLOR_YVU420_SP_8BIT_HFBC,
    DVPP_COLOR_YUV420_SP_10BIT_HFBC,
    DVPP_COLOR_YVU420_SP_10BIT_HFBC,

    DVPP_COLOR_YUV420_P_8BIT_LIN,

    DVPP_COLOR_YVU400_SP_8BIT,

    DVPP_COLOR_RGB888_RGB_P_8BIT_LIN,
    DVPP_COLOR_RGB888_RBG_P_8BIT_LIN,
    DVPP_COLOR_RGB888_GBR_P_8BIT_LIN,
    DVPP_COLOR_RGB888_GRB_P_8BIT_LIN,
    DVPP_COLOR_RGB888_BRG_P_8BIT_LIN,
    DVPP_COLOR_RGB888_BGR_P_8BIT_LIN,

    DVPP_COLOR_ARGB8888_ARGB_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_ARBG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_AGBR_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_AGRB_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_ABRG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_ABGR_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RAGB_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RABG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RGBA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RGAB_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RBAG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_RBGA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BRGA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BRAG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BGAR_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BGRA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BARG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_BAGR_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GRAG_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GRBA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GABR_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GARB_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GBRA_P_8BIT_LIN,
    DVPP_COLOR_ARGB8888_GBAR_P_8BIT_LIN,
    PIC_JPEG,
    PIC_PNG,
    VIO_H265,
    VIO_H264
};

enum dvpp_proto_type {
    dvpp_proto_unsupport = -1,
    dvpp_itu_t81,
    iso_iec_15948_2003,
    h265_main_profile_level_5_1_hightier,
    h265_main_10_profile_level_5_1_hightier,
    h264_main_profile_level_5_1,
    h264_baseline_profile_level_5_1,
    h264_high_profile_level_5_1,
    h264_high_profile_level_4_1,
    h264_main_profile_level_4_1,
    h264_baseline_profile_level_4_1,
    h265_main_profile_level_4_1
};


enum dvpp_compress_type {
    arithmetic_code = 0,
    huffman_code
};

// Maximum number of standard protocols supported
#define DVPP_PROTOCOL_TYPE_MAX 6

// Maximum number of supported input / output formats
#define DVPP_VADIO_FORMAT_MAX 64

#define DVPP_PERFOMANCE_MODE_MAX 4

#define DVPP_PRE_CONTRATION_TYPE_MAX 3 // Indicates the number of pre-reduced fixed proportions.

#define DVPP_DATA_INPUT_SPEC_TYPE_MAX 6

enum dvpp_high_align_type {
    pix_random = 0,
    two_pix_alignment = 2,
    four_pix_alignment = 4,
    eight_pix_alignment = 8,
    sixteen_pix_alignment = 16
};

enum dvpp_align_type {
    align_random = 0,
    two_alignment = 2,
    four_alignment = 4,
    eight_alignment = 8,
    sixteen_alignment = 16,
    thirty_two_alignment = 32,
    sixty_four_alignment = 64
};

// Indicates whether to support this function. The options are as follows:
enum dvpp_support_type {
    no_support = 0, // no support
    do_support // support
};

enum dvpp_spec_input_type {
    input_type_rdma = 0,
    input_type_cvdr = 1
};

// Definition of the pre-reduction information structure
struct dvpp_pre_contraction_stru {
    // ability Available for engine, just for vpc now
    enum dvpp_support_type is_support; // vpc:support, others: no support

    // contraction types
    uint32_t contraction_types; // vpc: 3, others: 0

    // contraction size support
    uint32_t contraction_size[DVPP_PRE_CONTRATION_TYPE_MAX]; // vpc:2/4/8, others: 0

    // horizontal support
    enum dvpp_support_type is_horizontal_support; // vpc:support, others: no support

    // vertical support
    enum dvpp_support_type is_vertical_support; // vpc:support, others: no support
};

// Definition of the postscaling information structure
struct dvpp_pos_scale_stru {
    // ability Available for engine, just for vpc now
    enum dvpp_support_type is_support; // vpc:support, others: no support

    // min scale
    uint32_t min_scale; // vpc:1, others: 1

    // max scale
    uint32_t max_scale; // vpc:4, others: 1

    // horizontal support
    enum dvpp_support_type is_horizontal_support; // vpc:support, others: no support

    // vertical support
    enum dvpp_support_type is_vertical_support; // vpc:support, others: no support
};

// Picture format type unit definition structure
struct dvpp_format_unit_stru {
    // Picture color format YUV420/YUV/RGB
    enum dvpp_color_format color_format;

    // Data compression type HFBC, the default is 0, not compressed
    uint32_t compress_type;

    // Alignment size, in bytes
    uint32_t stride_size;

    enum dvpp_high_align_type high_alignment; // high alignment type.

    enum dvpp_high_align_type width_alignment; // width alignment type.

    uint32_t out_mem_alignment; // Output memory alignment parameters.
};

// Picture Performance Unit Definition Structure
struct dvpp_perfomance_unit_stru {
    // High picture
    uint32_t resolution_high;

    // width picture
    uint32_t resolution_width;

    // How many ways to support this format of processing
    uint32_t stream_num;

    // According to the above high-width stream processing frames per second
    unsigned long fps;
};

// Resolution structure
struct dvpp_resolution_stru {
    // High picture
    uint32_t resolution_high;

    // width picture
    uint32_t resolution_width;
};

struct dvpp_vpc_data_spec_stru {
    // type
    uint32_t input_type;

    struct dvpp_resolution_stru min_resolution;

    struct dvpp_resolution_stru max_resolution;

    enum dvpp_align_type high_alignment;

    enum dvpp_align_type width_alignment;
};

// DVPP Engine capability information query structure
struct dvpp_engine_capability_stru {
    int32_t engine_type;
    // The maximum resolution supported by the engine
    struct dvpp_resolution_stru max_resolution;

    // The minimum resolution supported by the engine
    struct dvpp_resolution_stru min_resolution;

    // The number of standard protocols supported by the engine
    uint32_t protocol_num;

    // Engine supports the standard protocol type table, 0xFF is invalid value,
    // does not involve filling as invalid value
    uint32_t protocol_type[DVPP_PROTOCOL_TYPE_MAX];

    // The number of input picture / video formats supported by the engine
    uint32_t input_format_num;

    // Engine support input picture / video format table
    struct dvpp_format_unit_stru engine_input_format_table[DVPP_VADIO_FORMAT_MAX];

    // The number of output picture / video formats supported by the engine
    uint32_t output_format_num;

    // Engine Support Output Picture / Video Format Table
    struct dvpp_format_unit_stru engine_output_format_table[DVPP_VADIO_FORMAT_MAX];

    // How many common features does the engine support
    uint32_t performance_mode_num;

    // Supported performance tables: eg: Need support(16 way 1080P 30FPS/2 way 3840*2160 60FPS/1*1 995,328,000Pixel/Sec)
    struct dvpp_perfomance_unit_stru performance_mode_table[DVPP_PERFOMANCE_MODE_MAX];

    // pre contraction information(for VPC)
    struct dvpp_pre_contraction_stru pre_contraction;

    // pos scale information(for VPC)
    struct dvpp_pos_scale_stru pos_scale;
    // The number of special input description
    uint32_t spec_input_num;
    // special input description
    struct dvpp_vpc_data_spec_stru spec_input[DVPP_DATA_INPUT_SPEC_TYPE_MAX];
};

class IDVPPCAPABILITY {};

#endif // DVPP_DVPPCAPABILITY_H