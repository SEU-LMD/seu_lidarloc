/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2012-2019. All rights reserved.
 * Description:
 * Author: huawei
 * Create: 2019-10-15
 */
#ifndef __DSMI_COMMON_INTERFACE_H__
#define __DSMI_COMMON_INTERFACE_H__
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __linux
#define DLLEXPORT
#else
#define DLLEXPORT _declspec(dllexport)
#endif

#include "ascend_hal_error.h"

#define DMP_MSG_HEAD_LENGTH     12
#define DMP_MAX_MSG_DATA_LEN    (1024 - DMP_MSG_HEAD_LENGTH)

#define BIT_IF_ONE(number, n) (((number) >> (n)) & (0x1))
#define MAX_FILE_LEN 256
#define MAX_LINE_LEN 255
#define MAX_LOCK_NAME 30
#define PCIE_EP_MODE 0X0
#define PCIE_RC_MODE 0X1

#define DAVINCHI_SYS_VERSION 0XFF
#define INVALID_DEVICE_ID 0XFF



// 1980 dsmi return value
#define DM_DDMP_ERROR_CODE_EAGAIN DRV_ERROR_TRY_AGAIN                     /**< same as EAGAIN */
#define DM_DDMP_ERROR_CODE_PERM_DENIED DRV_ERROR_OPER_NOT_PERMITTED                 /**< same as EPERM */
// all of follow must same as inc/base.h
#define DM_DDMP_ERROR_CODE_SUCCESS DRV_ERROR_NONE                      /**< success */
#define DM_DDMP_ERROR_CODE_PARAMETER_ERROR DRV_ERROR_PARA_ERROR              /**< param error */
#define DM_DDMP_ERROR_CODE_INVALID_HANDLE_ERROR DRV_ERROR_INVALID_HANDLE         /**< invalid fd handle */
#define DM_DDMP_ERROR_CODE_TIME_OUT DRV_ERROR_WAIT_TIMEOUT                    /**< wait time out */
#define DM_DDMP_ERROR_CODE_IOCRL_ERROR DRV_ERROR_IOCRL_FAIL                 /**< ioctl error */
#define DM_DDMP_ERROR_CODE_INVALID_DEVICE_ERROR DRV_ERROR_INVALID_DEVICE        /**< invalid device */
#define DM_DDMP_ERROR_CODE_SEND_ERROR DRV_ERROR_SEND_MESG                  /**< hdc or upd send error */
#define DM_DDMP_ERROR_CODE_INTERNAL_ERROR DRV_ERROR_INNER_ERR              /**< internal error */
#define DM_DDMP_ERROR_CODE_NOT_SUPPORT DRV_ERROR_NOT_SUPPORT                 /**< dsmi command not support error */
#define DM_DDMP_ERROR_CODE_MEMERY_OPRATOR_ERROR DRV_ERROR_MEMORY_OPT_FAIL        /**< system memory function error */
#define DM_DDMP_ERROR_CODE_PERIPHERAL_DEVICE_NOT_EXIST DRV_ERROR_NOT_EXIST /**< peripheral device not exist, BMC used */


typedef struct dm_flash_info_stru {
    unsigned long long flash_id;    /**< combined device & manufacturer code */
    unsigned short device_id;       /**< device id    */
    unsigned short vendor;          /**< the primary vendor id  */
    unsigned int state;             /**< flash health */
    unsigned long long size;        /**< total size in bytes  */
    unsigned int sector_count;      /**< number of erase units  */
    unsigned short manufacturer_id; /** manufacturer id   */
} DM_FLASH_INFO_STRU, dm_flash_info_stru;

typedef struct tag_pcie_idinfo {
    unsigned int deviceid;
    unsigned int venderid;
    unsigned int subvenderid;
    unsigned int subdeviceid;
    unsigned int bdf_deviceid;
    unsigned int bdf_busid;
    unsigned int bdf_funcid;
} TAG_PCIE_IDINFO, tag_pcie_idinfo;

typedef struct tag_ecc_stat {
    unsigned int error_count;
} TAG_ECC_STAT, tag_ecc_stat;

typedef struct dsmi_upgrade_control {
    unsigned char control_cmd;
    unsigned char component_type;
    unsigned char file_name[MAX_FILE_LEN];
} DSMI_UPGRADE_CONTROL;

typedef enum dsmi_upgrade_device_state {
    UPGRADE_IDLE_STATE = 0,
    IS_UPGRADING = 1,
    UPGRADE_NOT_SUPPORT = 2,
    UPGRADE_UPGRADE_FAIL = 3,
    UPGRADE_STATE_NONE = 4,
    UPGRADE_WAITTING_RESTART = 5,
    UPGRADE_WAITTING_SYNC = 6,
    UPGRADE_SYNCHRONIZING = 7
} DSMI_UPGRADE_DEVICE_STATE;

typedef enum {
    DSMI_DEVICE_TYPE_DDR,
    DSMI_DEVICE_TYPE_SRAM,
    DSMI_DEVICE_TYPE_HBM,
    DSMI_DEVICE_TYPE_NPU,
    DSMI_DEVICE_TYPE_NONE = 0xff
} DSMI_DEVICE_TYPE;

typedef enum dsmi_boot_status {
    DSMI_BOOT_STATUS_UNINIT = 0, /**< uninit status */
    DSMI_BOOT_STATUS_BIOS,       /**< status of starting BIOS */
    DSMI_BOOT_STATUS_OS,         /**< status of starting OS */
    DSMI_BOOT_STATUS_FINISH      /**< finish boot start */
} DSMI_BOOT_STATUS;

typedef enum rdfx_detect_result {
    RDFX_DETECT_OK = 0,
    RDFX_DETECT_SOCK_FAIL = 1,
    RDFX_DETECT_RECV_TIMEOUT = 2,
    RDFX_DETECT_UNREACH = 3,
    RDFX_DETECT_TIME_EXCEEDED = 4,
    RDFX_DETECT_FAULT = 5,
    RDFX_DETECT_INIT = 6,
    RDFX_DETECT_MAX
} DSMI_NET_HEALTH_STATUS;

#define UTLRATE_TYPE_DDR 1
#define UTLRATE_TYPE_AICORE 2
#define UTLRATE_TYPE_AICPU 3
#define UTLRATE_TYPE_CTRLCPU 4
#define UTLRATE_TYPE_DDR_BANDWIDTH 5
#define UTLRATE_TYPE_HBM 6

#define TAISHAN_CORE_NUM 16
typedef struct dsmi_aicpu_info_stru {
    unsigned int maxFreq;
    unsigned int curFreq;
    unsigned int aicpuNum;
    unsigned int utilRate[TAISHAN_CORE_NUM];
} DSMI_AICPU_INFO;

typedef enum {
    ECC_CONFIG_ITEM = 0X0,
    P2P_CONFIG_ITEM = 0X1,
    DFT_CONFIG_ITEM = 0X2
} CONFIG_ITEM;

typedef enum dsmi_component_type {
    DSMI_COMPONENT_TYPE_NVE,
    DSMI_COMPONENT_TYPE_XLOADER,
    DSMI_COMPONENT_TYPE_M3FW,
    DSMI_COMPONENT_TYPE_UEFI,
    DSMI_COMPONENT_TYPE_TEE,
    DSMI_COMPONENT_TYPE_KERNEL,
    DSMI_COMPONENT_TYPE_DTB,
    DSMI_COMPONENT_TYPE_ROOTFS,
    DSMI_COMPONENT_TYPE_IMU,
    DSMI_COMPONENT_TYPE_IMP,
    DSMI_COMPONENT_TYPE_AICPU,
    DSMI_COMPONENT_TYPE_MAX,
    UPGRADE_AND_RESET_ALL_COMPONENT = 0xFFFFFFF7,
    UPGRADE_ALL_COMPONENT = 0xFFFFFFFF
} DSMI_COMPONENT_TYPE;

#define DSMI_COMPONENT_TYPE_BOOTROM  DSMI_COMPONENT_TYPE_MAX
#define DSMI_COMPONENT_TYPE_NONE  DSMI_COMPONENT_TYPE_MAX
#define MAX_COMPONENT_NUM 32


typedef struct cfg_file_des {
    unsigned char component_type;
    char src_component_path[MAX_FILE_LEN];
    char dst_compoent_path[MAX_FILE_LEN];
} CFG_FILE_DES;

typedef enum {
    DSMI_REVOCATION_TYPE_SOC = 0,
    DSMI_REVOCATION_TYPE_MAX
} DSMI_REVOCATION_TYPE;

typedef void (*fault_event_handler)(unsigned int faultcode, unsigned int faultstate);

#define DSMI_SOC_DIE_LEN 5
struct dsmi_soc_die_stru {
    unsigned int soc_die[DSMI_SOC_DIE_LEN]; /**< 5 soc_die arrary sizet */
};
struct dsmi_power_info_stru {
    unsigned short power;
};
struct dsmi_memory_info_stru {
    unsigned long long memory_size;
    unsigned int freq;
    unsigned int utiliza;
};

struct dsmi_hbm_info_stru {
    unsigned long long memory_size;      /**< HBM total size, KB */
    unsigned int freq;                   /**< HBM freq, MHZ */
    unsigned long long memory_usage;     /**< HBM memory_usage, KB */
    int temp;                            /**< HBM temperature */
    unsigned int bandwith_util_rate;
};

struct dsmi_aicore_info_stru {
    unsigned int freq;
    unsigned int utiliza;
};

struct dsmi_ecc_info_stru {
    int enable_flag;
    unsigned int single_bit_error_count;
    unsigned int double_bit_error_count;
};

struct tag_cgroup_info {
    unsigned long long limit_in_bytes;       /**< maximum number of used memory */
    unsigned long long max_usage_in_bytes;   /**< maximum memory used in history */
    unsigned long long usage_in_bytes;       /**< current memory usage */
};

#define MAX_CHIP_NAME 32
#define MAX_DEVICE_COUNT 64

struct dsmi_chip_info_stru {
    unsigned char chip_type[MAX_CHIP_NAME];
    unsigned char chip_name[MAX_CHIP_NAME];
    unsigned char chip_ver[MAX_CHIP_NAME];
};

#define DSMI_VNIC_PORT 0
#define DSMI_ROCE_PORT 1

enum ip_addr_type {
    IPADDR_TYPE_V4  = 0U,    /**< IPv4 */
    IPADDR_TYPE_V6  = 6U,    /**< IPv6 */
    IPADDR_TYPE_ANY = 46U    /**< IPv4+IPv6 ("dual-stack") */
};

#define DSMI_ARRAY_IPV4_NUM 4
#define DSMI_ARRAY_IPV6_NUM 16

typedef struct ip_addr {
    union {
        unsigned char ip6[DSMI_ARRAY_IPV6_NUM];
        unsigned char ip4[DSMI_ARRAY_IPV4_NUM];
    } u_addr;
    enum ip_addr_type ip_type;
} ip_addr_t;


#define COMPUTING_POWER_PMU_NUM 4

typedef struct dsmi_cntpct_stru {
    unsigned long long state;
    unsigned long long timestamp1;
    unsigned long long timestamp2;
    unsigned long long event_count[COMPUTING_POWER_PMU_NUM];
} DMSI_CNTPCT;


typedef enum dsmi_channel_index {
    DEVICE = 0,
    HOST = 1,
    MCU = 2
} DMSI_CHANNEL_INDEX;

struct dmp_req_message_stru {
    unsigned char lun;
    unsigned char arg;
    unsigned short opcode;
    unsigned int offset;
    unsigned int length;
    unsigned char data[DMP_MAX_MSG_DATA_LEN];
};

#define DSMI_RSP_MSG_DATA_LEN 1012
struct dmp_rsp_message_stru {
    unsigned short errorcode;
    unsigned short opcode;
    unsigned int total_length;
    unsigned int length;
    unsigned char data[DSMI_RSP_MSG_DATA_LEN]; /**< 1012 rsp data size */
};

struct dmp_message_stru {
    union {
        struct dmp_req_message_stru req;
        struct dmp_rsp_message_stru rsp;
    } data;
};

struct passthru_message_stru {
    unsigned int src_len;
    unsigned int rw_flag; /**< 0 read ,1 write */
    struct dmp_message_stru src_message;
    struct dmp_message_stru dest_message;
};

struct dsmi_board_info_stru {
    unsigned int board_id;
    unsigned int pcb_id;
    unsigned int bom_id;
    unsigned int slot_id;
};

typedef struct dsmi_llc_perf_stru {
    unsigned int wr_hit_rate;
    unsigned int rd_hit_rate;
    unsigned int throughput;
} DSMI_LLC_PERF_INFO;

#define SENSOR_DATA_MAX_LEN 16

#define DSMI_TAG_SENSOR_TEMP_LEN 2
#define DSMI_TAG_SENSOR_NTC_TEMP_LEN 4
typedef union tag_sensor_info {
    unsigned char uchar;
    unsigned short ushort;
    unsigned int uint;
    signed int iint;
    signed char temp[DSMI_TAG_SENSOR_TEMP_LEN];   /**<  2 temp size */
    signed int ntc_tmp[DSMI_TAG_SENSOR_NTC_TEMP_LEN]; /**<  4 ntc_tmp size */
    unsigned int data[SENSOR_DATA_MAX_LEN];
} TAG_SENSOR_INFO;

#ifndef MAX_MATRIX_PROC_NUM
#define MAX_MATRIX_PROC_NUM 256
#endif

typedef struct {
    unsigned int pid;
    unsigned int mem_rate;
    unsigned int cpu_rate;
} DSMI_MATRIX_PORC_INFO_S;

struct dsmi_matrix_proc_info_get_stru {
    DSMI_MATRIX_PORC_INFO_S proc_info[MAX_MATRIX_PROC_NUM];
    int output_num;
};

typedef enum {
    POWER_STATE_SUSPEND,
    POWER_STATE_POWEROFF,
    POWER_STATE_MAX,
} DSMI_POWER_STATE;

#define DDR_ECC_CONFIG_NAME      "ddr_ecc_enable"

#define MAX_CAN_NAME 32

typedef struct dsmi_emmc_status_stru {
    unsigned int    clock;               /**< clock rate */
    unsigned int    clock_store;         /**< store the clock before power off */
    unsigned short  vdd;                 /**< vdd stores the bit number of the selected voltage range from below. */
    unsigned int    power_delay_ms;      /**< waiting for stable power */
    unsigned char   bus_mode;            /**< command output mode */
    unsigned char   chip_select;         /**< SPI chip select */
    unsigned char   power_mode;          /**< power supply mode */
    unsigned char   bus_width;           /**< data bus width */
    unsigned char   timing;              /**< timing specification used */
    unsigned char   signal_voltage;      /**< signalling voltage (1.8V or 3.3V) */
    unsigned char   drv_type;            /**< A, B, C, D */
    unsigned char   enhanced_strobe;     /**< hs400es selection */
} DSMI_EMMC_STATUS_STRU;

typedef enum {
    BUS_STATE_ACTIVER,
    BUS_STATE_ERR_WARNING,
    BUS_STATE_ERR_PASSIVE,
    BUS_STATE_ERR_BUSOFF,
    BUS_STATE_DOWN,
} DSMI_CAN_BUS_STATE;

typedef struct dsmi_can_status_stru {
    DSMI_CAN_BUS_STATE bus_state;
    unsigned int rx_err_counter;
    unsigned int tx_err_counter;
    unsigned int err_passive;
} DSMI_CAN_STATUS_STRU;

typedef enum {
    UFS_STATE_LINKOFF,
    UFS_STATE_ACTIVE,
    UFS_STATE_HIBERN8,
} DSMI_UFS_STATE;

typedef enum {
    UFS_FAST_MODE   = 1,
    UFS_SLOW_MODE   = 2,
    UFS_FASTAUTO_MODE   = 4,
    UFS_SLOWAUTO_MODE   = 5,
    UFS_UNCHANGED   = 7,
} DSMI_UFS_PWR_MODE;

typedef enum {
    UFS_PA_HS_MODE_A    = 1,
    UFS_PA_HS_MODE_B    = 2,
} DSMI_UFS_HS_MODE;

typedef enum {
    UFS_DONT_CHANGE,
    UFS_GEAR_1,
    UFS_GEAR_2,
    UFS_GEAR_3,
} DSMI_UFS_GEAR;

typedef enum {
    UFS_UIC_LINK_OFF_STATE  = 0,     /**< Link powered down or disabled */
    UFS_UIC_LINK_ACTIVE_STATE   = 1, /**< Link is in Fast/Slow/Sleep state */
    UFS_UIC_LINK_HIBERN8_STATE  = 2, /**< Link is in Hibernate state */
} DSMI_UFS_LINK_STATE;

typedef enum {
    UFS_DEV_PWR_ACTIVE = 1,
    UFS_DEV_PWR_SLEEP  = 2,
    UFS_DEV_PWR_POWERDOWN  = 3,
} DSMI_UFS_DEV_PWR_STATE;

typedef enum {
    UFS_DEV_CLK_19M2 = 0,
    UFS_DEV_CLK_26M0,
    UFS_DEV_CLK_38M4,
    UFS_DEV_CLK_52M0,
    UFS_DEV_CLK_INVAL,
} DSMI_UFS_DEV_CLOCK;

typedef enum {
    UFS_PM_LEVEL_0, /**< UFS_ACTIVE_PWR_MODE, UIC_LINK_ACTIVE_STATE */
    UFS_PM_LEVEL_1, /**< UFS_ACTIVE_PWR_MODE, UIC_LINK_HIBERN8_STATE */
    UFS_PM_LEVEL_2, /**< UFS_SLEEP_PWR_MODE, UIC_LINK_ACTIVE_STATE */
    UFS_PM_LEVEL_3, /**< UFS_SLEEP_PWR_MODE, UIC_LINK_HIBERN8_STATE */
    UFS_PM_LEVEL_4, /**< UFS_POWERDOWN_PWR_MODE, UIC_LINK_HIBERN8_STATE */
    UFS_PM_LEVEL_5, /**< UFS_POWERDOWN_PWR_MODE, UIC_LINK_OFF_STATE */
    UFS_PM_LEVEL_MAX,
} DSMI_UFS_PM_LEVEL;

typedef enum {
    STATE_NORMAL = 0,
    STATE_MINOR,
    STATE_MAJOR,
    STATE_FATAL,
} DSMI_FAULT_STATE;


typedef struct dsmi_ufs_status_stru {
    DSMI_UFS_STATE status;                      /**< ufs status */
    DSMI_UFS_PWR_MODE rx_pwr_mode;              /**< rx rate mode */
    DSMI_UFS_PWR_MODE tx_pwr_mode;              /**< tx rate mode */
    DSMI_UFS_GEAR rx_pwr_gear;                  /**< rx rate */
    DSMI_UFS_GEAR tx_pwr_gear;                  /**< tx rate */
    unsigned int rx_lanes;                      /**< rx lanes */
    unsigned int tx_lanes;                      /**< tx lanes */
    DSMI_UFS_LINK_STATE link_pwr_status;        /**< link power status */
    DSMI_UFS_DEV_PWR_STATE device_pwr_status;   /**< device power status */
    unsigned int temperature;                   /**< ufs device temperature */
    unsigned int fault_status;                  /**< ufs device exception status */
    unsigned int total_capacity;                /**< total raw device capacity */
    unsigned int model_number;                  /**< ufs device sub class */
    unsigned int device_life_time;              /**< ufs device life time left */
    unsigned int fw_ver;                        /**< product revision level */
    unsigned int fw_update_enable;              /**< whether to support firmware update: 0-not support, 1-support */
} DSMI_UFS_STATUS_STRU;

typedef struct dsmi_sensorhub_status_stru {
    unsigned int status;                         /**< 0:normal, others:abnormal */
    unsigned int timestamp_lost_error_cnt;       /**< timestamp irq lost error count */
    unsigned int timestamp_op_error_cnt;         /**< timestamp read/write operation error count */
    unsigned int pps_lost_error_cnt;             /**< GPS PPS lost error count */
} DSMI_SENSORHUB_STATUS_STRU;

#define MAX_SID_FILTER_NUM  128
#define MAX_XID_FILTER_NUM  64

struct sid_filter_stru {
    /**
     * Standard Filter Type:
     * 0 - Range filter from SFID1 to SFID2
     * 1 - Dual ID filter for SFID1 or SFID2
     * 2 - Classic filter: SFID1 = filter, SFID2 = mask
     * 3 - Filter element disabled
     */
    unsigned int sft : 2;
    /**
     * Standard Filter Element Configuration:
     * 0 - Disable filter element
     * 1 - Store in Rx FIFO 0 if filter matches
     * 2 - Store in Rx FIFO 1 if filter matches
     * 3 - Reject ID if filter matches, not intended to be used with Sync messages
     * 4 - Set priority if filter matches, not intended to be used with Sync messages, no storage
     * 5 - Set priority and store in FIFO 0 if filter matches
     * 6 - Set priority and store in FIFO 1 if filter matches
     * 7 - Store into Rx Buffer or as debug message, configuration of SFT[1:0] ignored
     */
    unsigned int sfec : 3;
    unsigned int sfid1 : 11;   /**< Standard Filter ID 1 */
    /**
     * Standard Sync Message
     * 0 - Timestamping for the matching Sync message disabled
     * 1 - Timestamping for the matching Sync message enabled
     */
    unsigned int ssync : 1;
    unsigned int res : 4;    /**< reserved */
    unsigned int sfid2 : 11; /**< Standard Filter ID 2 */
};

struct xid_filter_stru {
    /**
     * Extended Filter Element Configuration
     * 0 - Disable filter element
     * 1 - Store in Rx FIFO 0 if filter matches
     * 2 - Store in Rx FIFO 1 if filter matches
     * 3 - Reject ID if filter matches, not intended to be used with Sync messages
     * 4 - Set priority if filter matches, not intended to be used with Sync messages, no storage
     * 5 - Set priority and store in FIFO 0 if filter matches
     * 6 - Set priority and store in FIFO 1 if filter matches
     * 7 - Store into Rx Buffer or as debug message, configuration of EFT[1:0] ignored
     */
    unsigned int efec : 3;
    unsigned int efid1 : 29; /* Extended Filter ID 1 */
    /**
     * Extended Filter Type
     * 0 - Range filter from SFID1 to SFID2
     * 1 - Dual ID filter for SFID1 or SFID2
     * 2 - Classic filter: SFID1 = filter, SFID2 = mask
     * 3 - Range filter from EFID1 to EFID2 (EFID2 >= EFID1), XIDAM mask not applied
     */
    unsigned int eft : 2;
    /**
     * Extended Sync Message
     * 0 - Timestamping for the matching Sync message disabled
     * 1 - Timestamping for the matching Sync message enabled
     */
    unsigned int esync : 1;
    unsigned int efid2 : 29; /* Extended Filter ID 2 */
};

struct global_filter_stru {
    /* reserved */
    unsigned int res : 26;
    /**
     * Accept Non-matching Frames Standard
     * 0 - Accept in Rx FIFO 0
     * 1 - Accept in Rx FIFO 1
     * 2 - Reject
     * 3 - Reject
     */
    unsigned int anfs : 2;
    /**
     * Accept Non-matching Frames Standard
     * 0 - Accept in Rx FIFO 0
     * 1 - Accept in Rx FIFO 1
     * 2 - Reject
     * 3 - Reject
     */
    unsigned int anfe : 2;
    /**
     * Reject Remote Frames Standard
     * 0 - Filter remote frames with 11-bit standard IDs
     * 1 - Reject all remote frames with 11-bit standard IDs
     */
    unsigned int rrfs : 1;
    /**
     * Reject Remote Frames Extended
     * 0 - Filter remote frames with 29-bit extended IDs
     * 1 - Reject all remote frames with 29-bit extended IDs
     */
    unsigned int rrfe : 1;
};

typedef enum {
    RX_FIFO_BLOCKING_MODE,
    RX_FIFO_OVERWRITE_MODE,
} DSMI_CAN_RX_FIFO_MODE;

typedef enum {
    TX_FIFO_OPERATION,
    TX_QUEUE_OPERATION,
} DSMI_CAN_TX_FIFO_QUEUE_MODE;

typedef struct  dsmi_can_config_stru {
    unsigned int element_num_rxf0;                          /**< Rx FIFO 0 quantity */
    unsigned int element_num_rxf1;                          /**< Rx FIFO 1 quantity */
    unsigned int element_num_rxb;                           /**< Rx Buffer quantity */
    unsigned int element_num_txef;                          /**< Tx Event FIFO quantity */
    unsigned int element_num_txb;                           /**< Tx Buffer quantity */
    unsigned int element_num_tmc;                           /**< Trigger Memory quantity */
    unsigned int tx_elmt_num_dedicated_buf;                 /**< Tx dedicated buf quantity */
    unsigned int tx_elmt_num_fifo_queue;                    /**< Tx FIFO/Queue quantity */
    unsigned int dsize_fifo0;                               /**< Rx FIFO 0 data size */
    unsigned int dsize_fifo1;                               /**< Rx FIFO 1 data size */
    unsigned int dsize_rxb;                                 /**< Rx Buffer data size */
    unsigned int dsize_txb;                                 /**< Tx Buffer data size */
    unsigned int watermark_rxf0;                            /**< Rx FIFO 0 watermark */
    unsigned int watermark_rxf1;                            /**< Rx FIFO 1 watermark */
    unsigned int watermark_txef;                            /**< Tx Event FIFO watermark */
    DSMI_CAN_RX_FIFO_MODE mode_rxf0;                        /**< Rx FIFO 0 work mode */
    DSMI_CAN_RX_FIFO_MODE mode_rxf1;                        /**< Rx FIFO 0 work mode */
    DSMI_CAN_TX_FIFO_QUEUE_MODE mode_txfq;                  /**< Tx Event FIFO work mode */
    unsigned int element_num_sidf;                          /**< Standard ID Filter quantity */
    unsigned int element_num_xidf;                          /**< Extended ID Filter quantity */
    struct sid_filter_stru sid_filter[MAX_SID_FILTER_NUM];  /**< Standard ID Filter */
    struct xid_filter_stru xid_filter[MAX_XID_FILTER_NUM];  /**< Extended ID Filter */
    struct global_filter_stru global_filter;                /**< Global Filter */
    unsigned int xid_and_mask;                              /**< Extended ID AND Mask */
    unsigned int echo_skb_max;                              /**< Local socket buffer quantity */
    unsigned int poll_weight;                               /**< napi poll weight */
    unsigned int ts_cnt_prescaler;                          /**< timestamp counter prescaler */
} DSMI_CAN_CONFIG_STRU;

typedef struct  dsmi_ufs_config_stru {
    DSMI_UFS_PWR_MODE pwr_mode;             /**< Link Rate Mode */
    DSMI_UFS_GEAR pwr_gear;                 /**< Link Rate */
    DSMI_UFS_HS_MODE hs_series;             /**< HS Series, Only query, not configuration */
    DSMI_UFS_PM_LEVEL suspend_pwr_level;    /**< HS Series, Only query, not configuration */
    unsigned int auto_h8;                   /**< enable autoH8: 0-disable, 1-enable */
    unsigned int lane_count;                /**< active lanes count */
    DSMI_UFS_DEV_CLOCK device_refclk;       /**< Reference Clock Frequency value, Only query, not configuration */
} DSMI_UFS_CONFIG_STRU;


#define __FSIN_INT_NUM 4
typedef struct dsmi_sensorhub_config_stru {
    unsigned int fsin_fps[__FSIN_INT_NUM];           /**< fsin0~3_thr frame rate value */
    unsigned int imu_fps;                            /**< imu0_thr frame rate value */
    unsigned int ssu_ctrl_ssu_en;                    /**< ssu enable,  */
    unsigned int ssu_ctrl_pps_sel;                   /**< pps resource select,  */
    unsigned int pps_lock_thr;                       /**< GPS pps lock threshold */
    unsigned int pps_lost_thr;                       /**< GPS pps lost threshold */
    unsigned int fsin_initial_pre[__FSIN_INT_NUM];   /**< fsin0~3 initial advance count to pps pulse */
    unsigned int imu0_initial_pre;                   /**< imu0 initial advance count to pps pulse */
    unsigned int ssu_normal_taishan_mask;            /**< mask of ssu normal irq to taishan */
    unsigned int ssu_normal_taishan_mask_2;          /**< mask of GPS PPS lost irq to taishan */
    unsigned int ssu_error_taishan_mask_1;           /**< mask of read/write error irq to taishan(expose mode) */
    unsigned int ssu_error_taishan_mask_2;           /**< mask of read/write error irq to taishan(strobe mode) */
    unsigned int ssu_path_en;                        /**< register to enable interrupt path */
    unsigned int fsin_pw[__FSIN_INT_NUM];            /**< fsin0~3 pulse width count per 5ns */
    unsigned int imu_pw;                             /**< imu0 pulse width count per 5ns */
    unsigned int inter_pps_thr;                      /**< pps pulse count per 5ns */
    unsigned int fsin_thr[__FSIN_INT_NUM];           /**< fsin0~3 pulse count per 5ns */
    unsigned int imu_thr;                            /**< imu0 pulse count per 5ns */
    unsigned int timestamp_check_en;                 /**< timestamp check enable */
} DSMI_SENSORHUB_CONFIG_INFO_STRU;

typedef enum {
    HISS_NOT_INITIALIZED,
    HISS_ERROR,
    HISS_OK
} ENUM_HISS_STATUS;

typedef enum {
    DSMI_UPGRADE_ATTR_SYNC
} DSMI_UPGRADE_ATTR;

typedef struct  dsmi_hiss_status_stru {
    ENUM_HISS_STATUS hiss_status_code;
    unsigned long long hiss_status_info;
} DSMI_HISS_STATUS_STRU;

typedef struct dsmi_lp_status_stru {
    unsigned int status;
    unsigned long long status_info;
} DSMI_LP_STATUS_STRU;

typedef struct dsmi_vector_info_stru {
    unsigned int freq;    /**< normal freq */
    unsigned int rate;    /**< normal freq */
} DSMI_VECTOR_INFO;

struct  dsmi_uart_status_stru {
    unsigned int status;
    unsigned int speed;
};

struct  dsmi_uart_config_stru {
    unsigned int Baud;
};

#define DSMI_EMU_ISP_MAX 2
#define DSMI_EMU_DVPP_MAX 3
#define DSMI_EMU_CPU_CLUSTER_MAX 4
#define DSMI_EMU_AICORE_MAX 10
#define DSMI_EMU_AIVECTOR_MAX 8

struct  dsmi_emu_subsys_state_stru {
    DSMI_FAULT_STATE emu_sys;
    DSMI_FAULT_STATE emu_sils;
    DSMI_FAULT_STATE emu_sub_sils;
    DSMI_FAULT_STATE emu_sub_peri;
    DSMI_FAULT_STATE emu_sub_ao;
    DSMI_FAULT_STATE emu_sub_hac;
    DSMI_FAULT_STATE emu_sub_gpu;
    DSMI_FAULT_STATE emu_sub_isp[DSMI_EMU_ISP_MAX];
    DSMI_FAULT_STATE emu_sub_dvpp[DSMI_EMU_DVPP_MAX];
    DSMI_FAULT_STATE emu_sub_io;
    DSMI_FAULT_STATE emu_sub_ts;
    DSMI_FAULT_STATE emu_sub_cpu_cluster[DSMI_EMU_CPU_CLUSTER_MAX];
    DSMI_FAULT_STATE emu_sub_aicore[DSMI_EMU_AICORE_MAX];
    DSMI_FAULT_STATE emu_sub_aivector[DSMI_EMU_AIVECTOR_MAX];
    DSMI_FAULT_STATE emu_sub_media;
    DSMI_FAULT_STATE emu_sub_lp;
    DSMI_FAULT_STATE emu_sub_tsv;
    DSMI_FAULT_STATE emu_sub_tsc;
};

struct  dsmi_safetyisland_status_stru {
    DSMI_FAULT_STATE status;
} ;

struct dsmi_cpu_cfg_stru {
    unsigned int ctrl_cpu_num;
    unsigned int data_cpu_num;
    unsigned int ai_cpu_num;
};

/* DSMI main command for common interface */
typedef enum {
    DSMI_MAIN_CMD_DVPP = 0,
    DSMI_MAIN_CMD_ISP,
    DSMI_MAIN_CMD_TS_GROUP_NUM,
} DSMI_MAIN_CMD;

#define COMPUTE_GROUP_INFO_RES_NUM 8

/* dsmi ts identifier for get ts group info */
typedef enum {
    DSMI_TS_AICORE = 0,
    DSMI_TS_AIVECTOR,
} DSMI_TS_ID;

struct dsmi_capability_group_info {
    unsigned int  group_id;
    unsigned int  state; // 0: not create, 1: created
    unsigned int  extend_attribute; // 0: default group attribute
    unsigned int  aicore_number; // 0~9
    unsigned int  aivector_number; // 0~7
    unsigned int  sdma_number; // 0~15
    unsigned int  aicpu_number; // 0~15
    unsigned int  active_sq_number; // 0~31
    unsigned int  res[COMPUTE_GROUP_INFO_RES_NUM];
};

/**
* @ingroup driver
* @brief Get the specified elabel
* @attention NULL
* @param [in] device_id  The device id
* @param [in] item_type The elabel_data type
* @param [out] elabel_data  data
* @param [out] len  data length
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_dft_get_elable(int device_id, int item_type, char *elable_data, int *len);

/**
* @ingroup driver
* @brief start upgrade
* @attention Support to upgrade one firmware of a device, or all upgradeable firmware of a device (the second
            parameter is set to 0xFFFFFFFF), Does not support upgrading all devices, implemented by upper
            layer encapsulation interface
* @param [in] device_id  The device id
* @param [in] component_type firmware type
* @param [in] file_name  the path of firmware
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_upgrade_start(int device_id, DSMI_COMPONENT_TYPE component_type, const char *file_name);

/**
* @ingroup driver
* @brief set upgrade attr
* @attention NULL
* @param [in] device_id  The device id
* @param [in] component_type firmware type
* @param [in] attr the upgrade attr
* @return  0 for success, others for fail
* @note Support:Reserve
*/
DLLEXPORT int dsmi_set_upgrade_attr(int device_id, DSMI_COMPONENT_TYPE component_type, DSMI_UPGRADE_ATTR attr);

/**
* @ingroup driver
* @brief get upgrade state
* @attention NULL
* @param [in] device_id  The device id
* @param [out] schedule  Upgrade progress
* @param [out] upgrade_status  Upgrade state
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_upgrade_get_state(int device_id, unsigned char *schedule, unsigned char *upgrade_status);

/**
* @ingroup driver
* @brief get the version of firmware
* @attention The address of the third parameter version number is applied by the user,
             the module only performs non-null check on it, and the size is guaranteed by the user
* @param [in] device_id  The device id
* @param [out] schedule  Upgrade progress
* @param [out] version_str  The space requested by the user stores the returned firmware version number
* @param [out] version_len  the length of version_str
* @param [out] ret_len  The space requested by the user is used to store the effective character length
               of the version number
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_upgrade_get_component_static_version(int device_id, DSMI_COMPONENT_TYPE component_type,
    unsigned char *version_str, unsigned int version_len, unsigned int *ret_len);

/**
* @ingroup driver
* @brief Get the system version number
* @attention The address of the second parameter version number is applied by the user,
             the module only performs non-null check on it, and the size is guaranteed by the user
* @param [in] device_id  The device id
* @param [out] version_str  User-applied space stores system version number
* @param [out] version_len  length of paramer version_str
* @param [out] ret_len  The space requested by the user is used to store the effective
               length of the returned system version number
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_version(int device_id, char *version_str, unsigned int version_len, unsigned int *ret_len);

/**
* @ingroup driver
* @brief get upgrade state
* @attention Get the number of firmware that can be support
* @param [in] device_id  The device id
* @param [out] component_count  The space requested by the user for storing the number of firmware returned
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_component_count(int device_id, unsigned int *component_count);


/**
* @ingroup driver
* @brief get upgrade state
* @attention NULL
* @param [in] device_id  The device id
* @param [out] component_table  The space requested by the user is used to store the returned firmware list
* @param [out] component_count  The count of firmware
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_component_list(int device_id, DSMI_COMPONENT_TYPE *component_table, unsigned int component_count);

/**
* @ingroup driver
* @brief Get the number of devices
* @attention NULL
* @param [out] device_count  The space requested by the user is used to store the number of returned devices
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_count(int *device_count);

/**
* @ingroup driver
* @brief Get the number of devices
* @attention the devices can obtain from lspci command,not just the devices can obtain from device manager.
* @param [out] all_device_count  The space requested by the user is used to store the number of returned devices
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_get_all_device_count(int *all_device_count);

/**
* @ingroup driver
* @brief Get the id of all devices
* @attention NULL
* @param [out] device_count The space requested by the user is used to store the id of all returned devices
* @param [out] count Number of equipment
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_list_device(int device_id_list[], int count);

/**
* @ingroup driver
* @brief Start the container service
* @attention Cannot be used simultaneously with the computing power distribution mode
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_enable_container_service(void);

/**
* @ingroup driver
* @brief Logical id to physical id
* @attention NULL
* @param [in] logicid logic id
* @param [out] phyid   physic id
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_phyid_from_logicid(unsigned int logicid, unsigned int *phyid);

/**
* @ingroup driver
* @brief physical id to Logical id
* @attention NULL
* @param [in] phyid   physical id
* @param [out] logicid logic id
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_logicid_from_phyid(unsigned int phyid, unsigned int *logicid);

/**
* @ingroup driver
* @brief Query the overall health status of the device, support AI Server
* @attention NULL
* @param [in] device_id  The device id
* @param [out] phealth  The pointer of the overall health status of the device only represents this component,
                        and does not include other components that have a logical relationship with this component.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_health(int device_id, unsigned int *phealth);

/**
* @ingroup driver
* @brief Query device fault code
* @attention NULL
* @param [in] device_id  The device id
* @param [out] errorcount  Number of error codes
* @param [out] perrorcode  error codes
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_errorcode(int device_id, int *errorcount, unsigned int *perrorcode);

/**
* @ingroup driver
* @brief Query the temperature of the ICE SOC of Shengteng AI processor
* @attention NULL
* @param [in] device_id  The device id
* @param [out] ptemperature  The temperature of the HiSilicon SOC of the Shengteng AI processor: unit Celsius,
                         the accuracy is 1 degree Celsius, and the decimal point is rounded. 16-bit signed type,
                         little endian. The value returned by the device is the actual temperature.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_temperature(int device_id, int *ptemperature);

/**
* @ingroup driver
* @brief Query device power consumption
* @attention NULL
* @param [in] device_id  The device id
* @param [out] schedule  Device power consumption: unit is W, accuracy is 0.1W. 16-bit unsigned short type,
               little endian
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_power_info(int device_id, struct dsmi_power_info_stru *pdevice_power_info);

/**
* @ingroup driver
* @brief Query PCIe device information
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pcie_idinfo  PCIe device information
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_pcie_info(int device_id, struct tag_pcie_idinfo *pcie_idinfo);

/**
* @ingroup driver
* @brief Query the voltage of Sheng AI SOC of ascend AI processor
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pvoltage  The voltage of the HiSilicon SOC of the Shengteng AI processor: the unit is V,
                         and the accuracy is 0.01V
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_voltage(int device_id, unsigned int *pvoltage);

/**
* @ingroup driver
* @brief Get the occupancy rate of the HiSilicon SOC of the Ascension AI processor
* @attention NULL
* @param [in] device_id  The device id
* @param [in] device_type  device_type
* @param [out] putilization_rate  Utilization rate of HiSilicon SOC of ascend AI processor, unit:%
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_utilization_rate(int device_id, int device_type, unsigned int *putilization_rate);

/**
* @ingroup driver
* @brief Get the frequency of the HiSilicon SOC of the Ascension AI processor
* @attention NULL
* @param [in] device_id  The device id
* @param [out] device_type  device_type
* @param [out] pfrequency  Frequency, unit MHZ
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_frequency(int device_id, int device_type, unsigned int *pfrequency);

/**
* @ingroup driver
* @brief Get the number of Flash
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pflash_count Returns the number of Flash, currently fixed at 1
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_flash_count(int device_id, unsigned int *pflash_count);

/**
* @ingroup driver
* @brief Get flash device information
* @attention NULL
* @param [in] device_id  The device id
* @param [in] flash_index Flash index number. The value is fixed at 0.
* @param [out] pflash_info Returns Flash device information.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_flash_info(int device_id, unsigned int flash_index, dm_flash_info_stru *pflash_info);

/**
* @ingroup driver
* @brief Get memory information
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pdevice_memory_info  Return memory information
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_memory_info(int device_id, struct dsmi_memory_info_stru *pdevice_memory_info);

/**
* @ingroup driver
* @brief Get ECC information
* @attention NULL
* @param [in] device_id  The device id
* @param [in] device_type  device type
* @param [out] pdevice_ecc_info  return ECC information
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_ecc_info(int device_id, int device_type, struct dsmi_ecc_info_stru *pdevice_ecc_info);

/**
* @ingroup driver
* @brief Message transfer interface implementation
* @attention NULL
* @param [in] device_id  The device id
* @param [out] passthru_message  passthru_message_stru struct
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_passthru_mcu(int device_id, struct passthru_message_stru *passthru_message);

/**
* @ingroup driver
* @brief Query device fault description
* @attention NULL
* @param [in] device_id  The device id
* @param [in] schedule  Error code to query
* @param [out] perrorinfo Corresponding error character description
* @param [out] buffsize The buff size brought in is fixed at 48 bytes. If the set buff size is greater
                        than 48 bytes, the default is 48 bytes
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_query_errorstring(int device_id, unsigned int errorcode, unsigned char *perrorinfo, int buffsize);

/**
* @ingroup driver
* @brief Get board information, including board_id, pcb_id, bom_id, slot_id version numbers of the board
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pboard_info  return board info
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_board_info(int device_id, struct dsmi_board_info_stru *pboard_info);

/**
* @ingroup driver
* @brief Get system time
* @attention NULL
* @param [in] device_id  The device id
* @param [out] ntime_stamp  the number of seconds from 00:00:00, January 1,1970.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_system_time(int device_id, unsigned int *ntime_stamp);

/**
* @ingroup driver
* @brief config device ecc
* @attention NULL
* @param [in] device_id  The device id
* @param [in] device_type  the DSMI_DEVICE_TYPE.
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_config_ecc_enable(int device_id, DSMI_DEVICE_TYPE device_type, int enable_flag);

/**
* @ingroup driver
* @brief get ecc enable status
* @attention NULL
* @param [in] device_id  The device id
* @param [in] device_type  the DSMI_DEVICE_TYPE.
* @param [out] enable_flag  flag value.
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_get_ecc_enable(int device_id, DSMI_DEVICE_TYPE device_type, int *enable_flag);


/**
* @ingroup driver
* @brief Set the MAC address of the specified device
* @attention NULL
* @param [in] device_id  The device id
* @param [in] mac_id Specify MAC, value range: 0 ~ dsmi_get_mac_count interface output
* @param [in] pmac_addr Set a 6-byte MAC address.
* @param [in] mac_addr_len  MAC address length, fixed length 6, unit byte.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_set_mac_addr(int device_id, int mac_id, const char *pmac_addr, unsigned int mac_addr_len);

/**
* @ingroup driver
* @brief Query the number of MAC addresses
* @attention NULL
* @param [in] device_id  The device id
* @param [out] count Query the MAC number, the value range: 0 ~ 4.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_mac_count(int device_id, int *count);

/**
* @ingroup driver
* @brief Get the MAC address of the specified device
* @attention NULL
* @param [in] device_id  The device id
* @param [in] mac_id Specify MAC, value range: 0 ~ dsmi_get_mac_count interface output
* @param [out] pmac_addr return a 6-byte MAC address.
* @param [in] mac_addr_len  MAC address length, fixed length 6, unit byte.
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_mac_addr(int device_id, int mac_id, char *pmac_addr, unsigned int mac_addr_len);

/**
* @ingroup driver
* @brief Set the ip address and mask address.
* @attention NULL
* @param [in] device_id  The device id
* @param [in] port_type  Specify the network port type
* @param [in] port_id  Specify the network port number, reserved field
* @param [in] ip_address  ip address info wants to set
* @param [in] mask_address  mask address info wants to set
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_set_device_ip_address(int device_id, int port_type, int port_id, ip_addr_t ip_address, ip_addr_t mask_address);

/**
* @ingroup driver
* @brief get the ip address and mask address.
* @attention NULL
* @param [in] device_id  The device id
* @param [in] port_type  Specify the network port type
* @param [in] port_id  Specify the network port number, reserved field
* @param [out] ip_address  return ip address info
* @param [out] mask_address  return mask address info
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_ip_address(int device_id, int port_type, int port_id, ip_addr_t *ip_address,
    ip_addr_t *mask_address);

/**
* @ingroup driver
* @brief get device fan number
* @attention NULL
* @param [in] device_id  The device id
* @param [out] count  fan count.
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_get_fan_count(int device_id, int *count);

/**
* @ingroup driver
* @brief get device fanspeed.
* @attention NULL
* @param [in] device_id  The device id
* @param [in] fan_id  Specify the fan port number,reserved field.
* @param [out] speed  fan speed
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_get_fan_speed(int device_id, int fan_id, int *speed);


/**
* @ingroup driver
* @brief send pre reset to device soc.
* @attention NULL
* @param [in] device_id  The device id
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_pre_reset_soc(int device_id);

/**
* @ingroup driver
* @brief send re scan soc.
* @attention NULL
* @param [in] device_id  The device id
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_rescan_soc(int device_id);

/**
* @ingroup driver
* @brief Reset the HiSonic SOC of the designated Ascent AI processor
* @attention NULL
* @param [in] device_id  The device id
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_hot_reset_soc(int device_id);

/**
* @ingroup driver
* @brief Get the startup state of the HiSilicon SOC of the Ascend AI processor
* @attention NULL
* @param [in] device_id  The device id
* @param [out] boot_status The startup state of the HiSilicon SOC of the Ascend AI processor
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_boot_status(int device_id, enum dsmi_boot_status *boot_status);

/**
* @ingroup driver
* @brief Relevant information about the HiSilicon SOC of the AI ??processor, including chip_type, chip_name,
         chip_ver version number
* @attention NULL
* @param [in] device_id  The device id
* @param [out] chip_info  Get the relevant information of ascend AI processor Hisilicon SOC
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_chip_info(int device_id, struct dsmi_chip_info_stru *chip_info);

/**
* @ingroup driver
* @brief Get SOC sensor information
* @attention NULL
* @param [in] device_id The device id
* @param [in] sensor_id Specify sensor index
* @param [out] Returns the value that needs to be obtained
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_soc_sensor_info(int device_id, int sensor_id, TAG_SENSOR_INFO *tsensor_info);


/**
* @ingroup driver
* @brief Query the gateway address.
* @attention NULL
* @param [in] device_id  The device id
* @param [in] port_type  Specify the network port type
* @param [in] port_id  Specify the network port number, reserved field
* @param [out] gtw_address  return gateway address info
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_gateway_addr(int device_id, int port_type, int port_id, ip_addr_t *gtw_address);

/**
* @ingroup driver
* @brief  get mini I2C heartbeat for mini to mcu.
* @attention NULL
* @param [in] device_id  The device id
* @param [out] status  heartbeat status
* @return  0 for success, others for fail
* @note Support:Ascend310
*/
DLLEXPORT int dsmi_get_mini2mcu_heartbeat_status(int device_id, unsigned char *status, unsigned int *disconn_cnt);

/**
* @ingroup driver
* @brief Query the frequency, capacity and utilization information of hbm
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pdevice_hbm_info return hbm infomation
* @return  0 for success, others for fail
* @note Support:Ascend910
*/
DLLEXPORT int dsmi_get_hbm_info(int device_id, struct dsmi_hbm_info_stru *pdevice_hbm_info);

/**
* @ingroup driver
* @brief Query the frequency and utilization information of aicore
* @attention NULL
* @param [in] device_id  The device id
* @param [out] pdevice_aicore_info  return aicore information
* @return  0 for success, others for fail
* @note Support:Ascend910
*/
DLLEXPORT int dsmi_get_aicore_info(int device_id, struct dsmi_aicore_info_stru *pdevice_aicore_info);

/**
* @ingroup driver
* @brief Query the connectivity status of the RoCE network card's IP address
* @attention NULL
* @param [in] device_id The device id
* @param [out] presult return the result wants to query
* @return  0 for success, others for fail
* @note Support:Ascend910
*/
DLLEXPORT int dsmi_get_network_health(int device_id, DSMI_NET_HEALTH_STATUS *presult);

/**
* @ingroup driver
* @brief Get the ID of the board
* @attention NULL
* @param [in] device_id The device id
* @param [out] board_id Board ID. In the AI ??Server scenario, the value is 0
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_board_id(int device_id, unsigned int *board_id);

/**
* @ingroup driver
* @brief Query LLC performance parameters, including LLC read hit rate, write hit rate, and throughput
* @attention NULL
* @param [in] device_id  The device id
* @param [out] perf_para  LLC performance parameter information, including LLC read hit rate,
                         write hit rate and throughput
* @return  0 for success, others for fail
* @note Support:Ascend910
*/
DLLEXPORT int dsmi_get_llc_perf_para(int device_id, DSMI_LLC_PERF_INFO *perf_para);

/**
* @ingroup driver
* @brief Query the number, maximum operating frequency, current operating frequency and utilization rate of AICPU.
* @attention NULL
* @param [in] device_id  The device id
* @param [out] schedule  return the value wants to query
* @return  0 for success, others for fail
* @note Support:Ascend910
*/
DLLEXPORT int dsmi_get_aicpu_info(int device_id, struct dsmi_aicpu_info_stru *pdevice_aicpu_info);

/**
* @ingroup driver
* @brief get user configuration
* @attention NULL
* @param [in] device_id  The device id
* @param [in] config_name Configuration item name, the maximum string length of the
                          configuration item name is 32
* @param [in] buf_size buf length, the maximum length is 1024 byte
* @param [out] buf  buf pointer to the content of the configuration item
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_user_config(int device_id, const char *config_name, unsigned int buf_size, unsigned char *buf);

/**
* @ingroup driver
* @brief set user configuration
* @attention NULL
* @param [in] device_id  The device id
* @param [in] config_name Configuration item name, the maximum string length of the
                          configuration item name is 32
* @param [in] buf_size buf length, the maximum length is 1024 byte
* @param [in] buf  buf pointer to the content of the configuration item
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_set_user_config(int device_id, const char *config_name, unsigned int buf_size, unsigned char *buf);

/**
* @ingroup driver
* @brief clear user configuration
* @attention NULL
* @param [in] device_id  The device id
* @param [in] config_name Configuration item name, the maximum string length of the
                          configuration item name is 32
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_clear_user_config(int device_id, const char *config_name);

/**
* @ingroup driver
* @brief Get the DIE ID of the specified device
* @attention NULL
* @param [in] device_id  The device id
* @param [out] schedule  return die id infomation
* @return  0 for success, others for fail
* @note Support:Ascend310,Ascend910
*/
DLLEXPORT int dsmi_get_device_die(int device_id, struct dsmi_soc_die_stru *pdevice_die);

/**
 * @ingroup driver
 * @brief: revocation for different type of operation
 * @param [in] device_id device id
 * @param [in] revo_type revocation type,only support for DSMI_REVOCATION_TYPE_SOC.
 * @param [in] file_data revocation file data
 * @param [in] file_size file data size for revocation
 * @return  0 for success, others for fail
 * @note Support:Ascend910
 */
DLLEXPORT int dsmi_set_sec_revocation(int device_id, DSMI_REVOCATION_TYPE revo_type, const unsigned char *file_data,
    unsigned int file_size);

/**
 * @ingroup driver
 * @brief: control systems sleep state
 * @param [in] device_id device id, not userd  default 0
 * @param [in] type determine the system to different sleep type
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_set_power_state(int device_id, DSMI_POWER_STATE type);

/**
 * @ingroup driver
 * @brief: get emmc status info
 * @param [in] device_id device id
 * @param [out] emmc_status_data return the value of emmc status info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_emmc_status(int device_id, struct dsmi_emmc_status_stru *emmc_status_data);

/**
 * @ingroup driver
 * @brief: get can status info
 * @param [in] device_id device id
 * @param [out] can_status_data return the value of can status info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_can_status(int device_id, const char *name, unsigned int name_len,
    struct dsmi_can_status_stru *can_status_data);

/**
 * @ingroup driver
 * @brief: get ufs status info
 * @param [in] device_id device id
 * @param [out] ufs_status_data return the value of ufs status info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_ufs_status(int device_id, struct dsmi_ufs_status_stru *ufs_status_data);

/**
 * @ingroup driver
 * @brief: get sensorhub status info
 * @param [in] device_id device id
 * @param [out] sensorhub_status_data return the value of sensorhub status info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_sensorhub_status(int device_id, struct dsmi_sensorhub_status_stru *sensorhub_status_data);

/**
 * @ingroup driver
 * @brief: get can config info
 * @param [in] device_id device id
 * @param [out] can_config_data return the value of can config info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_can_config(int device_id, const char *name, unsigned int name_len,
    struct dsmi_can_config_stru *can_config_data);

/**
 * @ingroup driver
 * @brief: get ufs config info
 * @param [in] device_id device id
 * @param [out] ufs_config_data return the value of ufs config info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_ufs_config(int device_id, struct dsmi_ufs_config_stru *ufs_config_data);

/**
 * @ingroup driver
 * @brief: set ufs config info
 * @param [in] device_id device id
 * @param [in] ufs_config_data return the value of ufs config info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_set_ufs_config(int device_id, struct dsmi_ufs_config_stru *ufs_config_data);

/**
 * @ingroup driver
 * @brief: get sensorhub config info
 * @param [in] device_id device id
 * @param [out] sensorhub_config_data return the value of sensorhub config info
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_sensorhub_config(int device_id, struct dsmi_sensorhub_config_stru *sensorhub_config_data);

/**
 * @ingroup driver
 * @brief: get gpio value
 * @param [in] device_id device id
 * @param [in] gpio_num gpio_num
 * @param [out] status return the value of gpio value
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_gpio_status(int device_id, unsigned int gpio_num, unsigned int *status);

/**
 * @ingroup driver
 * @brief: get hiss status info
 * @param [in] device_id device id, not userd  default 0
 * @param [out] hiss_status_data hiss status infomation
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_hiss_status(int device_id, struct dsmi_hiss_status_stru *hiss_status_data);

/**
 * @ingroup driver
 * @brief: get lp system status info
 * @param [in] device_id device id
 * @param [out] lp_status_data  lp system status information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_lp_status(int device_id, struct dsmi_lp_status_stru *lp_status_data);

/**
 * @ingroup driver
 * @brief: get vector core info
 * @param [in] device_id device id
 * @param [out] pdevice_aicore_info vector core information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_vectorcore_info(int device_id, struct dsmi_vector_info_stru *pdevice_aicore_info);

/**
 * @ingroup driver
 * @brief: get uart status info
 * @param [in] device_id device id
 * @param [in] name uart name
 * @param [in] name_len name length
 * @param [out] uart_status_data dsmi uart status information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_uart_status(int device_id, const char *name,  unsigned char name_len,
    struct dsmi_uart_status_stru *uart_status_data);

/**
 * @ingroup driver
 * @brief: get vector core info
 * @param [in] device_id device id
 * @param [in] name name
 * @param [in] namelen name length
 * @param [out] uart_config_data dsmi uart config information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_set_uart_config(int device_id, const char *name, unsigned char name_len,
    struct dsmi_uart_config_stru *uart_config_data);

/**
 * @ingroup driver
 * @brief: get soc hardware fault info
 * @param [in] device_id device id
 * @param [out] emu_subsys_state_data dsmi emu subsys status information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_sochwfault(int device_id, struct dsmi_emu_subsys_state_stru *emu_subsys_state_data);

/**
 * @ingroup driver
 * @brief: get safetyisland status info
 * @param [in] device_id device id
 * @param [out] safetyisland_status_data dsmi safetyisland status information.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_safetyisland_status(int device_id, struct dsmi_safetyisland_status_stru *safetyisland_status_data);

/**
 * @ingroup driver
 * @brief: register fault event handler
 * @param [in] device_id device id
 * @param [int] handler fault event callback func.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_register_fault_event_handler(int device_id, fault_event_handler handler);

 /**
 * @ingroup driver
 * @brief: set cpu number configuration, including ctrl cpu/data cpu/ai cpu
 * @param [in] device_id device id
 * @param [in] cpu_cfg cpu number configuration, including number of ctrl cpu/data cpu/ai cpu.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_set_cpu_nums(int device_id, struct dsmi_cpu_cfg_stru cpu_cfg);

/**
 * @ingroup driver
 * @brief: get cpu number configuration, including ctrl cpu/data cpu/ai cpu
 * @param [in] device_id device id
 * @param [out] cpu_cfg cpu number configuration, including number of ctrl cpu/data cpu/ai cpu.
 * @return  0 for success, others for fail
 * @note Support:Reserve
 */
DLLEXPORT int dsmi_get_cpu_nums(int device_id, struct dsmi_cpu_cfg_stru *cpu_cfg);

/**
 * @ingroup driver
 * @brief: get device cgroup info, including limit_in_bytes/max_usage_in_bytes/usage_in_bytes.
 * @param [in] device_id device id
 * @param [out] cgroup info limit_in_bytes/max_usage_in_bytes/usage_in_bytes.
 * @return  0 for success, others for fail
 * @note Support:Ascend310,Ascend910
 */
DLLEXPORT int dsmi_get_device_cgroup_info(int device_id, struct tag_cgroup_info *cg_info);

/**
 * @ingroup driver
 * @brief: get device information
 * @param [in] device_id device id
 * @param [in] main_cmd main command type for device information
 * @param [in] sub_cmd sub command type for device information
 * @param [in] buf input buffer
 * @param [in] buf_size buffer size
 * @return  0 for success, others for fail
 * @note Support:Ascend910,Ascend310
 */
DLLEXPORT int dsmi_set_device_info(unsigned int device_id, DSMI_MAIN_CMD main_cmd, unsigned int sub_cmd,
    const void *buf, unsigned int buf_size);

/**
 * @ingroup driver
 * @brief: get device information
 * @param [in] device_id device id
 * @param [in] main_cmd main command type for device information
 * @param [in] sub_cmd sub command type for device information
 * @param [out] buf output buffer
 * @param [in out] size input buffer size and output data size
 * @return  0 for success, others for fail
 * @note Support:Ascend910,Ascend310
 */
DLLEXPORT int dsmi_get_device_info(unsigned int device_id, DSMI_MAIN_CMD main_cmd, unsigned int sub_cmd,
    void *buf, unsigned int *size);

/**
* @ingroup driver
* @brief create ts group
* @attention null
* @param [in]  devId device id
* @param [in]  ts_id ts id 0 : TS_AICORE, 1 : TS_AIVECTOR
* @param [out]  info ts group info
* @return  0 for success, others for fail
* @note Support:Reserve
*/
DLLEXPORT int dsmi_create_capability_group(int device_id, int ts_id,
                                 struct dsmi_capability_group_info *group_info);

/**
* @ingroup driver
* @brief delete ts group
* @attention null
* @param [in]  devId device id
* @param [in]  ts_id ts id 0 : TS_AICORE, 1 : TS_AIVECTOR
* @param [in]  group_id group id
* @return  0 for success, others for fail
* @note Support:Reserve
*/
DLLEXPORT int dsmi_delete_capability_group(int device_id, int ts_id, int group_id);

/**
* @ingroup driver
* @brief get ts group info
* @attention null
* @param [in]  devId device id
* @param [in]  ts_id ts id 0 : TS_AICORE, 1 : TS_AIVECTOR
* @param [in]  group_id group id
* @param [in]  group_count group count
* @param [out]  info ts group info
* @return  0 for success, others for fail
* @note Support:Reserve
*/
DLLEXPORT int dsmi_get_capability_group_info(int device_id, int ts_id, int group_id,
                                   struct dsmi_capability_group_info *group_info, int group_count);

#ifdef __cplusplus
}
#endif
#endif
