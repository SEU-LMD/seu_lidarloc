#ifndef DVPP_DVPPCOMMON_H
#define DVPP_DVPPCOMMON_H
#include <cstdint>
#include "ExportMacro.h"
#include "securec.h"

const int32_t DVPP_SUCCESS = 0;
const int32_t DVPP_FAIL = -1;
const int32_t DVPP_CTL_VPC_PROC = 0;
const int32_t DVPP_CTL_PNGD_PROC = 1;
const int32_t DVPP_CTL_JPEGE_PROC = 2;
const int32_t DVPP_CTL_JPEGD_PROC = 3;
const int32_t DVPP_CTL_VDEC_PROC = 4;
const int32_t DVPP_CTL_DVPP_CAPABILITY = 6;
const int32_t DVPP_CTL_CMDLIST_PROC = 7;
const int32_t DVPP_CTL_TOOL_CASE_GET_RESIZE_PARAM = 8;
const uint32_t API_MAP_VA32BIT = 0x200; // 32Bit
const int32_t PAGE_SIZE = 4096; // Page Size
const int32_t MAP_2M = 2097152; // 2* 1024 * 1024
#ifndef ALIGN_UP
#define ALIGN_UP(x, align) ((((x) + ((align)-1))/(align))*(align))
#endif

struct dvppapi_ctl_msg {
    int32_t in_size; // Input Parameter Size
    int32_t out_size; // Output Parameter Size
    void* in; // Input Parameter
    void* out; // Output Parameter
    dvppapi_ctl_msg()
    {
        in_size = -1;
        out_size = -1;
        in = nullptr;
        out = nullptr;
    }
};

class IVPCAPI;
class IVDECAPI;
class IDVPPCAPABILITY;
class IDVPPAPI {
public:
    virtual ~IDVPPAPI(void) {}

    IVPCAPI* pIVPCAPI;
    IVDECAPI* pIVDECAPI;
    IDVPPCAPABILITY* pIDVPPCAPABILITY;
};

typedef enum {
    VDEC_SUB_PROC_SCD  = 0,
    VDEC_SUB_PROC_VDM
} VDEC_SUB_PROC_TYPE;

//common class
class AutoBuffer {
public:
    AutoBuffer();
    ~AutoBuffer();

    void Reset();

    char* allocBuffer(int32_t size);

    char* getBuffer() const { return dest_buffer; }
    // 128-aligned width, 16-aligned height. For example, image is 224*220, getBufferSize=256*224.
    int32_t getBufferSize() { return real_bufferSize; }

    char* dest_buffer;

private:
    char* src_buffer;
    int32_t real_bufferSize; // 128-aligned width, 16-aligned height.
    AutoBuffer(const AutoBuffer&); // no copyable to avoid double free.
    const AutoBuffer& operator= (const AutoBuffer&); // no copyable.
};
#endif // DVPP_DVPPCOMMON_H