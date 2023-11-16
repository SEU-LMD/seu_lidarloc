#TODO 1111 TODO add MDC cmake  --done
set(SRC_DIRS  ${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc)
set(SHARED_INC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc/include/shared/include)
set(GENERATED_DIRS  ${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc/include/generated/include)
set(AP_INCLUDE_DIRS  ${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc/include/ap/include)
set(MDC_ACLLIB_SDK  ${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc/include/acllib)
set(UDP_DIRS $${CMAKE_CURRENT_SOURCE_DIR}/pubsub/mdc/include/aarch64-linux-gnu)






set(Communication_LIBS  ${CMAKE_CURRENT_SOURCE_DIR}/../env/lib/libposition.a
        /opt/platform/mdc_platform/lib/liblogging.so.1
        /opt/platform/mdc_platform/lib/libplog.so.1
        /opt/platform/mdc_platform/lib/liblog.so.1
        /opt/platform/mdc_platform/lib/libsecurec.so
        /lib/aarch64-linux-gnu/libpthread.so.0 /opt/platform/mdc_platform/lib/libara_com.so.3.10.7
        /opt/platform/mdc_platform/lib/libyaml-cpp.so.0.6
        /opt/platform/mdc_platform/lib/libadsf.so
        /opt/platform/mdc_platform/lib/libadb_host.so
        /opt/platform/mdc_platform/lib/libvrtf_vcc.so.3.10.7
        /opt/platform/mdc_platform/lib/libE2EXf_CM.so.1
        /opt/platform/mdc_platform/lib/libara_core.so.1
        /usr/lib/aarch64-linux-gnu/libstdc++.so.6
        /lib/aarch64-linux-gnu/libgcc_s.so.1
        /lib/aarch64-linux-gnu/libc.so.6
        /lib/ld-linux-aarch64.so.1
        /opt/platform/mdc_platform/lib/libvcc_ddsdriver.so.3.10.7
        /opt/platform/mdc_platform/lib/libvcc_someipdriver.so.3.10.7
        /opt/platform/mdc_platform/lib/libJsonParser.so.1
        /opt/platform/mdc_platform/lib/librtf_cm.so.3.10.7
        /lib/aarch64-linux-gnu/libm.so.6
        /opt/platform/mdc_platform/lib/libdp_adapter.so
        /opt/platform/mdc_platform/lib/libE2EXf_Static.so.1
        /opt/platform/mdc_platform/lib/libddscpp.so.1
        /opt/platform/mdc_platform/lib/libsomeip.so
        /lib/aarch64-linux-gnu/librt.so.1
        /lib/aarch64-linux-gnu/libdl.so.2
        /opt/platform/mdc_platform/lib/libE2ELib.so.1
        /opt/platform/mdc_platform/lib/libddscore.so.1
        /opt/platform/mdc_platform/lib/libCrc.so.1
        /opt/platform/mdc_platform/lib/libhwcrc.so.1
        /opt/platform/mdc_platform/lib/libxpcshm.so.1
)

set(Communication_DIR
        ${SRC_DIRS}
        ${SHARED_INC_DIR}
        ${GENERATED_DIRS}
        ${AP_INCLUDE_DIRS}
        ${AP_INCLUDE_DIRS}/adsfi/adb/include
        ${AP_INCLUDE_DIRS}/adsfi/adsf/include
        ${AP_INCLUDE_DIRS}/adsfi/arxml_include
        #4
        ${MDC_ACLLIB_SDK}/include
        ${MDC_ACLLIB_SDK}/include/acl
        ${MDC_ACLLIB_SDK}/include/acl/ops
        #5
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/driver/include
        #6
        ${UDP_DIRS}
)


