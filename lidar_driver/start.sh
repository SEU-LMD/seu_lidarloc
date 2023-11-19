#!/usr/bin/env bash

export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export CM_CONFIG_FILE_PATH=./etc/
export RT_DDS_URI=./etc/dds.xml
if [ ! -d "etc" ];then
    slot_id=$(cat /proc/cmdline | awk -F 'slotid=' '{print $2}'|cut -d' ' -f1|cut -b 2)
    if [ ! ${slot_id} ]
    then
        cp -r /opt/platform/mdc_platform/manual_service/adsfi/lidar_driver_base/etc ./
    else
        cp -rf /opt/platform/mdc_platform/manual_service/adsfi/lidar_driver_base/mini${slot_id}/etc ./
    fi
fi
chmod +x $1
$1
