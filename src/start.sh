#!/usr/bin/env bash

export LD_LIBRARY_PATH=/opt/platform/mdc_platform/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/mdc/seu_wxq/autodrive_mdc/seu_lidarloc/env/pcl-1.10.0-install/lib:$LD_LIBRARY_PATH
export CM_CONFIG_FILE_PATH=./etc/
export RT_DDS_URI=./etc/dds.xml
if [ ! -d "etc" ];then
    cp -r /opt/platform/mdc_platform/manual_service/adsfi/position_base/etc ./
fi
chmod +x $1
$1|bash run_loc.sh

