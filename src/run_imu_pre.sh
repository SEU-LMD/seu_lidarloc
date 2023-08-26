time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_imuPreintegration | tee "./log/${time}_imu_pre.log"
