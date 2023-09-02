time=$(date "+%Y-%m-%d %H:%M:%S")
./build/imuPreintegration | tee "./log/${time}_imu_pre.log"
