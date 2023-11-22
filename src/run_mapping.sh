time=$(date "+%Y-%m-%d %H:%M:%S")
#TODO 1118 mkdir gnss
mkdir flag_gnss
./build/mapping | tee "./log/${time}_mapping.log"
