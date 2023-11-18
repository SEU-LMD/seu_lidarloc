time=$(date "+%Y-%m-%d %H:%M:%S")
#TODO 1118 mkdir gnss
./build/mapping | tee "./log/${time}_mapping.log"
