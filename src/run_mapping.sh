time=$(date "+%Y-%m-%d %H:%M:%S")
./build/mapping | tee "./log/${time}_mapping.log"
