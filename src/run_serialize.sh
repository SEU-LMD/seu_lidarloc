time=$(date "+%Y-%m-%d %H:%M:%S")
./app_mapserilize/build/serialize | tee "./log/${time}_serialize.log"