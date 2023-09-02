time=$(date "+%Y-%m-%d %H:%M:%S")
./map_engine/build/serialize | tee "./log/${time}_serialize.log"