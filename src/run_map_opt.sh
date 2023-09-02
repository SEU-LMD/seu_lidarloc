time=$(date "+%Y-%m-%d %H:%M:%S")
./build/mapOptmization | tee "./log/${time}_map_opt.log"
