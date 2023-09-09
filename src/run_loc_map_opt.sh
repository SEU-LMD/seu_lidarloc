time=$(date "+%Y-%m-%d %H:%M:%S")
./build/loc_mapOptmizationGps | tee "./log/${time}_loc_map_opt.log"
