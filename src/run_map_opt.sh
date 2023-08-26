time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_mapOptmization | tee "./log/${time}_map_opt.log"
