time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_LocOptmization | tee "./log/${time}_Loc_map_opt.log"
