time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_imageProjection | tee "./log/${time}_img_proj.log"
