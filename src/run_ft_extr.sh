time=$(date "+%Y-%m-%d %H:%M:%S")
./build/devel/lib/lio_sam_6axis/lio_sam_6axis_featureExtraction | tee "./log/${time}_ft_extr.log"