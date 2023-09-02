time=$(date "+%Y-%m-%d %H:%M:%S")
./build/featureExtraction | tee "./log/${time}_ft_extr.log"