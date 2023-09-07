time=$(date "+%Y-%m-%d %H:%M:%S")
./build/imageProjection | tee "./log/${time}_img_proj.log"
