time=$(date "+%Y-%m-%d %H:%M:%S")
./build/loc | tee "./log/${time}_loc.log"
