rm -r ./Remote_MDC
mkdir Remote_MDC
sshfs mdc@192.168.1.6:/home/mdc/seu_wxq/autodrive_mdc/seu_lidarloc ./Remote_MDC/
rm -rf ./Remote_MDC/*
cp -rf ./data_check_res ./Remote_MDC/
cp -rf ./env ./Remote_MDC/
cp -rf ./map_data ./Remote_MDC/
cp -rf ./src ./Remote_MDC/
sshpass -p 'mdcOs_123' ssh -tt mdc@192.168.1.6

