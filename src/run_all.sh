mkdir log

gnome-terminal --window -e 'bash -c "rviz -d ./config/vlp.rviz"' \
--tab -e 'bash -c "./run_ft_extr.sh"' \
--tab -e 'bash -c "./run_gps_odo.sh"' \
--tab -e 'bash -c "./run_img_proj.sh"' \
--tab -e 'bash -c "./run_imu_pre.sh"' \
--tab -e 'bash -c "./run_map_opt.sh"' 

