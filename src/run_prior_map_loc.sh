mkdir log

gnome-terminal --window -e 'bash -c "rviz -d ./config/priorMap.rviz"' \
--tab -e 'bash -c "./run_prior_map.sh"'

