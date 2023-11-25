python3 evo/main_traj.py bag $1 /gnss_odom_world --save_as_tum

python3 evo/main_traj.py bag $1 /loc_result --save_as_tum

python3 evo/main_ape.py tum loc_result.tum gnss_odom_world.tum -p -v -as plot_mode xy


