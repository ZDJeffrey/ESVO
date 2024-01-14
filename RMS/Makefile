rpg_bin_error:
	@evo_traj bag rpg_bin/evaluate_rpg_bin.bag /esvo_tracking/pose_pub --save_as_tum --plot_mode xyz
	@./evaluate_rpe.py bag/rpg_bin_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/rpg_bin_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
rpg_boxes_error:
	@evo_traj bag rpg_boxes/evaluate_rpg_boxes.bag /esvo_tracking/pose_pub --save_as_tum
	@./evaluate_rpe.py bag/rpg_boxes_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/rpg_boxes_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
rpg_desk_error:
	@evo_traj bag rpg_desk/evaluate_rpg_desk.bag /esvo_tracking/pose_pub --save_as_tum
	@./evaluate_rpe.py bag/rpg_desk_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/rpg_desk_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
rpg_monitor_error:
	@evo_traj bag rpg_monitor/evaluate_rpg_monitor.bag /esvo_tracking/pose_pub --save_as_tum
	@./evaluate_rpe.py bag/rpg_monitor_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/rpg_monitor_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
upenn1_error:
	@evo_traj bag upenn1/evaluate_upenn1.bag  /esvo_tracking/pose_pub --save_as_tum --plot_mode xyz
	@./evaluate_rpe.py bag/upenn1_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/upenn1_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
upenn3_error:
	@evo_traj bag upenn3/evaluate_upenn3.bag  /esvo_tracking/pose_pub --save_as_tum --plot_mode xyz
	@./evaluate_rpe.py bag/upenn3_groundtruth.tum esvo_tracking_pose_pub.tum --verbose --delta 1.0 --delta_unit s --fixed_delta
	@python2 ./evaluate_ate.py bag/upenn3_groundtruth.tum esvo_tracking_pose_pub.tum --verbose
	@rm -f *.tum
upenn1_depth_error:
	@python ./upenn1_depth_error.py
upenn3_depth_error:
	@python ./upenn3_depth_error.py
# rpg_bin_ape:
# 	evo_ape bag rpg_bin/evaluate_rpg_bin.bag  /optitrack/davis_stereo /esvo_tracking/pose_pub --pose_relation trans_part -a --verbose --plot_mode xyz
# rpg_boxes_ape:
# 	evo_ape bag rpg_boxes/evaluate_rpg_boxes.bag  /optitrack/davis_stereo /esvo_tracking/pose_pub --pose_relation trans_part -a --verbose --plot_mode xyz
# rpg_desk_ape: 
# 	evo_ape bag rpg_desk/evaluate_rpg_desk.bag  /optitrack/davis_stereo /esvo_tracking/pose_pub --pose_relation trans_part -a --verbose --plot_mode xyz
# rpg_monitor_ape:
# 	evo_ape tum bag/rpg_monitor_groundtruth.tum  rpg_monitor/esvo_tracking_pose_pub.tum --pose_relation trans_part -a --verbose --plot_mode xyz -p
# upenn1_ape:
# 	evo_ape bag upenn1/evaluate_upenn1.bag  /davis/left/pose /esvo_tracking/pose_pub --pose_relation trans_part -a --verbose --plot_mode xyz
# upenn3_ape:
# 	evo_ape bag upenn3/evaluate_upenn3.bag  /davis/left/pose /esvo_tracking/pose_pub --pose_relation trans_part -a --verbose --plot_mode xyz

clean:
	rm -rf *.tum