#!/usr/bin/env python
""""
@author: allenc1 

Formats generated walker planning queries to yaml format 
"""

import yaml 
import argparse 
import numpy as np 

from os.path import join

class WalkerExperimentEditor():

	def __init__(self, f_start_configs, f_goal_poses, f_goal_configs=None, 
		f_crit_configs=None): 

		# walker specific params
		self.walker_state_names = ["x", "y", "theta", "right_j1", "right_j2",
		    "right_j3", "right_j4", "right_j5", "right_j6", "right_j7"]

		self.using_goal_states = True if f_goal_configs is not None else False
		if self.using_goal_states:
			self.goal_configs = np.loadtxt(f_goal_configs)		

		self.using_crit_states = True if f_crit_configs is not None else False						
		if self.using_crit_states: 
			self.crit_configs = np.loadtxt(f_crit_configs, delimiter=",")

		self.start_configs = np.loadtxt(f_start_configs)
		self.goal_poses = np.loadtxt(f_goal_poses)

	def _format_pose(self, pose_vals): 
		pose_vals = [float(v) for v in pose_vals]
		return {'x':pose_vals[0], 'y':pose_vals[1], 'z':pose_vals[2], 
		    'roll':pose_vals[3], 'pitch':pose_vals[4], 'yaw':pose_vals[5]}

	def _format_config(self, state_vals):
		state_vals = [float(v) for v in state_vals]
		return {'joint_state':[{"name":n, "position":i} for n, i in \
		    zip(self.walker_state_names, state_vals)]}

	def _format_crit_configs(self, crit_states):
		to_float_list = lambda x : [float(e) for e in x]
		crit_states = [to_float_list(crit_states[i,:]) for i in \
		    xrange(len(crit_states))]
		return crit_states

	def generate_formatted_experiments(self, exp_dir, exp_label, topk_crit=0):
		for i in xrange(len(self.start_configs)):
			f_exp = join(exp_dir, "{}{}.yaml".format(exp_label, str(i)))

			d_start_config = self._format_config(self.start_configs[i,:])
			d_goal_pose = self._format_pose(self.goal_poses[i])

			d = {"initial_configuration":d_start_config, "goal":d_goal_pose}			

			if self.using_goal_states: 
				d_goal_config = self._format_config(self.goal_configs[i,:])
				d["goal_configuration"] = d_goal_config

			if self.using_crit_states and topk_crit > 0: 
				d_crit_configs = self._format_crit_configs(
					self.crit_configs[:topk_crit,:])
				d["critical_configurations"] = d_crit_configs

			# dump to yaml
			with open(f_exp, 'w+') as f_dst:
				yaml.dump(d, f_dst, default_flow_style=None)

if __name__ == "__main__": 
	# To Do: add arg parser

	# Setup raw experiment files
	exp_src_dir = "../critical_experiments/generated_raw_experiments"
	f_start_configs = join(exp_src_dir, "start_states.txt")
	f_goal_poses = join(exp_src_dir, "goal_poses.txt")
	f_goal_configs = join(exp_src_dir, "goal_states.txt")

	# Setup critical state files
	crit_roadmap_label = "grow60_s50_g2000" #test
	crit_roadmap_dir = join("../critical_roadmaps", crit_roadmap_label)
	# Note: f_crit_configs assumed to be sorted in decreasing criticality 
	f_crit_configs = join(crit_roadmap_dir, "crit_roadmap_vertices.csv") 

	e = WalkerExperimentEditor(f_start_configs, f_goal_poses, f_goal_configs, 
		f_crit_configs)	

	# gen exp files
	exp_label = "multi_room"
	exp_dst_dir = join("../critical_experiments/", exp_label)
	e.generate_formatted_experiments(exp_dst_dir, exp_label, topk_crit=20)

