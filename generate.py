#!/usr/bin/python
# -*- coding: UTF-8 -*-
import os

for i in range(1000):
	os.system('cd /home/will/WBK/SceneNet/dysonroboticslab-scenenetrgb-d-afb686496a4a/scene_generator/build;echo "yuanweihao" | sudo -S sh scene_generate.sh')
	val = os.popen('cd /home/will/WBK/SceneNet/dysonroboticslab-scenenetrgb-d-afb686496a4a/camera_trajectory_generator/build;echo "yuanweihao" | sudo -S sh camera_trajectory.sh').read()
	if ("Pose out of bounds" in val) or ("Target Pose out of bounds" in val) or ("Pose didn't move" in val) or ("Target didn't move" in val) or ("Did not see enough objects" in val):
		continue