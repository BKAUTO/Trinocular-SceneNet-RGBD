#!/usr/bin/python
# -*- coding: UTF-8 -*-
import os

for i in range(320, 1000):
	val = os.popen('cd /home/will/WBK/SceneNet/dysonroboticslab-scenenetrgb-d-afb686496a4a/renderer/build;mkdir /home/will/WBK/render_out_'+str(i)+'/;\
	echo "yuanweihao" | sudo -S ./scenenet_render /home/will/Backup/ShapeNetCore.v2/ /home/will/WBK/SceneNet/SceneNetRGBD_Layouts/ /home/will/WBK/SceneNet/dysonroboticslab-scenenetrgb-d-afb686496a4a/camera_trajectory_generator/build/scene_and_trajectory_description.txt /home/will/WBK/render_out_'+str(i)+'/').read()
	if ("Average intensity too extreme:" in val or "CVD::Exceptions" in val):
		os.system('rm -rf /home/will/WBK/render_out_'+str(i)+'/')
		continue
