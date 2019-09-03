# README #

### Related publications ###
Please cite this work if you make use of our system in any of your own endeavors:

* **[SceneNet RGB-D: Can 5M Synthetic Images Beat Generic ImageNet Pre-training on Indoor Segmentation?](http://www.imperial.ac.uk/media/imperial-college/research-centres-and-groups/dyson-robotics-lab/jmccormac_etal_iccv2017.pdf)**, *J. McCormac, A. Handa, S. Leutenegger, and A. J. Davison*, ICCV '17

# 1. Overview #
This is the SceneNet modified by *BKAUTO* to generate trinocular images. The build & compile steps are basically the same as in the [original SceneNet RGBD](https://bitbucket.org/dysonroboticslab/scenenetrgb-d/src/master/). Codes in the third part (render part) are modified to provide correct depth image and multiple-eye views. Models and their appearance probability can be adjust in '/scene_generator/textfiles/'.

To better understanding the original design of SceneNet RGBD, I strongly recommend you to read the paper below before any further modification. 

* **[SceneNet RGB-D: 5M Photorealistic Images of Synthetic Indoor Trajectories with Ground Truth](https://robotvault.bitbucket.io/SceneNetRGBD.pdf)**, *J. McCormac, A. Handa, S. Leutenegger, and A. J. Davison*, arXiv '16
