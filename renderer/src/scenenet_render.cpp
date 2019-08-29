/*
 * This file is part of SceneNet RGB-D.
 *
 * Copyright (C) 2017 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is SemanticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/semantic-fusion/scenenet-rgbd-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "Renderer/OptixRenderer.h"
#include "Scene/SceneNet.h"
#include "Util/sutil/sutil.h"

#include <iostream>
#include <memory>
#include <string>

extern SceneNetCamera now_camera;

class BaseScene {
public:
  BaseScene() : m_render_type(RenderType::IMAGE) {}
  void   initScene(std::string save_base,std::string layout_file,std::string obj_base_folder,std::string layout_base_folder,int gpu);
  bool   trace(std::string save_name, int frame_num);
  optix::Buffer getOutputBuffer();
  optix::Buffer getRawOutputBuffer();

private:
  std::unique_ptr<OptixRenderer> m_renderer;
  std::unique_ptr<SceneNet>      m_iscene;
  RenderType      m_render_type;

  static unsigned int WIDTH;
  static unsigned int HEIGHT;
};

unsigned int BaseScene::WIDTH  = 1280u;
unsigned int BaseScene::HEIGHT = 1080u;

void BaseScene::initScene(std::string save_base,std::string layout_file,std::string obj_base_folder,std::string layout_base_folder, int gpu) {
  // A seed of 0 signifies a random seed - any other number means to use that as
  // the deterministic seed value
  int seed = 0;
  m_iscene.reset(new SceneNet(save_base,layout_file,obj_base_folder,layout_base_folder,seed));
  //Check the scene isn't too bright or dark (i.e. bleached or totally black,with quick render
  m_renderer.reset(new OptixRenderer(WIDTH,HEIGHT,false));
  m_renderer->initialize(gpu,1,1);
  m_renderer->initScene(*m_iscene);
  m_renderer->calculatePhotonMap();
  const SceneNetCamera& default_camera = m_iscene->getCamera();
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose = m_iscene->getPose();
  if (start_pose.first[0] == 0.0 && start_pose.first[1] == 0.0 && start_pose.first[2] == 0.0 &&
      start_pose.second[0] == 0.0 && start_pose.second[1] == 0.0 && start_pose.second[2] == 0.0) {
    exit(1);
  }
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose = m_iscene->getPose();
  if (end_pose.first[0] == 0.0 && end_pose.first[1] == 0.0 && end_pose.first[2] == 0.0 &&
      end_pose.second[0] == 0.0 && end_pose.second[1] == 0.0 && end_pose.second[2] == 0.0) {
    exit(1);
  }
  m_renderer->render(default_camera,start_pose,end_pose,RenderType::IMAGE);
  double average_intensity = 0.0;
  optix::uchar4* buffer_Host = (optix::uchar4*) m_renderer->getOutputBuffer(RenderType::IMAGE)->map();
  for(int i = 0; i < WIDTH * HEIGHT; ++i) {
    optix::uchar4 rgb_val = buffer_Host[i];
    int max_val = 0;
    max_val = std::max(max_val,static_cast<int>(rgb_val.x));
    max_val = std::max(max_val,static_cast<int>(rgb_val.y));
    max_val = std::max(max_val,static_cast<int>(rgb_val.z));
    average_intensity += max_val;
  }
  average_intensity /= WIDTH * HEIGHT;
  m_renderer->getOutputBuffer(RenderType::IMAGE)->unmap();
  std::cout<<"Average intensity:"<<average_intensity<<std::endl;
  if (average_intensity < 60 || average_intensity > 180) {
    std::cout<<"Average intensity too extreme:"<<average_intensity<<std::endl;
    exit(1);
  }
  // Use this for normal quality renders
  // Samples are squared so 4 means 16 total
  m_renderer->setNumIterations(4);
  m_renderer->setNumPhotonMaps(4);
  m_renderer->calculatePhotonMap();
}

optix::Buffer BaseScene::getOutputBuffer() {
  return m_renderer->getOutputBuffer(m_render_type);
}

optix::Buffer BaseScene::getRawOutputBuffer() {
  return m_renderer->getRawOutputBuffer(m_render_type);
}

bool BaseScene::trace(std::string save_name_base, int frame_num) {
  const SceneNetCamera& default_camera = m_iscene->getCamera();
  bool reset = false;
  if (frame_num == 0) {
    reset=true;
  }

  // The two poses are for motion blur
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose = m_iscene->getPose(reset,false);
  if (start_pose.first[0] == 0.0 && start_pose.first[1] == 0.0 && start_pose.first[2] == 0.0 &&
      start_pose.second[0] == 0.0 && start_pose.second[1] == 0.0 && start_pose.second[2] == 0.0) {
    return false;
  }
  const std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose = m_iscene->getPose(false,reset);
  if (end_pose.first[0] == 0.0 && end_pose.first[1] == 0.0 && end_pose.first[2] == 0.0 &&
      end_pose.second[0] == 0.0 && end_pose.second[1] == 0.0 && end_pose.second[2] == 0.0) {
    return false;
  }

  if (frame_num % 100 != 0) {
    return true;
  }

  std::cout<<"About to render M_RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose,end_pose,m_render_type);
  std::string save_name = save_name_base+std::to_string(frame_num)+"_0_rgb.png";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());
  // Also save ground truth
  // Only need to call render once for all ground truth
  /*
  m_render_type = RenderType::INSTANCE;
  m_renderer->render(default_camera,start_pose,end_pose,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_instance.png";
  std::cout<<"Instance saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());
  */
  m_render_type = RenderType::DEPTH;
  m_renderer->render(default_camera,start_pose,end_pose,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_0_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose_left = start_pose;
  std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose_left = end_pose;
  start_pose_left.first[0] -= 0.2*now_camera.camera_u_1.x;
  start_pose_left.first[1] -= 0.2*now_camera.camera_u_1.y;
  start_pose_left.first[2] -= 0.2*now_camera.camera_u_1.z;
  start_pose_left.second[0] -= 0.2*now_camera.camera_u_1.x;
  start_pose_left.second[1] -= 0.2*now_camera.camera_u_1.y;
  start_pose_left.second[2] -= 0.2*now_camera.camera_u_1.z;
  end_pose_left.first[0] -= 0.2*now_camera.camera_u_1.x;
  end_pose_left.first[1] -= 0.2*now_camera.camera_u_1.y;
  end_pose_left.first[2] -= 0.2*now_camera.camera_u_1.z;
  end_pose_left.second[0] -= 0.2*now_camera.camera_u_1.x;
  end_pose_left.second[1] -= 0.2*now_camera.camera_u_1.y;
  end_pose_left.second[2] -= 0.2*now_camera.camera_u_1.z;


  std::cout<<"About to render L_RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose_left,end_pose_left,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_1_rgb.png";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());
  // Also save ground truth

  m_render_type = RenderType::DEPTH;
  m_renderer->render(default_camera,start_pose_left,end_pose_left,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_1_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose_right = start_pose;
  std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose_right = end_pose;
  start_pose_right.first[0] += 0.2*now_camera.camera_u_1.x;
  start_pose_right.first[1] += 0.2*now_camera.camera_u_1.y;
  start_pose_right.first[2] += 0.2*now_camera.camera_u_1.z;
  start_pose_right.second[0] += 0.2*now_camera.camera_u_1.x;
  start_pose_right.second[1] += 0.2*now_camera.camera_u_1.y;
  start_pose_right.second[2] += 0.2*now_camera.camera_u_1.z;
  end_pose_right.first[0] += 0.2*now_camera.camera_u_1.x;
  end_pose_right.first[1] += 0.2*now_camera.camera_u_1.y;
  end_pose_right.first[2] += 0.2*now_camera.camera_u_1.z;
  end_pose_right.second[0] += 0.2*now_camera.camera_u_1.x;
  end_pose_right.second[1] += 0.2*now_camera.camera_u_1.y;
  end_pose_right.second[2] += 0.2*now_camera.camera_u_1.z;


  std::cout<<"About to render R_RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose_right,end_pose_right,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_2_rgb.png";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());

  m_render_type = RenderType::DEPTH;
  m_renderer->render(default_camera,start_pose_right,end_pose_right,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_2_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose_bottom = start_pose;
  std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose_bottom = end_pose;
  start_pose_bottom.first[0] -= 0.2*now_camera.camera_v_1.x;
  start_pose_bottom.first[1] -= 0.2*now_camera.camera_v_1.y;
  start_pose_bottom.first[2] -= 0.2*now_camera.camera_v_1.z;
  start_pose_bottom.second[0] -= 0.2*now_camera.camera_v_1.x;
  start_pose_bottom.second[1] -= 0.2*now_camera.camera_v_1.y;
  start_pose_bottom.second[2] -= 0.2*now_camera.camera_v_1.z;
  end_pose_bottom.first[0] -= 0.2*now_camera.camera_v_1.x;
  end_pose_bottom.first[1] -= 0.2*now_camera.camera_v_1.y;
  end_pose_bottom.first[2] -= 0.2*now_camera.camera_v_1.z;
  end_pose_bottom.second[0] -= 0.2*now_camera.camera_v_1.x;
  end_pose_bottom.second[1] -= 0.2*now_camera.camera_v_1.y;
  end_pose_bottom.second[2] -= 0.2*now_camera.camera_v_1.z;

  std::cout<<"About to render B_RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose_bottom,end_pose_bottom,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_3_rgb.png";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());

  m_render_type = RenderType::DEPTH;
  m_renderer->render(default_camera,start_pose_bottom,end_pose_bottom,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_3_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  std::pair<TooN::Vector<3>,TooN::Vector<3> > start_pose_top = start_pose;
  std::pair<TooN::Vector<3>,TooN::Vector<3> > end_pose_top = end_pose;
  start_pose_top.first[0] += 0.2*now_camera.camera_v_1.x;
  start_pose_top.first[1] += 0.2*now_camera.camera_v_1.y;
  start_pose_top.first[2] += 0.2*now_camera.camera_v_1.z;
  start_pose_top.second[0] += 0.2*now_camera.camera_v_1.x;
  start_pose_top.second[1] += 0.2*now_camera.camera_v_1.y;
  start_pose_top.second[2] += 0.2*now_camera.camera_v_1.z;
  end_pose_top.first[0] += 0.2*now_camera.camera_v_1.x;
  end_pose_top.first[1] += 0.2*now_camera.camera_v_1.y;
  end_pose_top.first[2] += 0.2*now_camera.camera_v_1.z;
  end_pose_top.second[0] += 0.2*now_camera.camera_v_1.x;
  end_pose_top.second[1] += 0.2*now_camera.camera_v_1.y;
  end_pose_top.second[2] += 0.2*now_camera.camera_v_1.z;


  std::cout<<"About to render T_RGB"<<std::endl;
  m_render_type = RenderType::IMAGE;
  m_renderer->render(default_camera,start_pose_top,end_pose_top,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_4_rgb.png";
  std::cout<<"Saving here:"<<save_name<<std::endl;

  //std::cout<<"Saving file:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getOutputBuffer()->get());
  // Also save ground truth

  m_render_type = RenderType::DEPTH;
  m_renderer->render(default_camera,start_pose_top,end_pose_top,m_render_type);
  save_name = save_name_base+std::to_string(frame_num)+"_4_depth.png";
  std::cout<<"Depth saving here:"<<save_name<<std::endl;
  sutil::displayBufferPPM(save_name.c_str(),getRawOutputBuffer()->get());

  return true;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cout<<"Too few arguments"<<std::endl;
        std::cout<<"./render /path/to/ShapeNet/ /path/to/SceneNetLayouts/ /path/to/scene_and_trajectory_description.txt"<<std::endl;
        std::cout<<"Note that the folders should be followed by trailing / in the command"<<std::endl;
        exit(1);
    }
    std::string shapenets_dir = std::string(argv[1]);
    std::cout<<"ShapeNet directory:"<<shapenets_dir<<std::endl;
    std::string layouts_dir = std::string(argv[2]);
    std::cout<<"Layouts directory:"<<layouts_dir<<std::endl;
    std::string scene_description_file = std::string(argv[3]);
    std::cout<<"Input Scene Description:"<<scene_description_file<<std::endl;
    std::string output_dir = std::string(argv[4]);
  
    BaseScene scene;
    scene.initScene(output_dir,scene_description_file,shapenets_dir,layouts_dir,0);
    const int number_trajectory_steps = 10000;
    for (int i = 0; i < number_trajectory_steps; ++i) {
        if (!scene.trace(output_dir,i)) {
          std::cout<<"Finished render"<<std::endl;
          break;
        }
    }
    return 0;
}
