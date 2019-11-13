# EAGLE-TextureMapping

This is a (maybe) wrong implementation of the paper "Patch-Based Optimization for Image-Based Texture Mapping".

## Before use

1. Make sure you have installed **Boost**, **Eigen3**, **vtk 6.3**, **PCL 1.9** (with **OpenNI** and **CUDA 10.1** if using PCL_KinectFusion to get an PLY file in the _getmeshcamera.cpp_, otherwise you can delete all things related to gpu in _eagle_textureMapping.pro_), **OpenCV**.
   (I'm using Ubuntu 16.04)
2. Open the project with **Qt creator** (be sure to set the Make path as same as the project path), then you need to edit the INCLUDE paths in _eagle_textureMapping.pro_ file to fit in your envirenment.
   (If necessary, recompile _./lib/Eagle_Utils.cpp_ and _./patchmatch/eagle_pm_minimal.cpp_, the surfix \_id of the patchmatch bin filename is the patch width.)

## How to use

All parameters are in the _settings.h_ file, change them to your own data's info as the comment says.
   (The data in the project has been configed on _init_zhou()_ function.)

- If you don't have a PLY file:

  1. You need to rename all original Kinect pictures with consistent indexs, and put them under a folder.
  2. Change _settings.h_ with all infos, including the folder path, RGBD name pattern, camera intrinsics...
  3. Run the project. If _out-of-memory_ error occurs (this happens if the GPU has limited free memory, restart the computer or close Qt creator and run _sudo sync_ may help release), find the world.pcd under the _keyFramesPath_ path, and do _pcl_kinfu_largeScale_mesh_output world.pcd -vs 4.0_ in the console yourself (4.0 is the _volume_size_ in _getmeshcamera.cpp_, change it if you editted).
  4. Edit the PLY with meshlab, like deleting useless faces, deleting unreferenced vertex and faces, doing simplification...
  5. Now you have the PLY file, then do as in "If you have a PLY file".

- If you have a PLY file:

  1. It's recommended to choose some keyframes manually for better results, also delete the keyframe which *shifts* too much among others. (what the _kfIndexs_ variable does.)
     (That is to say, these frames' camera matrixs are inaccurate or with large errors. It's really difficult for my code to make these frames fit others. So ghosting areas remain at such situation.)
  2. Change _settings.h_, especially for _kfCameraTxtFile_ and _plyFile_ under the _keyFramesPath_.
  2. Run the project.

## About settings

1. The _patchStep_ variable controls the patch numbers when voting. If it's set to 1, the _lamda_ variable needs to be large enough to make Targets not as same as Sources.
   (I explore _patchStep = patchWidth / 2_ makes _lamda = 0.2 ~ 0.4_ effectively work, but larger _lamda_ makes the result fall into local minima too early, while smaller _lamda_ makes no sense as Textures will do nothing to Targets.)
2. The _scaleInitH_ variable sets the first scale resolution's height. I find if it's set to 64, it's quite difficult to store minor textures at finer scales in my datas. So I set it to _originImgH / 4_ which is better than too small.
