# EAGLE-TextureMapping

This is an attempt for implementation of the paper "Patch-Based Optimization for Image-Based Texture Mapping".

## Before use

1. Make sure you have installed **Boost**, **Eigen3**, **VTK 6.3**, **PCL 1.9** (with **OpenNI** and **CUDA 10.1** if using PCL::KinectFusion to get a PLY file from RGBDs in the _getmeshcamera.cpp_, otherwise you can delete all things related to gpu in _eagle_textureMapping.pro_), **OpenCV**, and **ImageMagick**.
   (I'm using Ubuntu 16.04, but the project works if you successfully install these things on Windows.)

2. Open the project with **Qt creator** (be sure to set the Make path as same as the project path), then you need to edit the INCLUDE paths in _eagle_textureMapping.pro_ file to fit in your envirenment.
   (If necessary, recompile _./lib/Eagle_Utils.cpp_ .)

3. **[UPDATE in 2020.1.9]** I upgrade a mini-version of TextureMapping codes, which only include things relating to Patch-Based Optimization. The old version can still be found on branch _Full-Ver_.

4. **[UPDATE in 2020.9.7]** I added some settings in _getalignresults.cpp_ file's header, check their comments for more information. Also I editted the obj generation function for much better result, now you could edit the function _pointProjectionValidMesh_ to change the condition when generating the obj. It's recommanded to set a more rigid condition then that when iterating, to avoid reprojecting which is on the boundary.

## How to use

All parameters are in the _settings.h_ file, change them to your own data's info as the comment says.
   (The data in the project has been configed on _init_zhou()_ function.)

- If you don't have a PLY file:
  （You need to run codes under _Full-Ver_ branch to get a PLY file.）

  1. You need to rename all original Kinect pictures with consistent indexs, and put them under a folder.

  2. Change _settings.h_ with all infos, including the folder path, RGBD name patterns, camera intrinsics...

  3. Run the project. If _out-of-memory_ error occurs (This happens when the GPU has limited free memory. To restart the computer or close Qt creator and run _sudo sync_ may help.), find the _world.pcd_ under the _keyFramesPath_ path, and do _pcl_kinfu_largeScale_mesh_output world.pcd -vs 4.0_ in the console yourself (4.0 is the _volume_size_ in _getmeshcamera.cpp_, change it if you editted). If memory is enough, the PCL will generate some PLYs under the folder named mesh_1.ply, mesh_2.ply... Generally, _mesh_1.ply_ is what we want.

  4. Edit the _mesh_1.ply_ with Meshlab, do some stuffs like deleting useless faces, deleting unreferenced vertex and faces, doing simplification...

  5. Now you have the PLY file, then do as in "If you have a PLY file".

- If you have a PLY file:

  1. It's recommended to choose some keyframes manually for better results, also to delete the keyframe which shifts too much among others. (what the _kfIndexs_ variable does.)
     (That is to say, these frames' camera matrixs are inaccurate or with large errors. It's really difficult for my code to make these frames fit others. So ghosting areas remain at such situation.)

  2. Change _settings.h_, especially checking for _kfCameraTxtFile_ and _plyFile_ under the _keyFramesPath_.
  
  3. Run the project.

## About settings

1. The _patchStep_ variable controls the patch numbers when voting. If it's set to 1, the _lamda_ variable needs to be large enough to make Targets not as same as Sources.
   (I explore _patchStep = patchWidth / 2_ makes _lamda = 0.2 ~ 0.4_ effectively work, but larger _lamda_ makes the result fall into local minima too early, while smaller _lamda_ makes no sense as Textures will do nothing to Targets.)

2. The _scaleInitH_ variable sets the first scale resolution's height. I find if it's set to 64, it's quite difficult to store minor textures at finer scales in my datas. So I set it to _originImgH / 4_ which is better than too small.
