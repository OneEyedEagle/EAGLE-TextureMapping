# EAGLE-TextureMapping

This is an attempt for implementation of the paper "Patch-Based Optimization for Image-Based Texture Mapping".

## Before use

1. Make sure you have installed **Boost**, **Eigen3**, **VTK 6.3**, **PCL 1.9**, **OpenCV 4**, and **ImageMagick**.
   (I'm using Ubuntu 16.04, but the project works if you successfully install these things on Windows.)

2. Open the project with **Qt creator** (be sure to set the Make path as same as the project path), then you need to edit the INCLUDE paths in _eagle_textureMapping.pro_ file to fit in your envirenment.
   (If necessary, recompile _./lib/Eagle_Utils.cpp_ with _g++ -shared -o libEagle_Utils.so Eagle_Utils.cpp_ , if the OpenCV4 is in the system path.)

3. **[UPDATE in 2020.1.9]** I upgrade a mini-version of TextureMapping codes, which only include things relating to Patch-Based Optimization. The old version can still be found on branch _Full-Ver_.

4. **[UPDATE in 2020.9.7]** I added some settings in _getalignresults.cpp_ file's header, check their comments for more information. Also I editted the obj generation function for much better result, now you could edit the function _pointProjectionValidMesh_ to change the condition when generating the obj. It's recommanded to set a more rigid condition then that when iterating, to avoid reprojecting which is on the boundary.

## How to use

All parameters are in the _settings.h_ file, change them to your own data's info as the comment says.
   (The data in the project has been configed on _init_zhou()_ function.)

- If you have a PLY file:

  1. It's recommended to choose some keyframes manually for better results, also to delete the keyframe which shifts too much among others. (what the _kfIndexs_ variable does.)
     (That is to say, these frames' camera matrixs are inaccurate or with large errors. It's really difficult for my code to make these frames fit others. So ghosting areas remain at such situation.)

  2. Change _settings.h_, especially checking for _kfCameraTxtFile_ and _plyFile_ under the _keyFramesPath_.
  
  3. Run the project.

## About settings

- The _patchStep_ variable controls the patch numbers when voting. If it's set to 1, the _lamda_ variable needs to be large enough to make Targets not as same as Sources.
   (I explore _patchStep = patchWidth / 2_ makes _lamda = 0.2 ~ 0.4_ effectively work, but larger _lamda_ makes the result fall into local minima too early, while smaller _lamda_ makes no sense as Textures will do nothing to Targets.)

- The _scaleInitH_ variable sets the first scale resolution's height. I find if it's set to 64, it's quite difficult to store minor textures at finer scales in my datas. So I set it to _originImgH / 4_ which is better than too small.

- **[UPDATE in 2020.9.7]** I added some limits when remapping (when pixels on the image A will be projected to the world, then projected to the image B), which could be found in the function _pointProjectionValid_. 

  1. I make sure the depth is close to the origin on image B, to avoid occlusion. 
  
  2. As pixels on the boundary (which means its neighbors may be on background.) will make mistakes, I disgard all pixels that projected to the boundary of the object.

  3. While the angle between ray (which is a line from the pixel's world position to the mesh) and the mesh is calculated, I add a condition that the angle should be larger than 90°, then the mesh is more close to the parallel state with the image plane.
  
