[depth] [image] are the raw depth and color images from a PrimeSense camera. The naming of the images is [index-timestamp.png] or [index-timestamp.jpg].

Camera parameters for depth images:
width = 640
height = 480
fx = 525.0
fy = 525.0
cx = 319.5
cy = 239.5

Camera parameters for color images:
width = 1280
height = 1024
fx = 1050.0
fy = 1050.0
cx = 639.5
cy = 511.5

All color images are converted to 640x480 images, and stored in [vga] folder with the same naming.
They are paired with depth images.
The pairing is stored in [depth_vga.match]

[key.txt] stores the indices of key frames from the color stream.
They are indices in [depth_vga.match] (index starts from 1).

[fountain_from_kinectfusion.ply] is the mesh extracted from KinectFusion.

[fountain_key.log] stores the camera poses of all keyframes. The result comes from optimization from our SIGGRAPH 2014 paper.

[fountain.ply] is the final mesh with color mapping.
