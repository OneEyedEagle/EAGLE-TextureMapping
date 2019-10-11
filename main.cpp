#include "settings.h"
#include "Eagle_Utils.h"
#include "getmeshcamera.h"
#include "getkeyframes.h"
#include "getalignresults.h"

int main()
{
    Settings settings = Settings();

    // 1 - estimate the geometry and the camera poses of each frame (D and RGB) using KF
    //getMeshCamera step1_mesh_camera(settings);

    // 2 - select keyframes
    //getKeyframes step2_getKfs(settings); // output the keyframes ( with new indexes )

    // for each image, Ti = Si, Mi = Si, iteratively two steps of alignment and reconstruction
    // 3 - Alignment (Patch-Match)
    getAlignResults step3_align(settings);

    return 0;
}
