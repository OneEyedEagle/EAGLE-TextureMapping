#include "settings.h"
#include "Eagle_Utils.h"
#include "getmeshcamera.h"
#include "getkeyframes.h"
#include "getalignresults.h"

int main()
{
    Settings settings = Settings();

    // 1 - estimate the geometry and the camera poses of each frame (D and RGB) using KF
    if ( !EAGLE::isFileExist(settings.cameraTxtFile) ) {
        getMeshCamera step1_mesh_camera(settings);
    }
    // output the ply(point cloud) from the pcd
    if ( !EAGLE::isFileExist(settings.plyWorldFile) ){
        //  when doing the PCL mesh output, the out-of-memory error occurs
        char buf2[24];
        sprintf(buf2, "-vs %f", settings.volumeSize);
        std::string str(buf2);
        system( ("pcl_kinfu_largeScale_mesh_output " + settings.pcdWorldFile + " " + str).c_str() );
        system( ("cp mesh_1.ply " + settings.plyWorldFile).c_str() );
        std::cout << "[ PLY Success ]" << std::endl;
    }

    // 2 - select keyframes
    if ( !EAGLE::isFileExist(settings.kfCameraTxtFile) ) {
        // output the keyframes ( with new indexes )
        getKeyframes step2_getKfs(settings);
    }

    // for each image, Ti = Si, Mi = Si, iteratively two steps of alignment and reconstruction
    // 3 - Alignment (Patch-Match)
    getAlignResults step3_align(settings);

    return 0;
}
