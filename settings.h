#ifndef SETTINGS_H
#define SETTINGS_H

#include "Eagle_Utils.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Settings
{
public:
    int originImgW, originImgH, imgW, imgH, patchWidth, patchSize;
    int frameStart, frameEnd, scaleTimes;
    double scaleFactor, scaleInitW, scaleInitH, alpha, lambda;
    float volumeSize;

    std::string allFramesPath, cameraTxtFile;
    std::string keyFramesPath, patchmatchBinFile, originResolution;
    std::string kfCameraTxtFile, kfBestTxtFile, kfPoseTxtFile, kfNumTxtFile;
    std::string rgbNamePattern, dNamePattern, kfRGBNamePattern, kfDNamePattern, rgbNameExt;
    std::string kfRGBMatch, kfDMatch;
    std::string pcdWorldFile, plyWorldFile, plySColorFilename, plyTColorFilename, plyMColorFilename;

    float cameraDepthFactor, cameraFx, cameraFy, cameraCx, cameraCy;
    cv::Mat cameraK;
    std::vector<size_t> kfIndexs;

    Settings()
    {
        // -----------------
        //  init lab
        // -----------------
        // origin source img's resolution
        originImgW = 640;
        originImgH = 480;
        // camera's data
        cameraDepthFactor = 1000.0f;
        cameraFx = 527.3f;
        cameraFy = 527.08f;
        cameraCx = 323.73f;
        cameraCy = 277.25f;

        // all frames' path
        allFramesPath = "/home/wsy/TextureRecover/Datas/bloster2";
        // all frames' name pattern
        rgbNameExt = "jpg";
        rgbNamePattern = "%05d." + rgbNameExt;
        dNamePattern = "%05d.png";
        // frames' start and end
        frameStart = 0;
        frameEnd = 394;

        // keyframes' path (with camera positions' txt file)
        keyFramesPath = "/home/wsy/TextureRecover/Results/bloster2_1";
        // keyframes' name pattern
        kfRGBNamePattern = "%03d_rgb." + rgbNameExt;
        kfRGBMatch = "*_rgb*" + rgbNameExt;
        kfDNamePattern = "%03d_d.png";
        kfDMatch = "*_d*";
        // valid keyframes
        kfIndexs = { 1,2,3,4,6,7,9,10,12,14,15 };

        // volume size in PCD file
        volumeSize = 4.0f;

        // the size of patch (patchWidth * patchWidth)
        patchWidth = 5;//7;

        // energy function's parameters
        //   weight the similarity from Ti to Si
        alpha = 1;
        //   weight how much Mi affects Ti
        lambda = 2;//0.1;

        // -----------------
        //  custom
        // -----------------
        init_zhou();

        // -----------------
        //  init
        // -----------------
        // path of PatchMatch's bin
        patchmatchBinFile = "/home/wsy/EAGLE/EAGLE-TextureMapping/patchmatch-2.1/eagle_pm_minimal";

        // scale
        int x = originImgW;
        if( x > originImgH )
            x = originImgH;
        scaleTimes = 4;//10;
        scaleFactor = 2;//pow( x / 64.0, 1.0 / 9 );
        scaleInitW = 80;//settings.originImgW * 1.0 / settings.originImgH * 64;
        scaleInitH = 60;//64;

        // make the dir
        EAGLE::checkPath(keyFramesPath);
        // all frames' camera positions txt-file's full path
        cameraTxtFile = keyFramesPath + "/traj.txt";
        // the file storing best frames' index
        kfBestTxtFile = keyFramesPath + "/kfBest.txt";
        // the file storing best frames' index from Camera Pose
        kfPoseTxtFile = keyFramesPath + "/kfPose.txt";
        // the file storing best frames' index from Number Counting method
        kfNumTxtFile = keyFramesPath + "/kfNum.txt";
        // key frames' camera positions txt-file's full path
        kfCameraTxtFile = keyFramesPath + "/kfTraj.txt";

        // world.pcd file's full path
        pcdWorldFile = keyFramesPath + "/world.pcd";
        // world.ply file's full path
        plyWorldFile = keyFramesPath + "/world.ply";
        // wolrd_color.ply file's name (in keyFramesPath path)
        plySColorFilename = "world_source.ply";
        plyTColorFilename = "world_target.ply";
        plyMColorFilename = "world_texture.ply";

        // camera's intrinsic matrix
        //   fx  0 cx
        // [  0 fy cy ]
        //    0  0  1
        cameraK = (cv::Mat_<float>(3,3) << cameraFx, 0, cameraCx, 0, cameraFy, cameraCy, 0, 0, 1);
        // patch's size
        patchSize = patchWidth * patchWidth;

        // img's resolution during the current scale
        imgW = originImgW;
        imgH = originImgH;
    }

    void init_meeting_small_1(){
        cameraFx = 570.3f;
        cameraFy = 570.3f;
        cameraCx = 320.0f;
        cameraCy = 240.0f;

        allFramesPath = "/home/wsy/TextureRecover/rgbd-scenes/meeting_small/meeting_small_1";
        keyFramesPath = "/home/wsy/TextureRecover/Results/meeting_small_1";
        rgbNameExt = "png";
        rgbNamePattern = "meeting_small_1_%d." + rgbNameExt;
        dNamePattern = "meeting_small_1_%d_depth.png";
        frameStart = 5;
        frameEnd = 180;

        kfRGBNamePattern = "%03d_rgb." + rgbNameExt;
        kfRGBMatch = "*_rgb*" + rgbNameExt;
    }

    void init_table_small_1(){
        cameraFx = 570.3f;
        cameraFy = 570.3f;
        cameraCx = 320.0f;
        cameraCy = 240.0f;

        allFramesPath = "/home/wsy/TextureRecover/rgbd-scenes/table_small/table_small_1";
        keyFramesPath = "/home/wsy/TextureRecover/Results/table_small_1";
        rgbNameExt = "png";
        rgbNamePattern = "table_small_1_%d." + rgbNameExt;
        dNamePattern = "table_small_1_%d_depth.png";
        frameStart = 15;
        frameEnd = 145;

        kfRGBNamePattern = "%03d_rgb." + rgbNameExt;
        kfRGBMatch = "*_rgb*" + rgbNameExt;
    }

    void init_toy(){
        originImgW = 480;
        originImgH = 640;

        cameraFx = 494.02698f;
        cameraFy = 494.78205f;
        cameraCx = 239.49163f;
        cameraCy = 317.96980f;

        keyFramesPath = "/home/wsy/TextureRecover/Results/toy";
        rgbNameExt = "jpg";
        rgbNamePattern = "color_%02d." + rgbNameExt;
        dNamePattern = "depth_%02d.png";

        kfRGBNamePattern = "color_%02d." + rgbNameExt;
        kfRGBMatch = "color_*" + rgbNameExt;
        kfDNamePattern = "depth_%02d.png";
        kfDMatch = "depth_*.png";
    }

    void init_zhou(){
        originImgW = 640;
        originImgH = 480;

        cameraFx = 525.0f;
        cameraFy = 525.0f;
        cameraCx = 319.5f;
        cameraCy = 239.5f;

        keyFramesPath = "/home/wsy/TextureRecover/Results/zhou";
        rgbNameExt = "jpg";
        rgbNamePattern = "%05d." + rgbNameExt;
        dNamePattern = "%05d.png";

        kfRGBNamePattern = "%05d." + rgbNameExt;
        kfRGBMatch = "*." + rgbNameExt;
        kfDNamePattern = "%05d.png";
        kfDMatch = "*.png";

        kfIndexs = {0,1,3,4,6,7,8,11,12,15,19,22};
    }
};

#endif // SETTINGS_H
