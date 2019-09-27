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
    double scaleFactor, scaleInitW, scaleInitH, alpha_u, alpha_v, lambda;
    float volumeSize;

    std::string allFramesPath, cameraTxtFile;
    std::string keyFramesPath, patchmatchBinFile, originResolution;
    std::string kfCameraTxtFile, kfBestTxtFile, kfPoseTxtFile, kfNumTxtFile;
    std::string rgbNamePattern, dNamePattern, kfRGBNamePattern, kfDNamePattern, rgbNameExt;
    std::string kfRGBMatch, kfDMatch;
    std::string pcdWorldFile, plyWorldFile, plySColorFilename, plyTColorFilename, plyMColorFilename;

    float cameraDepthFactor, cameraFx, cameraFy, cameraCx, cameraCy;
    cv::Mat1f cameraK;
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
        alpha_u = 0.5;//1;
        //   weight the similarity from Ti to Si
        alpha_v = 0.5;//2;
        //   weight how much Mi affects Ti
        lambda = 0.3;//0.1;

        // -----------------
        //  custom
        // -----------------
        init_zhou();

        // -----------------
        //  init
        // -----------------
        // path of PatchMatch's bin
        patchmatchBinFile = "/home/wsy/TextureRecover/patchmatch-2.1/eagle_pm_minimal";

        // scale
        scaleTimes = 10;
        scaleFactor = pow( originImgH / 64.0, 1.0 / 9 );
        scaleInitW = originImgW * 1.0 / originImgH * 64;
        scaleInitH = 64;
//        scaleTimes = 4;
//        scaleFactor = 2;
//        scaleInitW = 80;
//        scaleInitH = 60;
//        scaleTimes = 5;
//        scaleFactor = 2;
//        scaleInitW = 80;
//        scaleInitH = 64;

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

    void init_zhou(){
        originImgW = 1280;
        originImgH = 1024;

        cameraFx = 1050.0f;
        cameraFy = 1050.0f;
        cameraCx = 639.5f;
        cameraCy = 511.5f;

        keyFramesPath = "/home/wsy/TextureRecover/Results/zhou_full2";
        rgbNameExt = "jpg";
        rgbNamePattern = "%05d." + rgbNameExt;
        dNamePattern = "%05d.png";

        kfRGBNamePattern = "%05d." + rgbNameExt;
        kfRGBMatch = "*." + rgbNameExt;
        kfDNamePattern = "%05d.png";
        kfDMatch = "*.png";

        kfIndexs = {0,6,13,15,21};//{0,4,6,13,17,19,21};
    }
};

#endif // SETTINGS_H
