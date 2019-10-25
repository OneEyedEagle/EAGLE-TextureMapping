#ifndef SETTINGS_H
#define SETTINGS_H

#include "Eagle_Utils.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Settings
{
public:
    int originImgW, originImgH, originDepthW, originDepthH, imgW, imgH, patchWidth, patchSize;
    int frameStart, frameEnd, scaleTimes;
    double scaleFactor, scaleInitW, scaleInitH, alpha_u, alpha_v, lamda;
    float volumeSize;

    std::string allFramesPath, cameraTxtFile, camTrajNamePattern;
    std::string keyFramesPath, patchmatchBinFile, originResolution;
    std::string kfCameraTxtFile, kfBestTxtFile, kfPoseTxtFile, kfNumTxtFile;
    std::string rgbNamePattern, dNamePattern, kfRGBNamePattern, kfDNamePattern, rgbNameExt;
    std::string kfRGBMatch, kfDMatch;
    std::string pcdWorldFile, plyFile;

    float cameraDFx, cameraDFy, cameraDCx, cameraDCy, cameraFx, cameraFy, cameraCx, cameraCy;
    cv::Mat1f cameraK, cameraDK;
    std::vector<size_t> kfIndexs;

    Settings()
    {
        // -----------------
        //  init lab
        // -----------------
        // origin source img's resolution
        originImgW = 1920;
        originImgH = 1080;
        // camera's data
        cameraFx = 1081.37f;//527.3f;
        cameraFy = 1081.37f;//527.08f;
        cameraCx = 959.5f;//323.73f;
        cameraCy = 539.5f;//277.25f;
        // origin depth img's resolution
        originDepthW = originImgW;
        originDepthH = originImgH;
        // depth camera's data
        cameraDFx = cameraFx;
        cameraDFy = cameraFy;
        cameraDCx = cameraCx;
        cameraDCy = cameraCy;

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
        keyFramesPath = "/home/wsy/TextureRecover/Results/bloster2_2";
        // keyframes' name pattern
        kfRGBNamePattern = "color_%02d." + rgbNameExt;
        kfRGBMatch = "color_*." + rgbNameExt;
        kfDNamePattern = "%03d_d.png";
        kfDMatch = "*_d*";
        // valid keyframes
        kfIndexs = { 1,3,4,5,7,9,12 }; // ,5,6,7,9,10,12,14,15

        // (all files are under the keyFramesPath folder)
        // all frames' camera positions txt-file's full path
        cameraTxtFile = "traj.txt";
        // camera files' name pattern (if cameraTxtFile doesn't exist, then assume every image has its own camera file)
        camTrajNamePattern = "color_%02d.cam";
        // the file storing best frames' index
        kfBestTxtFile = "kfBest.txt";
        // the file storing best frames' index from Camera Pose
        kfPoseTxtFile =  "kfPose.txt";
        // the file storing best frames' index from Number Counting method
        kfNumTxtFile = "kfNum.txt";
        // key frames' camera positions txt-file's full path
        kfCameraTxtFile = "kfTraj.txt";
        // ply files
        pcdWorldFile = "world.pcd";
        plyFile = "world.ply";

        // volume size in PCD file
        volumeSize = 4.0f;

        // the size of patch (patchWidth * patchWidth)
        patchWidth = 5;

        // energy function's parameters
        alpha_u = 1;//1;
        //   weight the similarity from Ti to Si
        alpha_v = 2;//2;
        //   weight how much Mi affects Ti
        lamda = 0.1;//0.1;

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
        scaleInitH = 80;
        scaleInitW = originImgW * 1.0 / originImgH * scaleInitH;
        scaleFactor = pow( originImgH * 1.0 / scaleInitH, 1.0 / (scaleTimes-1) );

        // make the dir
        EAGLE::checkPath(keyFramesPath);

        // camera's intrinsic matrix
        //   fx  0 cx
        // [  0 fy cy ]
        //    0  0  1
        cameraK = (cv::Mat_<float>(3,3) << cameraFx, 0, cameraCx, 0, cameraFy, cameraCy, 0, 0, 1);
        cameraDK = (cv::Mat_<float>(3,3) << cameraDFx, 0, cameraDCx, 0, cameraDFy, cameraDCy, 0, 0, 1);
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

        keyFramesPath = "/home/wsy/EAGLE/EAGLE-TextureMapping/datas";
        rgbNameExt = "jpg";
        rgbNamePattern = "%05d." + rgbNameExt;
        dNamePattern = "%05d.png";

        kfRGBNamePattern = "%05d." + rgbNameExt;
        kfRGBMatch = "*." + rgbNameExt;
        kfDNamePattern = "%05d.png";
        kfDMatch = "*.png";

        kfIndexs = {0,4,6,13};//{0,4,6,13,17,19,21};
    }
};

#endif // SETTINGS_H
