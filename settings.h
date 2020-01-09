#ifndef SETTINGS_H
#define SETTINGS_H

#include "Eagle_Utils.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class Settings
{
public:
    int originImgW, originImgH, originDepthW, originDepthH, imgW, imgH, scaleInitW, scaleInitH;
    int patchWidth, patchStep, patchSize, frameStart, frameEnd;
    double scaleFactor, alpha_u, alpha_v, lamda, patchRandomSearchTimes;
    size_t scaleTimes;
    std::vector<size_t> kfIndexs, scaleIters;

    std::string resultsPathSurfix;
    std::string allFramesPath, cameraTxtFile, camTrajNamePattern;
    std::string keyFramesPath, kfCameraTxtFile, patchmatchBinFile, originResolution, plyFile;
    std::string rgbNamePattern, dNamePattern, kfRGBNamePattern, kfDNamePattern, rgbNameExt, kfRGBMatch;
    bool camTrajFromWorldToCam;
    float cameraDFx, cameraDFy, cameraDCx, cameraDCy, cameraFx, cameraFy, cameraCx, cameraCy;
    cv::Mat1f cameraK, cameraDK;
    char depthType;

    Settings()
    {
        // set the intrinsics of the RGB camera and the depth camera
        {
            // origin source img's resolution
            originImgW = 1920;
            originImgH = 1080;
            // camera's data
            cameraFx = 1081.37f;
            cameraFy = 1081.37f;
            cameraCx = 959.5f;
            cameraCy = 539.5f;
            // origin depth img's resolution
            originDepthW = originImgW;
            originDepthH = originImgH;
            // depth camera's data
            cameraDFx = cameraFx;
            cameraDFy = cameraFy;
            cameraDCx = cameraCx;
            cameraDCy = cameraCy;
        }

        // set the file extension of RGB image (to store the rgb file in Results folder)
        rgbNameExt = "jpg";
        // set the depth data type
        //  f - float  i - int  s - ushort  b - uchar
        depthType = 'f';

        // if you already have a ply file, than no need to set these variables to use PCL for getting a ply
        //   (more details can be found in main.cpp)
        {
            // all frames' path
            allFramesPath = "/home/wsy/TextureRecover/Datas/bloster2_1";
            // all frames' name pattern
            rgbNamePattern = "%05d." + rgbNameExt;
            dNamePattern = "%05d.png";
            // frames' start and end
            frameStart = 0;
            frameEnd = 394;
            // all frames' camera positions txt file after using PCL KF to get the ply (under the keyFramesPath folder)
            //  the format :
            //    id id id
            //    R(0,0) R(0,1) R(0,2) T(0)
            //    R(1,0) R(1,1) R(1,2) T(1)
            //    R(2,0) R(2,1) R(2,2) T(2)
            //    0 0 0 1
            cameraTxtFile = "traj.txt";
        }

        // keyframes' path (where the rgbd images are)
        keyFramesPath = "/media/eagle/4670D70170D6F6A1/Datas/LAB/bloster/raw";
        // keyframes' name pattern
        kfRGBNamePattern = "color_%02d." + rgbNameExt;
        kfRGBMatch = "color_*." + rgbNameExt;
        // [optional] keyframes' depth name pattern, using depth images to do a check when remapping
        kfDNamePattern = "depth_%02d.png";
        // valid keyframes ( all are valid if empty)
        kfIndexs = {0,1,3,4,5,11,12,13};

        // necessary files under the keyFramesPath folder
        {
            // key frames' camera positions txt file after running getKeyFrames.cpp
            //  the format is same as the cameraTxtFile
            kfCameraTxtFile = "kfTraj.txt";
            // camera files' name pattern (if kfCameraTxtFile doesn't exist, then assume every image has its own camera file)
            //  in each file, all data is in one line with format "T(0) T(1) T(2) R(0,0) R(0,1) R(0,2) R(1,0) R(1,1) R(1,2) R(2,0) R(2,1) R(2,2)"
            camTrajNamePattern = "color_%02d.cam";
            // the ply file
            plyFile = "mesh_1.ply";
        }

        // if the camera matrix is a projection from the world coord to camera coord, set this flag to true,
        //  otherwise, the data is from camera coord to world coord, and inv() will be called.
        //  if the cameraTxtFile and the plyFile are from PCL KF, then this should be false.
        camTrajFromWorldToCam = true;

        // surfix of the resultsPath to distinguish with different results under different params
        resultsPathSurfix = "";

        // scale
        scaleTimes = 10;
        scaleIters = {50, 45, 40, 35, 30, 25, 20, 15, 10, 5};
        scaleInitH = originImgH / 4;

        // the width and height of a patch
        patchWidth = 7;
        // the step of patchs when voting
        patchStep = 1;
        // the times of range when random searching in patchmatch
        //  searching window's width = patchRandomSearchTimes * sqrt(imgW * imgH)
        patchRandomSearchTimes = 0.01;

        // weight the similarity from Si to Ti
        alpha_u = 1.0;
        // weight the similarity from Ti to Si
        alpha_v = 2.0;
        // weight the consistency that how much M affects Ti
        lamda = 5.0;

        // -----------------
        //  custom
        // -----------------
        init_zhou_small();

        // -----------------
        //  init
        // -----------------
        // scale
        scaleInitW = static_cast<int>( std::round(originImgW * scaleInitH * 1.0 / originImgH) );
        if ( scaleTimes > 1 )
            scaleFactor = pow( originImgH * 1.0 / scaleInitH, 1.0 / (scaleTimes-1) );
        else
            scaleFactor = 1.0;

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

    void init_zhou_small(){
        originImgW = 640;
        originImgH = 480;
        cameraFx = 525.0f;
        cameraFy = 525.0f;
        cameraCx = 319.5f;
        cameraCy = 239.5f;
        // origin depth img's resolution
        originDepthW = originImgW;
        originDepthH = originImgH;
        // depth camera's data
        cameraDFx = cameraFx;
        cameraDFy = cameraFy;
        cameraDCx = cameraCx;
        cameraDCy = cameraCy;

        keyFramesPath = "/home/eagle/GitHub/EAGLE-TextureMapping/datas";
        rgbNameExt = "jpg";
        kfRGBNamePattern = "%05d." + rgbNameExt;
        kfRGBMatch = "*." + rgbNameExt;
        kfDNamePattern = "%05d.png";
        depthType = 's';
        camTrajFromWorldToCam = false;
        plyFile = "world.ply";
        kfIndexs = {0,1,2,3,4,5,6,7,12,13,14,15,17,18,19,20,21};

        resultsPathSurfix = "";

        scaleTimes = 10;
        scaleInitH = originImgH / 4;

        lamda = 10.0;
    }
};

#endif // SETTINGS_H
