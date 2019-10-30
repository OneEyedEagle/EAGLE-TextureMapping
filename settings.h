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

    std::string allFramesPath, cameraTxtFile, camTrajNamePattern;
    std::string keyFramesPath, patchmatchBinFile, originResolution, kfCameraTxtFile, plyFile;
    std::string rgbNamePattern, dNamePattern, kfRGBNamePattern, kfDNamePattern, rgbNameExt, kfRGBMatch, kfDMatch;
    bool camTrajFromWorldToCam;

    float cameraDFx, cameraDFy, cameraDCx, cameraDCy, cameraFx, cameraFy, cameraCx, cameraCy;
    cv::Mat1f cameraK, cameraDK;
    std::vector<size_t> kfIndexs;

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
        
        // if you already have a ply file, than no need to set these variables to use PCL for getting a ply
        //   (more details can be found in main.cpp)
        {
            // all frames' path
            allFramesPath = "/home/wsy/TextureRecover/Datas/bloster2";
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
        keyFramesPath = "/home/wsy/TextureRecover/Results/bloster2_1";
        // keyframes' name pattern
        kfRGBNamePattern = "color_%02d." + rgbNameExt;
        kfRGBMatch = "color_*." + rgbNameExt;
        // actrually, it's no need to read keyframes' depth images as my code uses the remapping algorithm
        //  so this is for an extension.
        {
            kfDNamePattern = "%03d_d.png";
            kfDMatch = "*_d*";
        }
        // valid keyframes ( all are valid if empty)
        kfIndexs = { 1,3,4,5,7,9,11,12 };

        // files necessary under the keyFramesPath folder
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
        
        // if the camera matrix is projection from the world coord to camera coord, set this flag to true,
        //  otherwise, the data is from camera coord to world coord, and inv() will be called.
        camTrajFromWorldToCam = true;

        // the size of patch (patchWidth * patchWidth)
        patchWidth = 5;

        // weight the similarity from Si to Ti
        alpha_u = 1;
        // weight the similarity from Ti to Si
        alpha_v = 0.1;//2;
        // weight the consistency that how much M affects Ti
        lamda = 2.0;//0.1;

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
        scaleTimes = 10;
        scaleInitH = 120;
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
        kfRGBNamePattern = "%05d." + rgbNameExt;
        kfRGBMatch = "*." + rgbNameExt;
        kfIndexs = {0,4,6,13};//{0,4,6,13,17,19,21};
        plyFile = "world.ply";
        camTrajFromWorldToCam = false;
    }
};

#endif // SETTINGS_H
