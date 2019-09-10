#ifndef GETKEYFRAMES_H
#define GETKEYFRAMES_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "settings.h"
#include "Eagle_Utils.h"

class generateByCameraPose
{
public:
    generateByCameraPose(Settings settings);

    double getDistanceTransform(const Eigen::Matrix4f &transformMat);
    void getAngleTransform(const Eigen::Matrix4f &transformMat, double angleEuler[3]);
    bool isKeyFrame(Eigen::Matrix4f mat);

public:
    double th_Angle;
    double th_Distance;
};

class generateByFrameNum
{
public:
    generateByFrameNum(Settings settings, int num);
};

class searchBestFrameByKF
{
public:
    searchBestFrameByKF(Settings settings, int _begin, int _end);
    float getImageScore(std::string filename);

private:
    int begin = -1, end = 10;
};

class outputKeyframes
{
public:
    outputKeyframes(Settings &settings);
};

class getKeyframes
{
public:
    getKeyframes(Settings &settings);
};

#endif // GETKEYFRAMES_H
