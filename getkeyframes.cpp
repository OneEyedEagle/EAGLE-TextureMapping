#include "getkeyframes.h"
#define PI180 3.1415926

generateByCameraPose::generateByCameraPose(Settings settings)
{
    th_Angle = 30;
    th_Distance = 0.2;

    std::ifstream  matifs( settings.cameraTxtFile.c_str());
    Eigen::Matrix4f  kfmat;
    std::vector<int>  kfvec;
    int index = 0;
    while(!matifs.eof()) {
        Eigen::Matrix4f  mat;
        int idx;
        matifs>>idx>>idx>>idx;
        matifs>>mat(0,0)>>mat(0,1)>>mat(0,2)>>mat(0,3);
        matifs>>mat(1,0)>>mat(1,1)>>mat(1,2)>>mat(1,3);
        matifs>>mat(2,0)>>mat(2,1)>>mat(2,2)>>mat(2,3);
        matifs>>mat(3,0)>>mat(3,1)>>mat(3,2)>>mat(3,3);
        if(index == 0) {
            kfmat = mat;
            kfvec.push_back(index);
        }
        if(matifs.fail())
            break;
        Eigen::Matrix4f  tmp = mat.inverse()*kfmat;
        if(isKeyFrame(tmp)) {
            kfmat = mat;
            kfvec.push_back(index);
        }
        index++;
    }
    matifs.close();

    std::ofstream  keyifs( (settings.keyFramesPath + "/" + settings.kfPoseTxtFile).c_str() );
    for(size_t i = 0; i < kfvec.size(); i++) {
        keyifs<<kfvec[i]<<std::endl;
    }
    keyifs.close();
}
double generateByCameraPose::getDistanceTransform(const Eigen::Matrix4f &trans)
{
    Eigen::Vector3d translation(trans(0,3), trans(1,3), trans(2,3));
    return translation.norm();
}

void generateByCameraPose::getAngleTransform(const Eigen::Matrix4f &trans, double angleEuler[3])
{
    // roll
    angleEuler[0]= atan2(trans(2,1),trans(2,2));
    //pitch
    angleEuler[1]= acos((trans(0,0)+trans(1,1)+trans(2,2) - 1) /2);
    // yaw
    angleEuler[2]= atan2(trans(1,0),trans(0,0));
}
bool generateByCameraPose::isKeyFrame(Eigen::Matrix4f mat)
{
    double maxAngle = th_Angle * PI180;	// convert to radians
    double angle[3];
    getAngleTransform(mat, angle);
    return (fabs(angle[0]) > maxAngle ||
            fabs(angle[1]) > maxAngle ||
            fabs(angle[2]) > maxAngle ||
            getDistanceTransform(mat) > th_Distance);
}


generateByFrameNum::generateByFrameNum(Settings settings, int num)
{
    std::vector<int>  kf;
    for(int i = settings.frameStart; i <= settings.frameEnd; i += num) {
        kf.push_back(i);
    }
    std::ofstream  keyifs( (settings.keyFramesPath + "/" + settings.kfNumTxtFile).c_str());
    for(size_t i = 0; i < kf.size(); i++) {
        keyifs<<kf[i]<<std::endl;
    }
    keyifs.close();
}


searchBestFrameByKF::searchBestFrameByKF(Settings settings, int _begin, int _end)
{
    begin = _begin; end = _end;
    std::ifstream  kfindex( (settings.keyFramesPath + "/" + settings.kfPoseTxtFile).c_str());
    std::vector<int>  kf;
    while(!kfindex.eof()) {
        int kidx;
        kfindex>>kidx;
        if(kfindex.fail())
            break;
        kf.push_back(kidx);
    }
    kfindex.close();

    int num = settings.frameEnd - settings.frameStart + 1;
    std::cout<<"----------->num:"<<num<<std::endl;

    int lastkf = 0;
    std::vector<int>  bestkf;
    for(size_t i = 0; i < kf.size(); i++) {
        int start = kf[i] + begin;
        int stop = kf[i] + end;

        if(start <= lastkf)
            start = lastkf + 1;
        if(stop >= num)
            stop = num - 1;
        int best = kf[i];
        float score = 0.0f;
        std::cout<<"-------------key id:"<<kf[i]<<"  startL"<<start<<"  end:"<<stop<<std::endl;
        for(int j = start; j <= stop; j++) {
            char buf[256];
            sprintf(buf, (settings.allFramesPath + "/" + settings.rgbNamePattern).c_str(), j);
            float tmps = std::abs(getImageScore(buf));
            std::cout<<"idx:"<<j<<"  sorce:"<<tmps<<std::endl;
            if(tmps > score) {
                score = tmps;
                best = j;
            }
        }
        std::cout<<best<<"----------best:"<<score<<std::endl<<std::endl;
        bestkf.push_back(best);
        lastkf = best;
    }

    std::ofstream  keyifs( (settings.keyFramesPath + "/" + settings.kfBestTxtFile).c_str());
    for(size_t i = 0; i < bestkf.size(); i++)
        keyifs<<bestkf[i]<<std::endl;
    keyifs.close();
}
float searchBestFrameByKF::getImageScore(std::string filename)
{
    cv::Mat in = cv::imread(filename);
    cv::Mat  grayIn;
    cv::cvtColor(in,grayIn,CV_BGR2GRAY);
    cv::Mat  Bver;
    cv::Mat  Bhor;
    cv::blur(grayIn, Bver, cv::Size(1,9));
    cv::blur(grayIn, Bhor, cv::Size(9,1));

    double difs_V = 0;
    double difs_H = 0;
    double difB_V = 0;
    double difB_H = 0;
    double somaV=0;
    double somaH = 0;
    double varV = 0;
    double varH = 0;
    for(int i = 0; i < grayIn.rows; i++) {
        for(int j = 0; j < grayIn.cols; j++) {
            if(i >= 1) {
                difs_V = grayIn.at<uchar>(i, j) - grayIn.at<uchar>(i - 1, j);
                difB_V = Bver.at<uchar>(i, j) - Bver.at<uchar>(i - 1, j);
            }
            if(j >= 1) {
                difs_H = grayIn.at<uchar>(i, j) - grayIn.at<uchar>(i, j - 1);
                difB_H = Bhor.at<uchar>(i, j) - grayIn.at<uchar>(i, j - 1);
            }
            varV += cv::max(0.0, difs_V - difB_V);
            varH += cv::max(0.0, difs_H - difB_H);
            somaV += difs_V;
            somaH += difs_H;
        }
    }
    float blur_index = cv::max((somaV-varV)/somaV,(somaH - varH)/somaH);
//    float blur_index =std::sqrt( ((somaV-varV)/somaV)*((somaV-varV)/somaV)+((somaH - varH)/somaH)*((somaH - varH)/somaH));

    return blur_index;
}


outputKeyframes::outputKeyframes(Settings &settings)
{
    std::string kfstring = settings.keyFramesPath + "/" + settings.kfBestTxtFile;
    std::ifstream  kfindex(kfstring.c_str());
    std::vector<int> kf;
    while(!kfindex.eof()) {
        int kidx;
        kfindex>>kidx;
        if(kfindex.fail())
            break;
        kf.push_back(kidx);
    }
    kfindex.close();

    //camera traj
    std::string intraj = settings.cameraTxtFile;
    std::ifstream trajifs(intraj.c_str());
    std::vector<cv::Mat1f> trajs;
    while(!trajifs.eof())
    {
        int kidx;
        trajifs>>kidx>>kidx>>kidx;

        if(trajifs.fail())
        {
            break;
        }
        cv::Mat1f mat( cv::Size(4, 4) );
        trajifs>>mat.at<float>(0,0)>>mat.at<float>(0,1)>>mat.at<float>(0,2)>>mat.at<float>(0,3);
        trajifs>>mat.at<float>(1,0)>>mat.at<float>(1,1)>>mat.at<float>(1,2)>>mat.at<float>(1,3);
        trajifs>>mat.at<float>(2,0)>>mat.at<float>(2,1)>>mat.at<float>(2,2)>>mat.at<float>(2,3);
        trajifs>>mat.at<float>(3,0)>>mat.at<float>(3,1)>>mat.at<float>(3,2)>>mat.at<float>(3,3);
        trajs.push_back(mat);
    }
    trajifs.close();
    std::cout<<"kf size:"<<kf.size()<<" traj size:"<<trajs.size()<<std::endl;


    std::ofstream  orajifs( (settings.keyFramesPath + "/" + settings.kfCameraTxtFile).c_str());
    for(size_t i = 0; i < kf.size(); i++)
    {
        int idx = kf[i];
        std::cout<<"---->"<<idx<<std::endl;
        char buf[256];
        //color
        {
            sprintf(buf, (settings.allFramesPath + "/" + settings.rgbNamePattern).c_str(), settings.frameStart + idx);
            std::string  sysorder;
            sysorder = "cp ";
            sysorder.append(buf);
            sprintf(buf, (settings.keyFramesPath + "/" + settings.kfRGBNamePattern).c_str(), i);
            sysorder.append(" ");
            sysorder.append(buf);
            system(sysorder.c_str());
        }

        //depth
        {
            sprintf(buf, (settings.allFramesPath + "/" + settings.dNamePattern).c_str(), settings.frameStart + idx);
            std::string  sysorder;
            sysorder = "cp ";
            sysorder.append(buf);
            sprintf(buf, (settings.keyFramesPath + "/" + settings.kfDNamePattern).c_str(), i);
            sysorder.append(" ");
            sysorder.append(buf);
            system(sysorder.c_str());
        }

        //trajactory
        {
            cv::Mat mat = trajs[idx];
            orajifs<<i<<" "<<i<<" "<<i<<std::endl;
            orajifs<<mat.at<float>(0,0)<<" "<<mat.at<float>(0,1)<<" "<<mat.at<float>(0,2)<<" "<<mat.at<float>(0,3)<<std::endl;
            orajifs<<mat.at<float>(1,0)<<" "<<mat.at<float>(1,1)<<" "<<mat.at<float>(1,2)<<" "<<mat.at<float>(1,3)<<std::endl;
            orajifs<<mat.at<float>(2,0)<<" "<<mat.at<float>(2,1)<<" "<<mat.at<float>(2,2)<<" "<<mat.at<float>(2,3)<<std::endl;
            orajifs<<mat.at<float>(3,0)<<" "<<mat.at<float>(3,1)<<" "<<mat.at<float>(3,2)<<" "<<mat.at<float>(3,3)<<std::endl;

        }
    }
    trajifs.close();
}


getKeyframes::getKeyframes(Settings &settings)
{
    //generate key frames by camera pose
    generateByCameraPose CP(settings);

    //generate key frames by frame Num
    generateByFrameNum FN(settings, 20);

    //在关键帧附近找到最好图像代替关键帧
    searchBestFrameByKF KF(settings, -5, 15);

    outputKeyframes OUT(settings);
}
