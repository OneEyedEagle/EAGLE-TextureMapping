#ifndef GETALIGNRESULTS_H
#define GETALIGNRESULTS_H

#include <stdio.h>
#include <string>
#include <vector>
#include <omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
//#include <pcl/visualization/pcl_visualizer.h>

#include "settings.h"
#include "Eagle_Utils.h"

class Eagle_Infos;

class getAlignResults
{
public:
    Settings settings;
    std::ofstream log;

    std::vector<cv::Mat1f> cameraPoses; // cameraPos matrix's array
    std::vector<cv::Mat3f> sourcesNormals; // origin source's point normals

    std::string sourcesPath; // Si - sources' path
    std::string targetsPath; // Ti - targets' path
    std::string texturesPath; // Mi - textures' path
    std::string resultsPath; // save results with different resolutions
    std::string pmResultPath; // path to store patchmatch results

    std::vector<cv::String> sourcesFiles; // all sources' full path (with filename and ext)
    std::vector<cv::String> targetsFiles;
    std::vector<cv::String> texturesFiles;
    size_t kfStart, kfTotal;
    std::vector<size_t> kfIndexs;

    std::vector<cv::Mat1i> depthsImgs;
    std::vector<cv::Mat3b> sourcesImgs;
    std::vector<cv::Mat3b> targetsImgs;
    std::vector<cv::Mat3b> texturesImgs;
    cv::Rect rectImg;

    float scaleF;

    getAlignResults(Settings &_settings);
    void LOG(std::string t, bool nl = true);
    void DEBUG(std::string t);

    size_t getImgID(std::string img_file);
    std::string getImgFile(size_t img_i);
    std::string getImgDFile(size_t img_i);

    std::string getImgFilename(size_t img_i, std::string pre, std::string ext);
    std::string getImgPLYFile(size_t img_i);
    std::string getImgXMLFile(size_t img_i);

    void loadSourcesPointN();
    void calcSourcesPointN();
    void calcSourceCloud(size_t img_i, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<cv::Point2i> &pos);

    void readCameraTraj(std::string camTraj_file);
    cv::Mat projectToCWorld(int x, int y, int depth);
    cv::Mat projectToCWorld(float x, float y, int depth);
    cv::Mat projectToWorld(cv::Mat X_c, size_t id);
    cv::Mat projectToCWorld(cv::Mat X_w, size_t id);
    int calcDepth(cv::Mat X_w, size_t id);
    cv::Mat projectToImg(cv::Mat X_w, size_t id);
    cv::Mat imgToScale(cv::Mat X_img);
    cv::Mat scaleToImg(cv::Point2i p_img_s);
    double calcWeightJ(size_t img_id, cv::Mat1f p, cv::Mat X_c);

    cv::Vec3b getRGBonImg(cv::Mat img, cv::Mat X_img);
    int getDonImg(cv::Mat depths, cv::Mat X_img);
    cv::Vec3f getNonImg(cv::Mat normals, cv::Mat X_img);

    bool isPointVisible(cv::Mat X_w, size_t id);
    bool isPointOnRect(cv::Mat X, cv::Rect r);
    bool isPointOnRect(cv::Point2i p, cv::Rect r);
    bool isPointOnRect(int px, int py, cv::Rect r);
    bool isPointOnRect(float px, float py, cv::Rect r);

    void calcPatchmatch();
    void patchMatch(std::string imgA_file, std::string imgB_file, std::string ann_raw_file, std::string annd_file);
    std::string getAnnFilename(std::string img_file, std::string sym);
    std::string getAnndFilename(std::string img_file, std::string sym);
    void readAnnTXT(std::string ann_txt_file, cv::Mat1i &result);

    void generateTargets();
    void generateTargetI(size_t target_id, std::vector<cv::Mat3b> textures);
    std::vector<cv::Point2i> getPixelXYonSourceS2T(cv::Mat1i result_ann_s2t, cv::Point2i p);
    std::vector<cv::Point2i> getPixelXYonSourceT2S(cv::Mat1i result_ann_t2s, cv::Point2i p);
    void getPatchOnImg(cv::Rect &r, int x, int y);

    void generateTextures();
    void generateTextureI(size_t texture_id, std::vector<cv::Mat3b> targets);

    void savePlysImage(std::string path, std::vector<cv::Mat3b> imgs);
    void saveImagePly(std::string ply_fullname, size_t tid, cv::Mat3b img, cv::Mat1i depths);
    void savePlys(std::string path);
    void saveImagesPly(std::string ply_fullname, std::vector<cv::Mat3b> imgs);

};

#endif // GETALIGNRESULTS_H
