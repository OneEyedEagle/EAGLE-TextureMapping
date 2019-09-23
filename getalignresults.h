#ifndef GETALIGNRESULTS_H
#define GETALIGNRESULTS_H

#include <stdio.h>
#include <string>
#include <vector>
#include <omp.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include "settings.h"
#include "Eagle_Utils.h"

class Eagle_Infos;

class getAlignResults
{
public:
    Settings settings;
    std::ofstream log;

    std::vector<cv::Mat1f> cameraPoses; // cameraPos matrix's array

    std::string sourcesPath; // Si - sources' path
    std::string targetsPath; // Ti - targets' path
    std::string texturesPath; // Mi - textures' path
    std::string resultsPath; // save results with different resolutions
    std::string pmResultPath; // path to store patchmatch results

    double scaleF;
    size_t kfStart, kfTotal;
    std::vector<size_t> kfIndexs;
    std::vector<cv::String> sourcesFiles; // all sources' full path (with filename and ext)
    std::vector<cv::String> targetsFiles;
    std::vector<cv::String> texturesFiles;

    std::vector<cv::Mat3b> sourcesImgs;
    std::vector<cv::Mat3b> targetsImgs;
    std::vector<cv::Mat3b> texturesImgs;

    pcl::PolygonMesh mesh;
    std::map<size_t, std::vector<cv::Point2f>> uvs;
    std::map<size_t, cv::Mat> weights;
    std::map<size_t, std::map<size_t, cv::Mat>> mappings;

    getAlignResults(Settings &_settings);
    ~getAlignResults();
    void LOG(std::string t, bool nl = true);

    std::string getImgFile(size_t img_i);
    std::string getImgFilename(size_t img_i, std::string pre, std::string ext);

    void readCameraTraj(std::string camTraj_file);
    cv::Mat projectToImg(cv::Mat X_w, size_t id);
    bool pointValid(cv::Point2f p_img);
    bool pointValid(cv::Point2i p_img);
    cv::Point2i imgToScale(cv::Point2i p_img);
    cv::Point2i scaleToImg(cv::Point2i p_img_s);

    void calcVertexMapping();
    void calcImgWeight(size_t img_i, std::vector<float> vertex_weight);
    void calcImgMapping(size_t img_i, size_t img_j);
    cv::Mat3f calcPosCoord(cv::Point2f uv1, cv::Point2f uv2, cv::Point2f uv3, cv::Rect &pos);

    void doIterations();

    void calcPatchmatch();
    void patchMatch(std::string imgA_file, std::string imgB_file, std::string ann_raw_file, std::string annd_file);
    std::string getAnnFilename(std::string img_file, std::string sym);
    std::string getAnndFilename(std::string img_file, std::string sym);
    void readAnnTXT(std::string ann_txt_file, cv::Mat1i &result);

    void generateTargets();
    void generateTargetI(size_t target_id, std::vector<cv::Mat3b> textures);
    void getSimilarityTerm(cv::Mat3b S, cv::Mat1i ann_s2t, cv::Mat1i ann_t2s, cv::Mat4i &su, cv::Mat4i &sv);
    void calcSuv(cv::Mat3b S, int i, int j, cv::Mat4i &s, int x, int y, int w);

    void generateTextures();
    void generateTextureI(size_t texture_id, std::vector<cv::Mat3b> targets);

    void generateColoredPLY(std::string path);
};

#endif // GETALIGNRESULTS_H
