#ifndef GETALIGNRESULTS_H
#define GETALIGNRESULTS_H

#include <stdio.h>
#include <string>
#include <vector>
#include <omp.h>
#include <cfloat>
#include <time.h>
#include <sys/time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include <acc/bvh_tree.h>

#include "settings.h"
#include "Eagle_Utils.h"

class getAlignResults
{
public:
    Settings settings;
    std::ofstream log;

    std::vector<cv::Mat1f> cameraPoses; // cameraPos matrix's array
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;
    size_t point_num, mesh_num;
    std::vector<cv::Vec3f> vertex_normal;

    std::string processPath, resultsPath;
    std::string sourcesPath, targetsPath, texturesPath, weightsPath;

    size_t kfStart, kfTotal;
    std::vector<size_t> kfIndexs;
    std::vector<cv::String> sourcesOrigin; // all sources' full path (with filename and ext)
    std::map<size_t, cv::String> sourcesFiles, targetsFiles, texturesFiles;
    std::map<size_t, cv::Mat3b> sourcesImgs, targetsImgs, texturesImgs;
    std::map<size_t, cv::Mat> depthImgs;

    struct valid_info // a pixel's valid info of the mesh
    {
        float depth = 0; // 0m ~ 1m
        float cos_alpha = 0;
        size_t mesh_id = 0;
    };
    std::map<size_t, std::vector<struct valid_info>> img_valid_info;
    std::map<size_t, cv::Mat> weights;
    std::map<size_t, cv::Mat> img_valid_patch;
    std::map<size_t, std::map<size_t, cv::Mat>> mappings;

    double scaleF;
    double lamda, patchRandomSearch;
    double E1, E2;

    getAlignResults(Settings &_settings);
    ~getAlignResults();
    void LOG(std::string t, bool nl = true);

    std::string getImgFilename(size_t img_i);
    std::string getImgFilename(size_t img_i, std::string pre, std::string ext);

    void readDepthImgs();
    float getDepthRaw(size_t img_i, int x, int y);
    float getDepth(size_t img_i, int x, int y);

    void readCameraTraj(std::string camTraj_file);
    void readCameraTraj();
    cv::Mat cameraToWorld(cv::Mat X_c, size_t id);
    cv::Mat worldToCamera(cv::Mat X_w, size_t id);
    cv::Mat imgToCamera(int x, int y, float z);
    cv::Mat cameraToImg(cv::Mat X_c);
    cv::Mat imgToWorld(int x, int y, float z, size_t id, int is_point = 1);
    cv::Mat worldToImg(cv::Mat X_w, size_t id);
    bool pointValid(int x, int y);
    bool pointValid(cv::Point2i p_img);
    bool pointValid(cv::Point2f p_img);
    bool pointProjectionValid(float point_z, size_t img_id, int x, int y);
    bool pointProjectionValidMesh(float point_z, size_t img_id, int x, int y);
    bool pointOnBoundary(size_t img_id, int x, int y);

    void calcNormals();
    void calcValidMesh();
    typedef acc::BVHTree<unsigned int, math::Vec3f> BVHTree;
    void calcImgValidMesh(size_t img_i, BVHTree &bvhtree);
    void calcValidPatch();
    void calcImgValidPatch(size_t img_i);
    int isPatchValid(size_t img_i, int x, int y);
    void calcRemapping();
    void calcImgRemapping(size_t img_i, size_t img_j);
    void showRemapping();

    void doIterations();
    void doOBJGenerationOnly();

    void patchmatch(size_t img_id, cv::Mat3b a, cv::Mat3b b, cv::Mat3i &ann);
    void patchmatch_iter(size_t img_id, cv::Mat3b a, cv::Mat3b b, cv::Mat3i &ann, int dir);
    void improve_guess(cv::Mat3b a, cv::Mat3b b, int ax, int ay, int &xbest, int &ybest, int &dbest, int bx, int by);
    int dist(cv::Mat3b a, cv::Mat3b b, int ax, int ay, int bx, int by, int cutoff=INT_MAX);

    void generateTargetI(size_t target_id, std::map<size_t, cv::Mat3b> textures);
    void getSimilarityTerm(cv::Mat3b S, cv::Mat3i ann_s2t, cv::Mat3i ann_t2s, cv::Mat4i &su, cv::Mat4i &sv);
    void calcSuv(cv::Mat3b S, int i, int j, cv::Mat4i &s, int x, int y, int w);

    void generateTextureI(size_t texture_id, std::map<size_t, cv::Mat3b> targets);
    void generateTextureIWithS(size_t texture_id, std::string fullname);

    struct face_info
    {
        std::vector<size_t> v_index = std::vector<size_t>(3);
        std::vector<size_t> uv_index = std::vector<size_t>(3);
        std::vector<size_t> n_index = std::vector<size_t>(3);
    };
    void generateTexturedOBJ(std::string path, std::string filename, std::string resultImgNamePattern);
    void saveOBJwithMTL(std::string path, std::string filename, std::string resultImgNamePattern, pcl::PointCloud<pcl::PointXYZRGB> cloud, std::vector<cv::Point2f> uv_coords, std::map<size_t, std::vector<struct face_info>> mesh_info);
    bool checkMeshMapImg(size_t mesh_i, size_t img_i, std::vector<cv::Point2i> &v_uv);
};

#endif // GETALIGNRESULTS_H
