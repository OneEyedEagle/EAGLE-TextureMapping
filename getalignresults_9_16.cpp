#include "getalignresults.h"

/*----------------------------------------------
 *  Log Settings
 * ---------------------------------------------*/
#define LOG_INIT_N true
#define LOG_SAVE_T true
#define LOG_SAVE_M true

/*----------------------------------------------
 *  To get the position of the patchmatch result
 * ---------------------------------------------*/
#define INT_TO_X(v) ((v)&((1<<12)-1))
#define INT_TO_Y(v) ((v)>>12)

/*----------------------------------------------
 *  Math
 * ---------------------------------------------*/
#define EPS 1e-10

/*----------------------------------------------
 *  Main
 * ---------------------------------------------*/
getAlignResults::getAlignResults(Settings &_settings)
{
    settings = _settings;
    /*----------------------------------------------
     *  DEBUG
     * ---------------------------------------------*/
    // don't do the optimization steps?
    bool onlyGenePlyWithBackups = false;
    // continue the last iteration with its Tis and Mis?
    bool continueIterForOnce = false;
    // current scale's count (based on the origin resolution)
    int scale = 0;
    //scale = settings.scaleTimes - 1;

    LOG("[ From Path: " + settings.keyFramesPath + " ] ");
    // get all keyframe imgs' full path
    sourcesPath = settings.keyFramesPath;
    EAGLE::checkPath(sourcesPath);
    cv::glob(sourcesPath + "/" + settings.kfRGBMatch, sourcesFiles, false);
    std::vector<cv::String> originSourcesFiles(sourcesFiles); // origin sources
    std::vector<cv::String> originDepthFiles; // origin depths
    cv::glob(sourcesPath + "/" + settings.kfDMatch, originDepthFiles, false);
    // range of all frames
    kfStart = 0;
    kfTotal = sourcesFiles.size();
    // range of valid frames
    if( settings.kfIndexs.size() > 0 ) {
        kfIndexs = settings.kfIndexs;
    } else {
        kfIndexs.clear();
        for( size_t i = kfStart; i < kfTotal; i++ )
            kfIndexs.push_back(i);
    }
    // rect of Image
    rectImg.x = 0; rectImg.y = 0; rectImg.width = settings.originImgW; rectImg.height = settings.originImgH;

    // make the dir to store targets
    targetsPath = sourcesPath + "/targets";
    EAGLE::checkPath(targetsPath);
    // make the dir to store textures
    texturesPath = sourcesPath + "/textures";
    EAGLE::checkPath(texturesPath);
    // make the dir to store results
    resultsPath = sourcesPath + "/results";
    EAGLE::checkPath(resultsPath);
    log.open( resultsPath + "/result.log" );
    // make the dir to store pm's results
    pmResultPath = sourcesPath + "/patchmatchs";
    EAGLE::checkPath(pmResultPath);

    // read the camera's world positions of keyframes
    LOG("[ Read Camera Matrixs ] ");
    readCameraTraj(settings.kfCameraTxtFile);

    // calc all points' normal
    LOG("[ Calc Point Cloud with Normals ] ");
    loadSourcesPointN();

    // read depth maps
    depthsImgs.clear();
    for(size_t i = 0; i < kfTotal; i++)
        depthsImgs.push_back( cv::imread(getImgDFile(i), CV_LOAD_IMAGE_UNCHANGED) );

    // ONLY generate the ply with current colors
    if(onlyGenePlyWithBackups == true){
        LOG("[ ONLY generate the Plys with CURRENT targets and textures ]");
        cv::glob(targetsPath + "/" + settings.kfRGBMatch, targetsFiles, false);
        cv::glob(texturesPath + "/" + settings.kfRGBMatch, texturesFiles, false);
        sourcesImgs.clear(); targetsImgs.clear(); texturesImgs.clear();
        for( size_t i = 0; i < kfTotal; i++ ){
            sourcesImgs.push_back( cv::imread(sourcesFiles[i]) );
            targetsImgs.push_back( cv::imread(targetsFiles[i]) );
            texturesImgs.push_back( cv::imread(texturesFiles[i]) );
        }
        scaleF = 1; // adjust the scale manually
        saveImagesPly(resultsPath + "/" + settings.plySColorFilename, sourcesImgs);
        scaleF = 1;
        saveImagesPly(resultsPath + "/" + settings.plyTColorFilename, targetsImgs);
        scaleF = 1;
        saveImagesPly(resultsPath + "/" + settings.plyMColorFilename, texturesImgs);
        LOG("[ END ]");
        log.close();
        return;
    }

    LOG("[ All inits Success. " + std::to_string(kfIndexs.size()) + " / " + std::to_string(kfTotal) + " Images. " +
        std::to_string(settings.originImgW) + "x" + std::to_string(settings.originImgH) + " ]");
    LOG("[ Alpha: " + std::to_string(settings.alpha) + " Lambda: " + std::to_string(settings.lambda) + " ] ");

    // multiscale
    for ( ; scale < settings.scaleTimes; scale++) {
        int iter_count = 100 - scale * 15;

        // downsample imgs
        settings.imgW = round(settings.scaleInitW * pow(settings.scaleFactor, scale));
        settings.imgH = round(settings.scaleInitH * pow(settings.scaleFactor, scale));
        scaleF = pow(settings.scaleFactor, settings.scaleTimes-1-scale);

        char tmp[10];
        sprintf(tmp, "%dx%d", settings.imgW, settings.imgH);
        std::string newResolution(tmp);
        LOG("[ Scale to " + newResolution + " ]");

        // get all keyframes' full path (after scale)
        sourcesPath = settings.keyFramesPath + "/" + newResolution;
        EAGLE::checkPath(sourcesPath);
        // generate source imgs with new resolution
        for( size_t i = 0; i < kfTotal; i++ ) {
            // [REQUIRE] ImageMagick
            system( ("convert " + originSourcesFiles[i] + " -resize " + newResolution + "! " + getImgFile(i)).c_str() );
        }
        cv::glob(sourcesPath + "/" + settings.kfRGBMatch, sourcesFiles, false);
        // read Si
        sourcesImgs.clear();
        for ( size_t i = 0; i < kfTotal; i++ ) {
            sourcesImgs.push_back( cv::imread(sourcesFiles[i]) ); // img.at<cv::Vec3b>(y, x)(0)
        }
        //if(settings.scaleTimes - 1 == scale)
        //    savePlysImage(sourcesPath, sourcesImgs);

        // continue the iteration in the last time
        if ( continueIterForOnce ) {
            LOG("[ Continue last iteration... ]");
            cv::glob(targetsPath + "/" + settings.kfRGBMatch, targetsFiles, false);
            cv::glob(texturesPath + "/" + settings.kfRGBMatch, texturesFiles, false);
            continueIterForOnce = false;
        }
        // init Ti and Mi or upsample all Ti and Mi
        if ( targetsFiles.size() == 0 ) {
            for( size_t i = 0; i < kfTotal; i++ ) {
                system( ("cp " + sourcesFiles[i] + " " + targetsPath).c_str() );
                system( ("cp " + sourcesFiles[i] + " " + texturesPath).c_str() );
            }
            cv::glob(targetsPath + "/" + settings.kfRGBMatch, targetsFiles, false);
            cv::glob(texturesPath + "/" + settings.kfRGBMatch, texturesFiles, false);
        }else{
            for( size_t i = 0; i < kfTotal; i++ ){
                // [REQUIRE] ImageMagick
                system( ("convert " + targetsFiles[i] + " -resize " + newResolution + "! " + targetsFiles[i]).c_str() );
                system( ("convert " + texturesFiles[i] + " -resize " + newResolution + "! " + texturesFiles[i]).c_str() );
            }
        }

        // do iterations
        for ( int _count = 0; _count < iter_count; _count++) {
            LOG("[ Iteration " + std::to_string(_count+1) + " at " + newResolution + " ]");
            calcPatchmatch();
            generateTargets(); // also output the target imgs for patchmatch
            generateTextures();

            // [add new] set the target = texture
            for( auto i = kfIndexs.begin(); i != kfIndexs.end(); i++ )
                system( ("cp " + texturesFiles[*i] + " " + targetsFiles[*i]).c_str() );
        }

        // save results
        // make the sub-folder to save results
        std::string texturesResultPath = resultsPath + "/" + newResolution;
        EAGLE::checkPath(texturesResultPath);
        for( auto i = kfIndexs.begin(); i != kfIndexs.end(); i++ ){
            system( ("cp " + texturesFiles[*i] + " " + texturesResultPath+"/" + getImgFilename(*i, "M_", "."+settings.rgbNameExt)).c_str() );
            system( ("cp " + targetsFiles[*i] + " " + texturesResultPath+"/" + getImgFilename(*i, "T_", "."+settings.rgbNameExt)).c_str() );
        }
        if(settings.scaleTimes - 2 <= scale)
            savePlys(texturesResultPath);
        LOG( "[ Results Saving Success ]" );
    }
    LOG( "[ End ]" );
    log.close();
}

/*----------------------------------------------
 *  LOG
 * ---------------------------------------------*/
void getAlignResults::LOG(std::string t, bool nl)
{
    std::cout << t;
    log << t;
    if (nl) {
        std::cout << std::endl;
        log << std::endl;
    } else {
        std::cout << std::flush;
        log << std::flush;
    }
}
void getAlignResults::DEBUG(std::string t)
{
    std::cout << " DEBUG | " << t << std::endl;
}

/*----------------------------------------------
 *  Image File
// [caution] these all will be affected by the scale step
 * ---------------------------------------------*/
// img_file = "/home/wsy/EAGLE/00001.jpg"
// return 1
size_t getAlignResults::getImgID(std::string img_file)
{
    std::string img_filename = EAGLE::getFilename(img_file, false);
    return atoi( img_filename.c_str() );
}
// id = 1
// return "/home/wsy/EAGLE/00001.jpg"
std::string getAlignResults::getImgFile(size_t img_i)
{
    char buf[18];
    sprintf(buf, (settings.kfRGBNamePattern).c_str(), img_i);
    return sourcesPath + "/" + std::string(buf);
}
// id = 1
// return "/home/wsy/EAGLE/00001.png"
std::string getAlignResults::getImgDFile(size_t img_i)
{
    char buf[18];
    sprintf(buf, (settings.kfDNamePattern).c_str(), img_i);
    return sourcesPath + "/" + std::string(buf);
}

std::string getAlignResults::getImgFilename(size_t img_i, std::string pre, std::string ext)
{
    char filename[18] = "\n";
    sprintf(filename, (pre + "%03d" + ext).c_str(), img_i);
    std::string filename_ = filename;
    return filename_;
}
// id = 1
// return "/home/wsy/EAGLE/00001.ply"
std::string getAlignResults::getImgPLYFile(size_t img_i)
{
    return sourcesPath + "/" + getImgFilename(img_i, "imgModel_", ".ply");
}
// id = 1
// return "/home/wsy/EAGLE/00001.xml"
std::string getAlignResults::getImgXMLFile(size_t img_i)
{
    return sourcesPath + "/" + getImgFilename(img_i, "imgNormal_", ".xml");
}

/*----------------------------------------------
 *  Point Normals
 * ---------------------------------------------*/
void getAlignResults::loadSourcesPointN()
{
    sourcesNormals.clear();
    std::vector<cv::String> pnXMLFiles;
    cv::glob(sourcesPath + "/*.xml", pnXMLFiles, false);
    if(pnXMLFiles.size() == 0){
        calcSourcesPointN();
        return;
    }
    if(LOG_INIT_N)
        LOG(" Load Point Normals(.XML) << ", false);
    cv::FileStorage fs;
    for (size_t i = 0; i < pnXMLFiles.size(); i++) {
        fs.open(pnXMLFiles[i], cv::FileStorage::READ);
        cv::Mat3f mat( cv::Size(settings.imgW, settings.imgH) );
        fs["imgPointNormals"] >> mat;
        sourcesNormals.push_back(mat);
        fs.release();
        if(LOG_INIT_N)
            LOG( std::to_string(i) + " ", false );
    }
    if(LOG_INIT_N)
        LOG( "<< Done" );
}
// for every origin source img i,
//  calc each pixel's normal in 3D (camera i as the world origin)
void getAlignResults::calcSourcesPointN()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<cv::Point2i> onImgPos;
    cv::FileStorage fs;
    if(LOG_INIT_N)
        LOG(" Output Pointcloud(.PCD) and Point Normals(.XML) << ", false);
    for (size_t i = 0; i < sourcesFiles.size(); i++) {
        calcSourceCloud(i, cloud, onImgPos);
        // calc point normals
        // ( source: https://blog.csdn.net/wolfcsharp/article/details/93711068 )
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //  using kdtree to search NN
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        //  set the NN value
        n.setKSearch(20);
        n.compute(*normals);

        cv::Mat3f mat( cv::Size(settings.imgW, settings.imgH) );
        for ( size_t j = 0; j < cloud->points.size(); j++) {
            //std::cout << j << " Point: " << cloud->points[j].x << " " << cloud->points[j].y << " " << cloud->points[j].z << std::endl;
            //std::cout << "  N: " << normals->points[j].normal_x << " " << normals->points[j].normal_y << " " << normals->points[j].normal_z << std::endl;
            mat.at<cv::Vec3f>(onImgPos[j].y, onImgPos[j].x) = cv::Vec3f(normals->points[j].normal_x, normals->points[j].normal_y, normals->points[j].normal_z);
        }
        sourcesNormals.push_back(mat);

        fs.open( getImgXMLFile(i), cv::FileStorage::WRITE );
        fs << "imgPointNormals" << mat;
        fs.release();

        cloud->points.clear();
        onImgPos.clear();
        if(LOG_INIT_N)
            LOG( std::to_string(i) + " ", false );
    }
    if(LOG_INIT_N)
        LOG( "<< Done" );
}
// calculate the image's point cloud (3D), with each point's position on img (to store the point normal)
void getAlignResults::calcSourceCloud(size_t img_i, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<cv::Point2i> &pos)
{
    cv::Mat1i depth_i = cv::imread(getImgDFile(img_i), CV_LOAD_IMAGE_UNCHANGED);
    for(int y = 0; y < settings.imgH; y++) {
        for(int x = 0; x < settings.imgW; x++) {
            int d = depth_i.at<int>(y, x);
            if( d == 0 )
                continue;
            cv::Mat X_c = projectToCWorld(x, y, d);
            pcl::PointXYZ p(X_c.at<float>(0), X_c.at<float>(1), X_c.at<float>(2));
            cloud->points.push_back(p);
            pos.push_back( cv::Point2i(x, y) );
        }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::io::savePLYFile( getImgPLYFile(img_i), *cloud );
}

/*----------------------------------------------
 *  Camera
 * ---------------------------------------------*/
void getAlignResults::readCameraTraj(std::string camTraj_file)
{
    std::ifstream  matifs(camTraj_file.c_str());
    int id;
    while( !matifs.eof() )
    {
        cv::Mat1f mat( cv::Size(4, 4) ); // from camera to world
        matifs >> id >> id >> id;
        matifs >> mat.at<float>(0,0) >> mat.at<float>(0,1) >> mat.at<float>(0,2) >> mat.at<float>(0,3);
        matifs >> mat.at<float>(1,0) >> mat.at<float>(1,1) >> mat.at<float>(1,2) >> mat.at<float>(1,3);
        matifs >> mat.at<float>(2,0) >> mat.at<float>(2,1) >> mat.at<float>(2,2) >> mat.at<float>(2,3);
        matifs >> mat.at<float>(3,0) >> mat.at<float>(3,1) >> mat.at<float>(3,2) >> mat.at<float>(3,3);
        if(matifs.fail())
            break;
        cameraPoses.push_back(mat);
    }
    matifs.close();
    /*for ( size_t i = 0; i < cameraPoses.size(); i++)
        std::cout << cameraPoses[i] << std::endl; */
}

// project the point to the camera 3D axis
//   p(x, y) with depth on the (id)th image
//   return 4*1 matrix [x, y, z, 1]
cv::Mat getAlignResults::projectToCWorld(int x, int y, int depth)
{
    return projectToCWorld( (float)x, (float)y, depth);
}
cv::Mat getAlignResults::projectToCWorld(float x, float y, int depth)
{
    cv::Mat X_c = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1); // 4*1
    X_c.at<float>(2) = depth * 1.0f / settings.cameraDepthFactor; // z
    X_c.at<float>(0) = (x - settings.cameraCx) * X_c.at<float>(2) / settings.cameraFx; // x
    X_c.at<float>(1) = (y - settings.cameraCy) * X_c.at<float>(2) / settings.cameraFy; // y
    // here, the world origin is the camera i
    //std::cout << "World Pos (with camera " << texture_id << " as origin): " << X_c.t() << "T" << std::endl;
    return X_c;
}

// project the point (with position in the camera axis) to the world axis
//   return 4*1 matrix [x, y, z, 1]
cv::Mat getAlignResults::projectToWorld(cv::Mat X_c, size_t id)
{
    cv::Mat X_w = cameraPoses[id] * X_c;
    return X_w;
}

// project the world point to the (id)th camera's axis
//   return 4*1 matrix
cv::Mat getAlignResults::projectToCWorld(cv::Mat X_w, size_t id)
{
    cv::Mat R = cameraPoses[id]; // from camera to world
    cv::Mat RT = R.inv(); //  from world to camera
    return RT * X_w;
}

// calculating the depth value with the point's world position under (id)th camera
int getAlignResults::calcDepth(cv::Mat X_w, size_t id)
{
    cv::Mat X_c = projectToCWorld(X_w, id);
    return round( X_c.at<float>(2) * settings.cameraDepthFactor );
}

// project the point to the (id)th image's plane (on origin-resolution)
//   X_w is the point's world position [x, y, z, 1]
//   return 3*1 matrix [x, y, 1]
cv::Mat getAlignResults::projectToImg(cv::Mat X_w, size_t id)
{
    cv::Mat X_c = projectToCWorld(X_w, id);

    cv::Mat1f X_img = (cv::Mat_<float>(3, 1) << X_c.at<float>(0), X_c.at<float>(1), X_c.at<float>(2));
    X_img = settings.cameraK * X_img;
    X_img = X_img / X_c.at<float>(2);
    //std::cout << "from World " << X_w.t() << " to Img " << id << " " << X_c.t() << " " << X_img.t() << std::endl;
    return X_img;
}

// apply the scale to the point p (p is on the origin-resolution img)
cv::Mat getAlignResults::imgToScale(cv::Mat X_img)
{
    cv::Mat1f X_img_s( cv::Size(1,2) );
    X_img_s.at<float>(0) = X_img.at<float>(0) / scaleF;
    X_img_s.at<float>(1) = X_img.at<float>(1) / scaleF;
    return X_img_s;
}

// project the position on the scaled img to the origin-resolution img
cv::Mat getAlignResults::scaleToImg(cv::Point2i p_img_s)
{
    cv::Mat1f X_img( cv::Size(1,2) );
    X_img.at<float>(0) = p_img_s.x * scaleF;
    X_img.at<float>(1) = p_img_s.y * scaleF;
    return X_img;
}

// Wj = (cos(angle))^2 / (d^2)
//  angle is the angle between the surface normal and the viewing direction at image j
//  d is the distance between the camera and the surface
// point p on Ti(on full resolution), X_c is its 3D position under the camera i
double getAlignResults::calcWeightJ(size_t img_id, cv::Mat1f p, cv::Mat X_c)
{
    double d_2 = pow(X_c.at<float>(0), 2) + pow(X_c.at<float>(1), 2) + pow(X_c.at<float>(2), 2);
    if( d_2 < EPS )
        return 0;

    // get the normal on point p on Ti (a unit vector)
    cv::Vec3f N_V = getNonImg(sourcesNormals[img_id], p);

    // calc the angle between N and (0,0,1) the camera's viewing direction
    //double cos_theta = (1 + 1 - (pow(N_V(0), 2) + pow(N_V(1), 2) + pow(N_V(2)-1, 2))) / 2;
    double cos_theta = N_V(2);

    return pow(cos_theta, 2) / d_2;
}

/*----------------------------------------------
 *  RGBD Value
 * ---------------------------------------------*/
// using Bilinear Interpolation to get the RGB value on img's float position
//  X_img is a 3*1 matrix [x, y, 1] corresponding to the img's scale
cv::Vec3b getAlignResults::getRGBonImg(cv::Mat img, cv::Mat X_img)
{
    float x = X_img.at<float>(0), y = X_img.at<float>(1);
    return img.at<cv::Vec3b>( round(y), round(x) );

    int x1 = floor(x), x2 = ceil(x);
    int y1 = floor(y), y2 = ceil(y);
    cv::Vec3b pixel(0,0,0);
    for ( int i = 0; i < 3; i++) {
        pixel(i) = round(
          img.at<cv::Vec3b>(y1, x1)(i) * (x2 - x) * (y2 - y) +
          img.at<cv::Vec3b>(y1, x2)(i) * (x - x1) * (y2 - y) +
          img.at<cv::Vec3b>(y2, x1)(i) * (x2 - x) * (y - y1) +
          img.at<cv::Vec3b>(y2, x2)(i) * (x - x1) * (y - y1) );
    }
    return pixel;
}
int getAlignResults::getDonImg(cv::Mat depths, cv::Mat X_img)
{
    float x = X_img.at<float>(0), y = X_img.at<float>(1);
    return depths.at<int>( round(y), round(x) );

    int x1 = floor(x), x2 = ceil(x);
    int y1 = floor(y), y2 = ceil(y);
    if( depths.at<int>(y1, x1) == 0 || depths.at<int>(y1, x2) == 0 ||
          depths.at<int>(y2, x1) == 0 || depths.at<int>(y2, x2) == 0 )
        return depths.at<int>( round(y), round(x) );
    int depth_img = round(
          depths.at<int>(y1, x1) * (x2 - x) * (y2 - y) +
          depths.at<int>(y1, x2) * (x - x1) * (y2 - y) +
          depths.at<int>(y2, x1) * (x2 - x) * (y - y1) +
          depths.at<int>(y2, x2) * (x - x1) * (y - y1) );
    return depth_img;
}
cv::Vec3f getAlignResults::getNonImg(cv::Mat normals, cv::Mat X_img)
{
    float x = X_img.at<float>(0), y = X_img.at<float>(1);
    return normals.at<cv::Vec3f>( round(y), round(x) );

    int x1 = floor(x), x2 = ceil(x);
    int y1 = floor(y), y2 = ceil(y);
    cv::Vec3f normal(0,0,0);
    for ( int i = 0; i < 3; i++) {
        normal(i) =
          normals.at<cv::Vec3f>(y1, x1)(i) * (x2 - x) * (y2 - y) +
          normals.at<cv::Vec3f>(y1, x2)(i) * (x - x1) * (y2 - y) +
          normals.at<cv::Vec3f>(y2, x1)(i) * (x2 - x) * (y - y1) +
          normals.at<cv::Vec3f>(y2, x2)(i) * (x - x1) * (y - y1);
    }
    return normal;
}

/*----------------------------------------------
 *  Point Check
 * ---------------------------------------------*/
// is the world point visible on the (id)th img's plane?
bool getAlignResults::isPointVisible(cv::Mat X_w, size_t id)
{
    cv::Mat X_img = projectToImg(X_w, id);
    if( !isPointOnRect(X_img, rectImg) )
        return false;

    // get the depth on the origin img plane
    int depth_img = getDonImg(depthsImgs[id], X_img);

    // get the world point's depth on (id)th camera
    int depth_proj = calcDepth(X_w, id);

    if (depth_img > 0 && depth_proj > depth_img + 5) // 35 // add an offset for depth error
        return false;

    return true;
}

bool getAlignResults::isPointOnRect(cv::Mat X, cv::Rect r)
{
    return isPointOnRect(X.at<float>(0), X.at<float>(1), r);
}
bool getAlignResults::isPointOnRect(cv::Point2i p, cv::Rect r)
{
    return isPointOnRect(p.x, p.y, r);
}
bool getAlignResults::isPointOnRect(int px, int py, cv::Rect r)
{
    return isPointOnRect((float)px, (float)py, r);
}
bool getAlignResults::isPointOnRect(float px, float py, cv::Rect r)
{
    if(r.x > px || r.x + r.width - 1 < px)
        return false;
    if(r.y > py || r.y + r.height - 1 < py)
        return false;
    return true;
}

/*----------------------------------------------
 *  PatchMatch
 * ---------------------------------------------*/
void getAlignResults::calcPatchmatch()
{
    std::string source_file, target_file, ann_file, annd_file;
    for (auto i = kfIndexs.begin(); i != kfIndexs.end(); i++) {
        source_file = sourcesFiles[*i];
        target_file = targetsFiles[*i];

        ann_file = pmResultPath + "/" + getAnnFilename(source_file, "s2t.jpg");
        annd_file = pmResultPath + "/" + getAnndFilename(source_file, "s2t.jpg");
        patchMatch(source_file, target_file, ann_file, annd_file);

        ann_file = pmResultPath + "/" + getAnnFilename(source_file, "t2s.jpg");
        annd_file = pmResultPath + "/" + getAnndFilename(source_file, "t2s.jpg");
        patchMatch(target_file, source_file, ann_file, annd_file);
    }
}
// 利用PatchMatch方法查找出其中的相似块
void getAlignResults::patchMatch(std::string imgA_file, std::string imgB_file, std::string ann_file, std::string annd_file)
{
    // bin imageA_filename imageB_filename ann_img_filename annd_img_filename
    std::string buf = settings.patchmatchBinFile + " " + imgA_file + " " + imgB_file + " " + ann_file + " " + annd_file;
    system( buf.c_str() );
}
// img_file = "/home/wsy/EAGLE/00000.jpg"
// sym = "s2t.jpg"
//  return "/home/wsy/EAGLE/00000_s2t.jpg"
std::string getAlignResults::getAnnFilename(std::string img_file, std::string sym)
{
    return EAGLE::getFilename(img_file, false) + "_ann_" + sym;
}
// img_file = "/home/wsy/EAGLE/00001.jpg"
// sym = "t2s.jpg"
//  return "/home/wsy/EAGLE/00001_t2s.jpg"
std::string getAlignResults::getAnndFilename(std::string img_file, std::string sym)
{
    return EAGLE::getFilename(img_file, false) + "_annd_" + sym;
}
// ann_txt_file = "/home/wsy/EAGLE/00000_s2t.txt"
// cv::Mat1i result(480, 640) // for 640X480 img to store the ANN result of PatchMatch
void getAlignResults::readAnnTXT(std::string ann_txt_file, cv::Mat1i &result)
{
    std::ifstream infile(ann_txt_file.c_str(), std::ios::in | std::ios::binary);
    if ( !infile.is_open() ) {
      LOG("!!! Open file [" + ann_txt_file + "] failed!");
      return;
    }
    for( int j = 0; j < result.size().height; j++ ){
        for( int i = 0; i < result.size().width; i++ ){
            infile >> result.at<int>(j, i);
        }
    }
    infile.close();
}

/*----------------------------------------------
 *  Generate Tis
 * ---------------------------------------------*/
// calculate camera's world matrix's inv (4*4)
void getAlignResults::generateTargets()
{
    if(LOG_SAVE_T)
        LOG( " Targets << ", false );
    texturesImgs.clear();
    for( size_t i = 0; i < kfTotal; i++ )
        texturesImgs.push_back( cv::imread(texturesFiles[i]) );
    for( auto i = kfIndexs.begin(); i != kfIndexs.end(); i++ ) {
        generateTargetI(*i, texturesImgs);
        if(LOG_SAVE_T)
            LOG( std::to_string(*i) + " ", false);
    }
    if(LOG_SAVE_T)
        LOG( "<< Done" );
}
void getAlignResults::generateTargetI(size_t target_id, std::vector<cv::Mat3b> textures)
{
    std::string sourceFile = getImgFile( target_id );
    std::string ann_txt = pmResultPath + "/" + getAnnFilename(sourceFile, "s2t.txt");
    cv::Mat1i result_ann_s2t( settings.imgH, settings.imgW );
    readAnnTXT(ann_txt, result_ann_s2t);
    ann_txt = pmResultPath + "/" + getAnnFilename(sourceFile, "t2s.txt");
    cv::Mat1i result_ann_t2s( settings.imgH, settings.imgW );
    readAnnTXT(ann_txt, result_ann_t2s);

    cv::Mat3b target( cv::Size(settings.imgW, settings.imgH) );
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i pixel(i, j);
        std::vector<cv::Point2i> pXY_U = getPixelXYonSourceS2T(result_ann_s2t, pixel);
        std::vector<cv::Point2i> pXY_V = getPixelXYonSourceT2S(result_ann_t2s, pixel);

        cv::Vec3i sum_U(0,0,0), sum_V(0,0,0), sum_M(0,0,0);
        std::vector<cv::Point2i>::iterator it = pXY_U.begin();
        while( it != pXY_U.end() ){
            sum_U += sourcesImgs[target_id].at<cv::Vec3b>(it->y, it->x);
            it++;
        }
        it = pXY_V.begin();
        while( it != pXY_V.end() ){
            sum_V += sourcesImgs[target_id].at<cv::Vec3b>(it->y, it->x);
            it++;
        }

        int count_M = 0;
        cv::Mat1f X_img = scaleToImg(pixel);
        int _depth = getDonImg(depthsImgs[target_id], X_img);
        cv::Mat X_c = projectToCWorld(X_img.at<float>(0), X_img.at<float>(1), _depth);
        double weightJ = calcWeightJ(target_id, X_img, X_c);
        if( _depth > 0 ){
            cv::Mat X_w = projectToWorld(X_c, target_id);
            for ( auto it = kfIndexs.begin(); it != kfIndexs.end(); it++ ) {
                size_t t = *it;
                if (t == target_id){
                    sum_M += textures[t].at<cv::Vec3b>(j, i);
                    count_M += 1;
                } else {
                    if( isPointVisible(X_w, t) ){
                        cv::Mat X_img_t = projectToImg(X_w, t);
                        cv::Mat X_img_t_s = imgToScale(X_img_t);
                        sum_M += getRGBonImg(textures[t], X_img_t_s);
                        count_M += 1;
                    }
                }
            }
        } else {
            sum_M += textures[target_id].at<cv::Vec3b>( j, i );
            count_M += 1;
        }
        //std::cout << "DEBUG | " << target_id << " (" << i << "," << j << ") " << sum_U.t() << sum_V.t() << sum_M.t() << std::endl;

        // generate the pixel of Ti
        cv::Vec3b bgr(0,0,0);
        double _factor = 1.0*pXY_U.size() / settings.patchSize + settings.alpha * pXY_V.size() / settings.patchSize + settings.lambda * weightJ;
        for ( int p_i = 0; p_i < 3; p_i++ ) {
            double tmp = 1.0 * sum_U(p_i) / settings.patchSize + settings.alpha * sum_V(p_i) / settings.patchSize + settings.lambda * weightJ * sum_M(p_i) / count_M;
            bgr(p_i) = round(tmp / _factor);
        }
        //cv::Vec3b _pixel = textures[target_id].at<cv::Vec3b>(j, i);
        //if( _pixel(0) != bgr(0) || _pixel(1) != bgr(1) || _pixel(2) != bgr(2) )
        //    std::cout << "DEBUG | " << target_id << " (" << i << "," << j << ") " << _pixel.t() << bgr.t() << std::endl;
        target.at<cv::Vec3b>(j, i) = bgr;
    }
    cv::imwrite( targetsFiles[target_id], target );
}

// find p's pos in source patches that overlap pixel p (patches are from the completeness process)
//   the completeness process get a patch in target with min D for each patch from source.
//   First we select patches we get in target that overlap pixel p (px, py),
//   then, we return p's corresponding position on the source.
std::vector<cv::Point2i> getAlignResults::getPixelXYonSourceS2T(cv::Mat1i result_ann_s2t, cv::Point2i p)
{
    int v, x, y;
    cv::Rect patch(0, 0, settings.patchWidth, settings.patchWidth);
    std::vector<cv::Point2i> pXYonSource;
    for( int j = 0; j < settings.imgH; j++ ){
        for( int i = 0; i < settings.imgW; i++ ){
            v = result_ann_s2t.at<int>(j, i);
            if( v == 0 ){
                x = i; y = j;
            }else{
                x = INT_TO_X(v); y = INT_TO_Y(v);
            }
            getPatchOnImg(patch, x, y);
            if( isPointOnRect(p, patch) )
                pXYonSource.push_back( cv::Point2i(p.x - patch.x + i, p.y - patch.y + j) );
        }
    }
    return pXYonSource;
}
// find source patches that overlap pixel p (patches are from the coherence process)
//   the coherence process get a patch in source with min D for each patch from target
//   First we select all patches in target that overlap pixel p (px, py),
//   then, we return p's corresponding position on source.
std::vector<cv::Point2i> getAlignResults::getPixelXYonSourceT2S(cv::Mat1i result_ann_t2s, cv::Point2i p)
{
    int v, x, y;
    cv::Rect patch(0, 0, settings.patchWidth, settings.patchWidth);
    std::vector<cv::Point2i> pXYonSource;
    for(int j = 0 ; j < settings.imgH; j++ ){
        for(int i = 0 ; i < settings.imgW; i++ ){
            getPatchOnImg(patch, i, j);
            if( isPointOnRect(p, patch) ){
                v = result_ann_t2s.at<int>(j, i);
                if (v == 0){
                    x = i; y = j;
                }else{
                    x = INT_TO_X(v); y = INT_TO_Y(v);
                }
                pXYonSource.push_back( cv::Point2i(p.x - patch.x + x, p.y - patch.y + y) );
            }
        }
    }
    return pXYonSource;
}

void getAlignResults::getPatchOnImg(cv::Rect &r, int x, int y)
{
    r.x = x;
    r.y = y;
    r.width = settings.patchWidth;
    r.height = settings.patchWidth;
    if ( r.x + r.width > settings.imgW )
        r.width = settings.imgW - r.x;
    if ( r.y + r.height > settings.imgH )
        r.height = settings.imgH - r.y;
}

/*----------------------------------------------
 *  Generate Mis
 * ---------------------------------------------*/
void getAlignResults::generateTextures()
{
    if(LOG_SAVE_M)
        LOG( " Textures << ", false );
    targetsImgs.clear();
    for( size_t i = 0; i < kfTotal; i++ )
       targetsImgs.push_back( cv::imread(targetsFiles[i]) );
    for( auto i = kfIndexs.begin(); i != kfIndexs.end(); i++ ) {
        generateTextureI(*i, targetsImgs);
        if(LOG_SAVE_M)
            LOG( std::to_string(*i) + " ", false);
    }
    if(LOG_SAVE_M)
        LOG("<< Done");
}
void getAlignResults::generateTextureI(size_t texture_id, std::vector<cv::Mat3b> targets)
{
    cv::Mat3b texture( cv::Size(settings.imgW, settings.imgH) ); // texturesImgs[texture_id]
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i p_img_s(i, j);
        cv::Mat X_img = scaleToImg(p_img_s);
        int depth = getDonImg(depthsImgs[texture_id], X_img);
        if( depth > 0 ) {
            cv::Mat X_c = projectToCWorld(X_img.at<float>(0), X_img.at<float>(1), depth);
            cv::Mat X_w = projectToWorld(X_c, texture_id);

            cv::Vec3b pixel;
            double sum_b = 0.0, sum_g = 0.0, sum_r = 0.0;
            double sum_weightJ = 0.0, weightJ = 0.0;
            for ( auto it = kfIndexs.begin(); it != kfIndexs.end(); it++ ) {
                size_t t = *it;
                if ( t == texture_id ) {
                    weightJ = calcWeightJ(t, X_img, X_c);
                    pixel = targets[t].at<cv::Vec3b>(j, i);
                } else {
                    if ( isPointVisible(X_w, t) ) {
                        cv::Mat X_img_t = projectToImg(X_w, t);
                        cv::Mat X_c_t = projectToCWorld(X_w, t);
                        weightJ = calcWeightJ(t, X_img_t, X_c_t);

                        cv::Mat X_img_t_s = imgToScale(X_img_t);
                        pixel = getRGBonImg(targets[t], X_img_t_s);
                    }
                }
                if ( weightJ > 0 ){
                    sum_weightJ += weightJ;
                    // process seperately, case it may go out the range
                    sum_b = sum_b + weightJ * (pixel(0) * 1.0);
                    sum_g = sum_g + weightJ * (pixel(1) * 1.0);
                    sum_r = sum_r + weightJ * (pixel(2) * 1.0);
                }
            }
            if (sum_weightJ > 0) {
                // generate the pixel of Mi
                texture.at<cv::Vec3b>(j, i)(0) = round( sum_b / sum_weightJ );
                texture.at<cv::Vec3b>(j, i)(1) = round( sum_g / sum_weightJ );
                texture.at<cv::Vec3b>(j, i)(2) = round( sum_r / sum_weightJ );
            } else { // the current point's sum of weightJ is 0, then it's given up
                texture.at<cv::Vec3b>(j, i) = targets[texture_id].at<cv::Vec3b>(j, i);
            }
            //std::cout << "DEBUG | Texture " << texture_id << " (" << i << ", " << j << ") " << "(" << sum_b << ", " << sum_g << ", " << sum_r << ") " << sum_weightJ << std::endl;
        } else { // if depth is 0, then no optimization
            texture.at<cv::Vec3b>(j, i) = targets[texture_id].at<cv::Vec3b>(j, i);
        }
    }
    cv::imwrite( texturesFiles[texture_id], texture );
}

/*----------------------------------------------
 *  Generate PLY with colors
 * ---------------------------------------------*/
void getAlignResults::savePlys(std::string path)
{
    // save ply with colors
    saveImagesPly(path + "/" + settings.plySColorFilename, sourcesImgs);
    LOG("[ PLY with Sources Success ]");
    saveImagesPly(path + "/" + settings.plyTColorFilename, targetsImgs);
    LOG("[ PLY with Targets Success ]");
    saveImagesPly(path + "/" + settings.plyMColorFilename, texturesImgs);
    LOG("[ PLY with Textures Success ]");
}
void getAlignResults::saveImagesPly(std::string ply_fullname, std::vector<cv::Mat3b> imgs)
{
    // read the model with mesh
    pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(settings.plyWorldFile, mesh);
    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
    viewer->addPolygonMesh(mesh, "mesh");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }*/

    // create a RGB point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // convert to PointCloud
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_rgb);

    // for each point, get its average RGB
    for (size_t i = 0; i < cloud_rgb->points.size(); i++) {
        cv::Mat X_w = (cv::Mat_<float>(4, 1) << cloud_rgb->points[i].x, cloud_rgb->points[i].y, cloud_rgb->points[i].z, 1); // 4*1
        //std::cout << "\tDEBUG | X_world: " << X_w.t() << std::endl;
        cv::Vec3i sum_M(0,0,0);
        int count_M = 0;
        for (auto it = kfIndexs.begin(); it != kfIndexs.end(); it++) {
            size_t t = *it;
            // do the depth check to remove the invisible 3D point's color
            if( isPointVisible(X_w, t) ){
                cv::Mat X_img = projectToImg(X_w, t);
                cv::Mat X_img_s = imgToScale(X_img);
                cv::Vec3b pixel = getRGBonImg(imgs[t], X_img_s);
                sum_M += pixel;
                count_M += 1;
            }
        }
        if ( count_M > 0 ) {
            cloud_rgb->points[i].b = sum_M(0) * 1.0 / count_M;
            cloud_rgb->points[i].g = sum_M(1) * 1.0 / count_M;
            cloud_rgb->points[i].r = sum_M(2) * 1.0 / count_M;
        }
    }

    // convert to mesh
    pcl::toPCLPointCloud2(*cloud_rgb, mesh.cloud);
    pcl::io::savePLYFile( ply_fullname, mesh );
}

/*----------------------------------------------
 *  Generate PLY with solo image
 * ---------------------------------------------*/
void getAlignResults::savePlysImage(std::string path, std::vector<cv::Mat3b> imgs)
{
    // save ply with only one image and its depth map
    for (size_t tid = 0; tid < kfTotal; tid++) {
        char name[20] = "\n";
        sprintf(name, "img_%03d.ply", tid);
        saveImagePly(path + "/" + name, tid, imgs[tid], depthsImgs[tid]);
    }
    LOG("[ PLY with Every Source and Depth Success ]");
}
void getAlignResults::saveImagePly(std::string ply_fullname, size_t tid, cv::Mat3b img, cv::Mat1i depths)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    for (int i = 0; i < settings.imgW; i++) {
        for (int j = 0; j < settings.imgH; j++) {
            int depth = depths.at<int>(j, i);
            cv::Point2i p_img(i, j);
            cv::Mat X_c = projectToCWorld(p_img.x, p_img.y, depth);
            if( depth == 0 )
                continue;
            cv::Mat X_w = projectToWorld(X_c, tid);
            pcl::PointXYZRGB point;
            point.x = X_w.at<float>(0);
            point.y = X_w.at<float>(1);
            point.z = X_w.at<float>(2);
            point.b = img.at<cv::Vec3b>(p_img.y, p_img.x)(0);
            point.g = img.at<cv::Vec3b>(p_img.y, p_img.x)(1);
            point.r = img.at<cv::Vec3b>(p_img.y, p_img.x)(2);
            cloud->push_back(point);

//            pcl::Normal p_n;
//            p_n._Normal::normal_x = sourcesNormals[tid].at<cv::Vec3f>(j, i)(0);
//            p_n._Normal::normal_y = sourcesNormals[tid].at<cv::Vec3f>(j, i)(1);
//            p_n._Normal::normal_z = sourcesNormals[tid].at<cv::Vec3f>(j, i)(2);
//            normals->push_back(p_n);
        }
    }
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer A"));
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
//    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 20, 0.03f, "normals");
//    while (!viewer->wasStopped()) {
//        viewer->spinOnce();
//    }

    pcl::io::savePLYFile( ply_fullname, *cloud );
}
