#include "getalignresults.h"

/*----------------------------------------------
 *  Log Settings
 * ---------------------------------------------*/
#define LOG_PM     true
#define LOG_SAVE_T true
#define LOG_SAVE_M true
#define OUTPUT_T_M_INSTANT true

/*----------------------------------------------
 *  To get the position of the patchmatch result
 * ---------------------------------------------*/
#define INT_TO_X(v) ((v)&((1<<12)-1))
#define INT_TO_Y(v) ((v)>>12)

/*----------------------------------------------
 *  Math
 * ---------------------------------------------*/
#define EPS 1e-10
#define EAGLE_MAX(x,y) (x > y ? x : y)
#define EAGLE_MIN(x,y) (x < y ? x : y)

/*----------------------------------------------
 *  Main
 * ---------------------------------------------*/
getAlignResults::getAlignResults(Settings &_settings)
{
    settings = _settings;
    // get all keyframe imgs' full path
    sourcesPath = settings.keyFramesPath;
    EAGLE::checkPath(sourcesPath);
    cv::glob(sourcesPath + "/" + settings.kfRGBMatch, sourcesOrigin, false);
    // range of all frames
    kfStart = 0; kfTotal = sourcesOrigin.size();
    // range of valid frames
    if( settings.kfIndexs.size() > 0 ) {
        kfIndexs = settings.kfIndexs;
    } else {
        kfIndexs.clear();
        for( size_t i = kfStart; i < kfTotal; i++ )
            kfIndexs.push_back(i);
    }
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
    LOG("[ From Path: " + settings.keyFramesPath + " ] ");
    LOG("[ Alpha U: " + std::to_string(settings.alpha_u) + " | Alpha V: " + std::to_string(settings.alpha_v) + " | Lambda: " + std::to_string(settings.lamda) + " ] ");
    LOG("[ Patch Width: " + std::to_string(settings.patchWidth) + " | Patch Step: " + std::to_string(settings.patchStep) + " ]");
    LOG("[ Scale: " + std::to_string(settings.scaleTimes) + " | From " + std::to_string(settings.scaleInitW) + "x" + std::to_string(settings.scaleInitH) + " to " + std::to_string(settings.originImgW) + "x" + std::to_string(settings.originImgH) + " ]");

    //pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(settings.keyFramesPath + "/" + settings.plyFile, mesh);
    point_num = mesh.cloud.width;
    mesh_num = mesh.polygons.size();
    LOG("[ PLY Model: " + std::to_string(point_num) + " vertexs | " + std::to_string(mesh_num) + " faces ]");
    // create a RGB point cloud
    cloud_rgb = pcl::PointCloud<pcl::PointXYZRGB>();
    // convert to PointCloud
    pcl::fromPCLPointCloud2(mesh.cloud, cloud_rgb);
    calcNormals();

    // read the camera's world positions of keyframes
    LOG("[ Read Camera Matrixs ] ");
    if ( EAGLE::isFileExist(settings.keyFramesPath + "/" + settings.kfCameraTxtFile) )
        readCameraTraj(settings.keyFramesPath + "/" + settings.kfCameraTxtFile);
    else
        readCameraTraj();

    LOG("[ Init Success. " + std::to_string(kfIndexs.size()) + " / " + std::to_string(kfTotal) + " Images " + "]");
    doIterations();
    LOG("[ End ]");
}
getAlignResults::~getAlignResults()
{
    log.close();

    sourcesImgs.clear();
    targetsImgs.clear();
    texturesImgs.clear();

    weights.clear();
    for( size_t t : kfIndexs ) {
        mappings[t].clear();
    }
    mappings.clear();
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

/*----------------------------------------------
 *  Image File
 * ---------------------------------------------*/
std::string getAlignResults::getImgFilename(size_t img_i)
{
    char buf[18];
    sprintf(buf, (settings.kfRGBNamePattern).c_str(), img_i);
    std::string name = buf;
    return name;
}
std::string getAlignResults::getImgFilename(size_t img_i, std::string pre, std::string ext)
{
    char filename[18] = "\n";
    sprintf(filename, (pre + "%03d" + ext).c_str(), img_i);
    std::string filename_ = filename;
    return filename_;
}

/*----------------------------------------------
 *  Camera
 * ---------------------------------------------*/
// the cameraPoses are matrixs that project a point from world coord to camera coord
void getAlignResults::readCameraTraj(std::string camTraj_file)
{
    std::ifstream  matifs(camTraj_file.c_str());
    int id;
    while( !matifs.eof() )
    {
        cv::Mat1f mat( cv::Size(4, 4) );
        matifs >> id >> id >> id;
        matifs >> mat.at<float>(0,0) >> mat.at<float>(0,1) >> mat.at<float>(0,2) >> mat.at<float>(0,3);
        matifs >> mat.at<float>(1,0) >> mat.at<float>(1,1) >> mat.at<float>(1,2) >> mat.at<float>(1,3);
        matifs >> mat.at<float>(2,0) >> mat.at<float>(2,1) >> mat.at<float>(2,2) >> mat.at<float>(2,3);
        matifs >> mat.at<float>(3,0) >> mat.at<float>(3,1) >> mat.at<float>(3,2) >> mat.at<float>(3,3);
        if(matifs.fail())
            break;
        if ( ! settings.camTrajFromWorldToCam )
            mat = mat.inv();
        cameraPoses.push_back( mat );
    }
    matifs.close();
    /*for ( size_t i = 0; i < cameraPoses.size(); i++)
        std::cout << cameraPoses[i] << std::endl; */
}
void getAlignResults::readCameraTraj()
{
    char buf[18];
    std::ifstream matifs;
    for( size_t i = 0; i < kfTotal; i++ ){
        sprintf(buf, (settings.camTrajNamePattern).c_str(), i);
        std::string name(buf);
        matifs.open( settings.keyFramesPath + "/" + name );
        cv::Mat1f mat( cv::Size(4, 4) );
        // the first, second, third number are T for camera, others are R for camera
        matifs >> mat.at<float>(0,3) >> mat.at<float>(1,3) >> mat.at<float>(2,3);
        matifs >> mat.at<float>(0,0) >> mat.at<float>(0,1) >> mat.at<float>(0,2);
        matifs >> mat.at<float>(1,0) >> mat.at<float>(1,1) >> mat.at<float>(1,2);
        matifs >> mat.at<float>(2,0) >> mat.at<float>(2,1) >> mat.at<float>(2,2);
        mat.at<float>(3,0) = 0;
        mat.at<float>(3,1) = 0;
        mat.at<float>(3,2) = 0;
        mat.at<float>(3,3) = 1;
        if ( ! settings.camTrajFromWorldToCam )
            mat = mat.inv();
        cameraPoses.push_back(mat);
        matifs.close();
    }
    /*for ( size_t i = 0; i < cameraPoses.size(); i++)
        std::cout << cameraPoses[i] << std::endl; */
}

// project the point from the camera coord to the world coordinate system
cv::Mat getAlignResults::cameraToWorld(cv::Mat X_c, size_t id)
{
    cv::Mat R = cameraPoses[id]; // from world to camera
    return R.inv() * X_c;
}
// project the point from the world to the (id)th camera's coordinate system
cv::Mat getAlignResults::worldToCamera(cv::Mat X_w, size_t id)
{
    cv::Mat R = cameraPoses[id]; // from world to camera
    return R * X_w;
}

// project a pixel to the camera coord
cv::Mat getAlignResults::imgToCamera(int x, int y, float z)
{
    float cx = settings.cameraCx / static_cast<float>(scaleF);
    float cy = settings.cameraCy / static_cast<float>(scaleF);
    float fx = settings.cameraFx / static_cast<float>(scaleF);
    float fy = settings.cameraFy / static_cast<float>(scaleF);

    float z_c = z;
    float x_c = (x * 1.0f - cx) * z_c / fx;
    float y_c = (y * 1.0f - cy) * z_c / fy;
    return (cv::Mat_<float>(4, 1) << x_c, y_c, z_c, 1);
}
cv::Mat getAlignResults::cameraToImg(cv::Mat X_c)
{
    float cx = settings.cameraCx / static_cast<float>(scaleF);
    float cy = settings.cameraCy / static_cast<float>(scaleF);
    float fx = settings.cameraFx / static_cast<float>(scaleF);
    float fy = settings.cameraFy / static_cast<float>(scaleF);

    float x = (X_c.at<float>(0) * fx + X_c.at<float>(2) * cx) / X_c.at<float>(2);
    float y = (X_c.at<float>(1) * fy + X_c.at<float>(2) * cy) / X_c.at<float>(2);
    float z = X_c.at<float>(2);
    return (cv::Mat_<float>(3, 1) << x, y, z);
}

// project a pixel to the world coord system
cv::Mat getAlignResults::imgToWorld(int x, int y, float z, size_t id, int is_point)
{
    cv::Mat X_c = imgToCamera(x, y, z);
    X_c.at<float>(3) = is_point * 1.0f;
    return cameraToWorld(X_c, id);
}
// project the point to the (id)th image's plane (on origin-resolution)
//   X_w is the point's world position [x_w, y_w, z_w, 1]
//   return 3*1 matrix [x_img, y_img, z_c]
cv::Mat getAlignResults::worldToImg(cv::Mat X_w, size_t id)
{
    cv::Mat X_c = worldToCamera(X_w, id);
    return cameraToImg(X_c);
}

// chech if the point is valid under current resolution
bool getAlignResults::pointValid(int x, int y)
{
    if(x < 0 || x >= settings.imgW)
        return false;
    if(y < 0 || y >= settings.imgH)
        return false;
    return true;
}
bool getAlignResults::pointValid(cv::Point2i p_img)
{
    return pointValid(p_img.x, p_img.y);
}
bool getAlignResults::pointValid(cv::Point2f p_img)
{
    return pointValid(static_cast<int>(std::round(p_img.x)), static_cast<int>(std::round(p_img.y)));
}

// chech if the point can project to the position(x,y) on the (img_id)th image
//  point_z is the point's z on the (img_id)th camera
bool getAlignResults::pointProjectionValid(float point_z, size_t img_id, int x, int y)
{
    // check if the point is valid
    if ( !pointValid(x, y) )
        return false;
    // get the position's valid info
    size_t p_index = static_cast<size_t>(x + y * settings.imgW);
    struct valid_info * info = &img_valid_info[img_id][p_index];
    // check the depth (whether the point is occluded)
    if ( point_z > info->z + 0.01f )
        return false;
    // check the weight (if the weight is too small, then it's a bad projection)
    if ( weights[img_id].at<float>(y, x) < 0.1f )
        return false;
    // the point can be projected to the position on img
    return true;
}

/*----------------------------------------------
 *  Pre-Process
 * ---------------------------------------------*/
// for each vertex, calculate its normal
// ( source: https://blog.csdn.net/wolfcsharp/article/details/93711068 )
void getAlignResults::calcNormals()
{
    LOG("[ Calculating Normals of each Vertex ]");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    //std::vector<cv::Vec3f> vertex_normal(point_num); // vertex id => cv::Vec3f
    vertex_normal.resize(point_num);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //  using kdtree to search NN
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    //  set the NN value
    n.setKSearch(20);
    n.compute(*normals);

    // output the normals
#pragma omp parallel for
    for ( size_t i = 0; i < cloud->points.size(); i++)
        vertex_normal[i] = cv::Vec3f(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
}

// calculate valid mesh for each pixel on every image
void getAlignResults::calcValidMesh()
{
    LOG("[ Calculating Depth and Weight of every pixel on each Si ]");

    // init ray intersection
    std::vector<unsigned int> faces(mesh_num * 3);
    for( size_t i = 0; i < mesh_num; i++ ) {
        for( size_t v_i = 0; v_i < 3; v_i++ )
            faces[i * 3 + v_i] = mesh.polygons[i].vertices[v_i];
    }
    std::vector<math::Vec3f> vertices(point_num);
    for(size_t i = 0; i < point_num; i++) {
        math::Vec3f v( cloud_rgb.points[i].x, cloud_rgb.points[i].y, cloud_rgb.points[i].z );
        vertices[i] = v;
    }
    BVHTree bvhtree(faces, vertices);

    LOG( " Valid Info << ", false );
    img_valid_info.clear(); // pixel_index => valid_info
    weights.clear();
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
    for( size_t t : kfIndexs ) {
        img_valid_info[t] = std::vector<struct valid_info>(total);
        weights[t] = cv::Mat1f( settings.imgH, settings.imgW, 0.0 );
        for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
            struct valid_info info;
            img_valid_info[t][pixel_index] = info;
        }
        calcImgValidMesh(t, bvhtree);
        LOG( std::to_string(t) + " ", false );
    }
    LOG( "<< Done" );
}
// using the ray intersection method to get the pixel's depth
void getAlignResults::calcImgValidMesh(size_t img_i, BVHTree &bvhtree)
{
    cv::Mat cam_c = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat cam_w = cameraToWorld(cam_c, img_i);
    math::Vec3f cam_world_p(cam_w.at<float>(0), cam_w.at<float>(1), cam_w.at<float>(2));

    cv::Mat cam_v_c = (cv::Mat_<float>(4, 1) << 0, 0, 1, 0);
    cv::Mat cam_v_w = cameraToWorld(cam_v_c, img_i);
    math::Vec3f cam_world_v(cam_v_w.at<float>(0), cam_v_w.at<float>(1), cam_v_w.at<float>(2));
    cam_world_v = cam_world_v.normalized();

    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;
        BVHTree::Ray ray;
        ray.origin = cam_world_p;

        cv::Mat p_V = imgToWorld(x, y, 1.0, img_i, 0);
        math::Vec3f v(p_V.at<float>(0), p_V.at<float>(1), p_V.at<float>(2));
        ray.dir = v.normalize();

        ray.tmin = 0.0f;
        ray.tmax = std::numeric_limits<float>::infinity();

        BVHTree::Hit hit;
        if(bvhtree.intersect(ray, &hit)) {
            struct valid_info * info = &img_valid_info[img_i][pixel_index];
            // intersection face's id: hit.idx
            // its points ids:  hit.idx * 3 + 0, hit.idx * 3 + 1, hit.idx * 3 + 2
            info->mesh_id = hit.idx;

            math::Vec3f const & w = hit.bcoords;
            info->lamda = cv::Vec3f( w(0), w(1), w(2) );

            float z = cam_world_v.dot(hit.t * ray.dir);
            info->z = z;

            // calc weight
            cv::Vec3f n1 = vertex_normal[ mesh.polygons[hit.idx].vertices[0] ];
            cv::Vec3f n2 = vertex_normal[ mesh.polygons[hit.idx].vertices[1] ];
            cv::Vec3f n3 = vertex_normal[ mesh.polygons[hit.idx].vertices[2] ];
            math::Vec3f normal;
            normal(0) = n1(0) * w(0) + n2(0) * w(1) + n3(0) * w(2);
            normal(1) = n1(1) * w(0) + n2(1) * w(1) + n3(1) * w(2);
            normal(2) = n1(2) * w(0) + n2(2) * w(1) + n3(2) * w(2);
            normal = normal.normalize();

            math::Vec3f vert2view = -ray.dir;
            float cos_alpha = -vert2view.dot(normal); // the cos of angle between camera dir and vertex normal
            float weight = cos_alpha * cos_alpha / (z * z);
            weights[img_i].at<float>(y, x) = weight;
        }
    }
}

// for every triangle mesh, do projection from i to j
void getAlignResults::calcRemapping()
{
    //std::map<size_t, std::map<size_t, cv::Mat>> mappings;
    mappings.clear();
    LOG("[ Image Remapping ]");
    for( size_t img_i : kfIndexs) {
        mappings[img_i] = std::map<size_t, cv::Mat>();
        LOG( " " + std::to_string(img_i) + " to ", false );
        for( size_t img_j : kfIndexs ) {
            mappings[img_i][img_j] = cv::Mat3i( cv::Size(settings.imgW, settings.imgH) );
            for( int i = 0; i < settings.imgW; i++ )
                for( int j = 0; j < settings.imgH; j++ )
                    mappings[img_i][img_j].at<cv::Vec3i>(j, i) = cv::Vec3i(0,0,0);
            calcImgRemapping(img_i, img_j);
            LOG( std::to_string(img_j) + " ", false );
        }
        LOG( "<< Done" );
    }
}
// for each pixel in img_i, remapping it to img_j only when the mesh is visible both in i and j
void getAlignResults::calcImgRemapping(size_t img_i, size_t img_j)
{
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
//#pragma omp parallel for
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;

        struct valid_info * info = &img_valid_info[img_i][pixel_index];
        if( info->z <= 0 )
            continue;

        if( img_i == img_j ){
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(0) = x;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(1) = y;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) = 1;
            continue;
        }

        cv::Mat p_w = imgToWorld(x, y, info->z, img_i);
        cv::Mat p_j = worldToImg(p_w, img_j);
        cv::Point2i p_img_j( static_cast<int>(round(p_j.at<float>(0))), static_cast<int>(round(p_j.at<float>(1))));
        if ( !pointProjectionValid(p_j.at<float>(2), img_j, p_img_j.x, p_img_j.y) )
            continue;

        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(0) = p_img_j.x;
        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(1) = p_img_j.y;
        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) = 1;
    }
}

/*----------------------------------------------
 *  Do Iterations
 * ---------------------------------------------*/
void getAlignResults::doIterations()
{
    size_t scale = 0;
//    scale = settings.scaleTimes-1;
    bool init_T_M = true;
    for ( ; scale < settings.scaleTimes; scale++) {
        // downsample imgs
        settings.imgW = static_cast<int>(std::round(settings.scaleInitW * 1.0 * pow(settings.scaleFactor, scale)));
        settings.imgH = static_cast<int>(std::round(settings.scaleInitH * 1.0 * pow(settings.scaleFactor, scale)));
        scaleF = settings.originImgW * 1.0 / settings.imgW;

        char tmp[10];
        sprintf(tmp, "%dx%d", settings.imgW, settings.imgH);
        std::string newResolution(tmp);
        LOG("[ Scale to " + newResolution + " ]");

        // get all keyframes' full path (after scale)
        sourcesPath = settings.keyFramesPath + "/" + newResolution;
        EAGLE::checkPath(sourcesPath);
        // generate source imgs with new resolution // [REQUIRE] ImageMagick
        sourcesImgs.clear();
        for( size_t i : kfIndexs ) {
            std::string filename = EAGLE::getFilename(sourcesOrigin[i]);
            sourcesFiles[i] = sourcesPath + "/" + filename;
            system( ("convert " + sourcesOrigin[i] + " -resize " + newResolution + "! " + sourcesFiles[i]).c_str() );
            // read Si
            sourcesImgs[i] = cv::imread(sourcesFiles[i]); // img.at<cv::Vec3b>(y, x)(0)
        }

        // using ray intersection method to get all pixels' depth and weight
        calcValidMesh();
//        break;

        // doing the remapping to project a pixel to other views
        calcRemapping();

        // init Ti and Mi or upsample
        if ( init_T_M == true ) {
            for( size_t i : kfIndexs ) {
                std::string filename = EAGLE::getFilename(sourcesFiles[i]);
                targetsFiles[i] = targetsPath + "/" + filename;
                system( ("cp " + sourcesFiles[i] + " " + targetsFiles[i]).c_str() );
                texturesFiles[i] = texturesPath + "/" + filename;
                system( ("cp " + sourcesFiles[i] + " " + texturesFiles[i]).c_str() );
            }
            init_T_M = false;
        }else{
            for( size_t i : kfIndexs ){
                // [REQUIRE] ImageMagick
                system( ("convert " + targetsFiles[i] + " -resize " + newResolution + "! " + targetsFiles[i]).c_str() );
                system( ("convert " + texturesFiles[i] + " -resize " + newResolution + "! " + texturesFiles[i]).c_str() );
            }
        }

        if( ! OUTPUT_T_M_INSTANT) {
            targetsImgs.clear();
            texturesImgs.clear();
            for( size_t i : kfIndexs ) {
                targetsImgs[i] = cv::imread(targetsFiles[i]);
                texturesImgs[i] = cv::imread(texturesFiles[i]);
            }
        }

        // do iterations
        for ( size_t _count = 0; _count < settings.scaleIters[scale]; _count++) {
            LOG("[ Iteration " + std::to_string(_count+1) + " at " + newResolution + " ]");
            calcPatchmatch();
            generateTargets();
            generateTextures();
        }

        if( ! OUTPUT_T_M_INSTANT)
            for( size_t i : kfIndexs ){
                cv::imwrite( targetsFiles[i], targetsImgs[i] );
                cv::imwrite( texturesFiles[i], texturesImgs[i] );
            }

        // save results
        std::string texturesResultPath = resultsPath + "/" + newResolution;
        EAGLE::checkPath(texturesResultPath);
        for( size_t i : kfIndexs ){
            generateTextureIWithS(i, texturesResultPath+"/" +getImgFilename(i, "S_", "."+settings.rgbNameExt) );
            system( ("cp " + targetsFiles[i] + " " + texturesResultPath+"/" + getImgFilename(i, "T_", "."+settings.rgbNameExt)).c_str() );
            system( ("cp " + texturesFiles[i] + " " + texturesResultPath+"/" + getImgFilename(i, "M_", "."+settings.rgbNameExt)).c_str() );
        }

        LOG( "[ Results at " + newResolution + " Saving Success ]" );
    }
    char tmp[16]; sprintf(tmp, "%dx%d", settings.originImgW, settings.originImgH);
    std::string finalResolution(tmp);
    LOG("[ Generate OBJ file at " + finalResolution + " ]");
    generateTexturedOBJ(resultsPath + "/" + finalResolution, "S", "S_%03d");
    generateTexturedOBJ(resultsPath + "/" + finalResolution, "T", "T_%03d");
    generateTexturedOBJ(resultsPath + "/" + finalResolution, "M", "M_%03d");
}

/*----------------------------------------------
 *  PatchMatch
 * ---------------------------------------------*/
void getAlignResults::calcPatchmatch()
{
    if(LOG_PM)
        LOG( " Patchmatchs << ", false );
    std::string source_file, target_file, ann_file, annd_file;
    for (size_t i : kfIndexs) {
        source_file = sourcesFiles[i];
        target_file = targetsFiles[i];

        ann_file = pmResultPath + "/" + getAnnFilename(i, "s2t.jpg");
        annd_file = pmResultPath + "/" + getAnndFilename(i, "s2t.jpg");
        patchMatch(source_file, target_file, ann_file, annd_file);

        ann_file = pmResultPath + "/" + getAnnFilename(i, "t2s.jpg");
        annd_file = pmResultPath + "/" + getAnndFilename(i, "t2s.jpg");
        patchMatch(target_file, source_file, ann_file, annd_file);

        if(LOG_PM)
            LOG( std::to_string(i) + " ", false);
    }
    if(LOG_PM)
        LOG( "<< Done" );
}
void getAlignResults::patchMatch(std::string imgA_file, std::string imgB_file, std::string ann_file, std::string annd_file)
{
    // bin imageA_filename imageB_filename ann_img_filename annd_img_filename
    std::string buf = settings.patchmatchBinFile + " " + imgA_file + " " + imgB_file + " " + ann_file + " " + annd_file;
    system( buf.c_str() );
}
// img_file = "/home/wsy/EAGLE/00000.jpg"
// sym = "s2t.jpg"
//  return "/home/wsy/EAGLE/00000_s2t.jpg"
std::string getAlignResults::getAnnFilename(size_t img_id, std::string sym)
{
    return std::to_string(img_id) + "_ann_" + sym;
}
// img_file = "/home/wsy/EAGLE/00001.jpg"
// sym = "t2s.jpg"
//  return "/home/wsy/EAGLE/00001_t2s.jpg"
std::string getAlignResults::getAnndFilename(size_t img_id, std::string sym)
{
    return std::to_string(img_id) + "_annd_" + sym;
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
double getAlignResults::calcAnndSum(std::string annd_txt_file)
{
    std::ifstream infile(annd_txt_file.c_str(), std::ios::in | std::ios::binary);
    if ( !infile.is_open() ) {
      LOG("!!! Open file [" + annd_txt_file + "] failed !!!");
      return 0;
    }
    double result = 0;
    int v = 0;
    while( ! infile.eof() ) {
        infile >> v;
        result += (v * 1.0 / settings.patchSize);
    }
    infile.close();
    return result;
}

/*----------------------------------------------
 *  Generate Tis
 * ---------------------------------------------*/
void getAlignResults::generateTargets()
{
    E1 = 0;
    if(LOG_SAVE_T)
        LOG( " Targets << ", false );
    if(OUTPUT_T_M_INSTANT){
        texturesImgs.clear();
        for( size_t i : kfIndexs )
            texturesImgs[i] = cv::imread(texturesFiles[i]);
    }
    for( size_t i : kfIndexs ) {
        generateTargetI(i, texturesImgs);
        if(LOG_SAVE_T)
            LOG( std::to_string(i) + " ", false);
    }
    if(LOG_SAVE_T)
        LOG( "<< Done << E1: " + std::to_string(E1) );
}
void getAlignResults::generateTargetI(size_t target_id, std::map<size_t, cv::Mat3b> textures)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b target( cv::Size(settings.imgW, settings.imgH) );

    // similarity term
    cv::Mat1i result_ann_s2t( settings.imgH, settings.imgW );
    readAnnTXT( pmResultPath + "/" + getAnnFilename(target_id, "s2t.txt"), result_ann_s2t);
    cv::Mat1i result_ann_t2s( settings.imgH, settings.imgW );
    readAnnTXT( pmResultPath + "/" + getAnnFilename(target_id, "t2s.txt"), result_ann_t2s);
    cv::Mat4i result_su( cv::Size(settings.imgW, settings.imgH) );
    cv::Mat4i result_sv( cv::Size(settings.imgW, settings.imgH) );
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        result_su.at<cv::Vec4i>(j, i) = cv::Vec4i(0,0,0,0);
        result_sv.at<cv::Vec4i>(j, i) = cv::Vec4i(0,0,0,0);
    }
    getSimilarityTerm(sourcesImgs[target_id], result_ann_s2t, result_ann_t2s, result_su, result_sv);

    // calculate E1
    double E1_1 = calcAnndSum( pmResultPath + "/" + getAnndFilename(target_id, "s2t.txt") );
    double E1_2 = calcAnndSum( pmResultPath + "/" + getAnndFilename(target_id, "t2s.txt") );
    E1 += (settings.alpha_u * E1_1 + settings.alpha_v * E1_2);

#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i p_img = cv::Point2i(i, j);

        // if the pixel is in bg, then no optimization
        cv::Vec3i Xij = mappings[target_id][target_id].at<cv::Vec3i>(p_img.y, p_img.x);
        if ( Xij(2) == 0 ) {
            target.at<cv::Vec3b>(j, i) = sourcesImgs[target_id].at<cv::Vec3b>(j, i);
            continue;
        }

        double _factor1, _factor2; cv::Vec3d sum_bgr(0,0,0);

        // similarity term
        cv::Vec3d sum_S(0,0,0);
        _factor1  = settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        _factor1 += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        for( int p_i = 0; p_i < 3; p_i++ ) {
            sum_S(p_i) += settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            sum_S(p_i) += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            sum_bgr(p_i) = sum_S(p_i);
        }

        // consistency term
        cv::Vec3i sum_M(0,0,0); int count_M = 0;
        double weight = static_cast<double>(weights[target_id].at<float>(p_img.y, p_img.x));
        _factor2 = settings.lamda * weight;
        for( size_t t : kfIndexs ) {
            if ( t == target_id ) {
                sum_M += textures[t].at<cv::Vec3b>(j, i);
                count_M += 1;
            } else {
                cv::Vec3i Xij = mappings[target_id][t].at<cv::Vec3i>(p_img.y, p_img.x);
                if ( Xij(2) > 0 ){
                    cv::Point2i p_img_t(cv::Point2i(Xij(0), Xij(1)));
                    sum_M += textures[t].at<cv::Vec3b>(p_img_t.y, p_img_t.x);
                    count_M += 1;
                }
            }
        }
        for ( int p_i = 0; p_i < 3; p_i++ )
          sum_bgr(p_i) += _factor2 * sum_M(p_i) / count_M;

        // generate the pixel of Ti
        cv::Vec3b bgr(0,0,0);
        for ( int p_i = 0; p_i < 3; p_i++ ) {
            sum_bgr(p_i) = sum_bgr(p_i) / (_factor1 + _factor2);
            bgr(p_i) = static_cast<uchar>( EAGLE_MAX(EAGLE_MIN(static_cast<int>(std::round(sum_bgr(p_i))), 255), 0) );
        }
        target.at<cv::Vec3b>(j, i) = bgr;
    }
    if(OUTPUT_T_M_INSTANT) {
        cv::imwrite( targetsFiles[target_id], target );
    } else {
        targetsImgs[target_id] = target;
    }
}

void getAlignResults::getSimilarityTerm(cv::Mat3b S, cv::Mat1i ann_s2t, cv::Mat1i ann_t2s, cv::Mat4i &su, cv::Mat4i &sv)
{
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;

        if( i % settings.patchStep != 0 || j % settings.patchStep != 0 ) // set the patch's step
            continue;

        int x, y, v;
        // Su: completeness
        // here, (i,j) is on Si, and (x,y) on Ti
        if( i >= settings.imgW - (settings.patchWidth-1) || j >= settings.imgH - (settings.patchWidth-1)) {
            calcSuv(S, i, j, su, i, j, 1);
        } else {
            v = ann_s2t.at<int>(j, i);
            x = INT_TO_X(v); y = INT_TO_Y(v);
            calcSuv(S, i, j, su, x, y, settings.patchWidth);
        }
        // Sv: coherence
        // here, (i,j) is on Ti, and (x,y) on Si
        if( i >= settings.imgW - (settings.patchWidth-1) || j >= settings.imgH - (settings.patchWidth-1)) {
            calcSuv(S, i, j, sv, i, j, 1);
        } else {
            v = ann_t2s.at<int>(j, i);
            x = INT_TO_X(v); y = INT_TO_Y(v);
            calcSuv(S, x, y, sv, i, j, settings.patchWidth);
        }
    }
}

void getAlignResults::calcSuv(cv::Mat3b S, int i, int j, cv::Mat4i &s, int x, int y, int w)
{
    for ( int dy = 0; dy < w; dy++ ) {
        for ( int dx = 0; dx < w; dx++ ) {
            if( !pointValid( cv::Point2i(x+dx, y+dy) ) || !pointValid( cv::Point2i(i+dx, j+dy) ) )
                continue;
            s.at<cv::Vec4i>(y + dy, x + dx)(0) += S.at<cv::Vec3b>(j + dy, i + dx)(0);
            s.at<cv::Vec4i>(y + dy, x + dx)(1) += S.at<cv::Vec3b>(j + dy, i + dx)(1);
            s.at<cv::Vec4i>(y + dy, x + dx)(2) += S.at<cv::Vec3b>(j + dy, i + dx)(2);
            s.at<cv::Vec4i>(y + dy, x + dx)(3) += 1;
        }
    }
}

/*----------------------------------------------
 *  Generate Mis
 * ---------------------------------------------*/
void getAlignResults::generateTextures()
{
    E2 = 0;
    if(LOG_SAVE_M)
        LOG( " Textures << ", false );
    if(OUTPUT_T_M_INSTANT) {
        targetsImgs.clear();
        for( size_t i : kfIndexs )
           targetsImgs[i] = cv::imread(targetsFiles[i]);
    }
    for( size_t i : kfIndexs ) {
        generateTextureI(i, targetsImgs);
        if(LOG_SAVE_M)
            LOG( std::to_string(i) + " ", false);
    }
    if(LOG_SAVE_M)
        LOG("<< Done << E2: " + std::to_string(E2));
}
void getAlignResults::generateTextureI(size_t texture_id, std::map<size_t, cv::Mat3b> targets)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b texture( cv::Size(settings.imgW, settings.imgH) );
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i p_img = cv::Point2i(i, j);

        cv::Vec3f sum(0,0,0);
        float weight = 0, sum_w = 0;
        cv::Vec3b pixel; bool flag_valid;

        // for E2 calculation
        std::vector<cv::Vec3b> E2_pixels;
        std::vector<float> E2_weights;

        for ( size_t t : kfIndexs ) {
            flag_valid = true;
            if ( t == texture_id ) {
                weight = weights[t].at<float>(p_img.y, p_img.x);
                pixel = targets[t].at<cv::Vec3b>(j, i);
            } else {
                cv::Vec3i Xij = mappings[texture_id][t].at<cv::Vec3i>(p_img.y, p_img.x);
                if ( Xij(2) > 0 ){
                    cv::Point2i p_img_t = cv::Point2i(Xij(0), Xij(1));
                    weight = weights[t].at<float>(p_img_t.y, p_img_t.x);
                    pixel = targets[t].at<cv::Vec3b>(p_img_t.y, p_img_t.x);
                } else
                    flag_valid = false;
            }
            if(flag_valid == true) {
                sum_w += weight;
                for( int p_i = 0; p_i < 3; p_i++ )
                    sum(p_i) = sum(p_i) + weight * pixel(p_i);

                E2_pixels.push_back(pixel);
                E2_weights.push_back(weight);
            }
        }
        for( int p_i = 0; p_i < 3; p_i++ )
            texture.at<cv::Vec3b>(j, i)(p_i) = static_cast<uchar>( std::round( sum(p_i) / sum_w ) );

        // calculating E2
        if(E2_pixels.size() > 0) {
            double E2_1 = 0;
            for( size_t _i = 0; _i < E2_pixels.size(); _i++ ) {
                double E2_2 = 0;
                for( int _pi = 0; _pi < 3; _pi++ ) {
                    int p1 = E2_pixels[_i](_pi);
                    int p2 = texture.at<cv::Vec3b>(j,i)(_pi);
                    E2_2 += std::pow(p1-p2, 2);
                }
                E2_1 += (E2_2 * E2_weights[_i]);
            }
            E2 += (E2_1 / E2_pixels.size());
        }
    }
    if(OUTPUT_T_M_INSTANT) {
        cv::imwrite( texturesFiles[texture_id], texture );
    } else {
        texturesImgs[texture_id] = texture;
    }
}

/*----------------------------------------------
 *  Generate Mi with S
 * ---------------------------------------------*/
void getAlignResults::generateTextureIWithS(size_t texture_id, std::string fullname)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b texture( cv::Size(settings.imgW, settings.imgH) );
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i p_img = cv::Point2i(i, j);

        cv::Vec3f sum(0,0,0);
        float weight = 0, sum_w = 0;
        cv::Vec3b pixel; bool flag_valid;

        for ( size_t t : kfIndexs ) {
            flag_valid = true;
            if ( t == texture_id ) {
                weight = weights[t].at<float>(p_img.y, p_img.x);
                pixel = sourcesImgs[t].at<cv::Vec3b>(j, i);
            } else {
                cv::Vec3i Xij = mappings[texture_id][t].at<cv::Vec3i>(p_img.y, p_img.x);
                if ( Xij(2) > 0 ){
                    cv::Point2i p_img_t = cv::Point2i(Xij(0), Xij(1));
                    weight = weights[t].at<float>(p_img_t.y, p_img_t.x);
                    pixel = sourcesImgs[t].at<cv::Vec3b>(p_img_t.y, p_img_t.x);
                } else
                    flag_valid = false;
            }
            if(flag_valid == true) {
                sum_w += weight;
                for( int p_i = 0; p_i < 3; p_i++ )
                    sum(p_i) = sum(p_i) + weight * pixel(p_i);
            }
        }
        for( int p_i = 0; p_i < 3; p_i++ )
            texture.at<cv::Vec3b>(j, i)(p_i) = static_cast<uchar>( std::round(sum(p_i) / sum_w) );
    }
    cv::imwrite( fullname, texture );
}

/*----------------------------------------------
 *  Generate OBJ
 * ---------------------------------------------*/
// generate a textured obj file based on the aligned results
//   path = resultsPath + "/" + newResolution
//   filename = "result"
//   resultImgNamePattern = "T_%03d"
void getAlignResults::generateTexturedOBJ(std::string path, std::string filename, std::string resultImgNamePattern)
{
    // store uv coords
    //  start from 1
    std::vector<cv::Point2f> uv_coords;
    uv_coords.push_back( cv::Point2f(0,0) ); // the 0th is empty
    size_t next_empty_uv_index = 1;

    // store each vertex's uv index at every image to avoid duplication
    //  img_index => { vertex_index => uv_index }
    std::map<size_t, std::vector<size_t>> vertex_uv_index;
    for( size_t img_i : kfIndexs )
        vertex_uv_index[img_i] = std::vector<size_t>(point_num, 0); // 0 is the invalid index of uv

    // store each mesh's info under the mtl
    //  img_index => { mesh_index => [ [point_index, uv_index], [point_index, uv_index], [point_index, uv_index] ] }
    std::map<size_t, std::vector<struct face_info>> mesh_info;
    for( size_t img_i : kfIndexs )
        mesh_info[img_i] = std::vector<struct face_info>();

    std::vector<cv::Point2i> v_uv(3);
    for( size_t i = 0; i < mesh_num; i++ ) {
        size_t img_index = kfTotal;
        for( size_t img_i : kfIndexs ) {
            v_uv.clear();
            bool flag = true;
            for(size_t p_i = 0; p_i < 3; p_i++){
                size_t v_index = mesh.polygons[i].vertices[p_i];
                cv::Mat X_w = (cv::Mat_<float>(4, 1) << cloud_rgb.points[v_index].x, cloud_rgb.points[v_index].y, cloud_rgb.points[v_index].z, 1);
                cv::Mat X_img = worldToImg(X_w, img_i);
                cv::Point2i p_img( std::round(X_img.at<float>(0)), std::round(X_img.at<float>(1)) );
                if( !pointProjectionValid(X_img.at<float>(2), img_i, p_img.x, p_img.y) )
                    flag = false;
                else {
                    v_uv[p_i] = p_img;
                }
            }
            if ( flag == true ) {
                img_index = img_i;
                break;
            }
        }
        if ( img_index < kfTotal ) {
            // valid mesh, then find its 3 points' uv-coord's index
            struct face_info info;
            for(size_t p_i = 0; p_i < 3; p_i++) {
                size_t v_index = mesh.polygons[i].vertices[p_i];
                // to get its uv-coord index
                size_t uv_coord_index = 0;
                // if its uv-coord has been put into the uv_coords
                if ( vertex_uv_index[img_index][v_index] > 0 ) {
                    uv_coord_index = vertex_uv_index[img_index][v_index];
                } else { // put into a new uv-coord into the uv_coords
                    float uv_x = (v_uv[p_i].x + 1) * 1.0f / settings.imgW;
                    float uv_y = (v_uv[p_i].y + 1) * 1.0f / settings.imgH;
                    uv_coords.push_back( cv::Point2f(uv_x, uv_y) );
                    // set its index to the vertex_uv_index
                    vertex_uv_index[img_index][v_index] = next_empty_uv_index;
                    uv_coord_index = next_empty_uv_index;
                    // for next uv-coord
                    next_empty_uv_index += 1;
                }
                // the current point's uv-coord is uv_coord_index
                info.v_index[p_i] = v_index; // the point's index
                info.uv_index[p_i] = uv_coord_index; // the uv-coord's index
                info.n_index[p_i] = v_index;
            }
            mesh_info[img_index].push_back( info );
        }
    }
    saveOBJwithMTL(path, filename, resultImgNamePattern, cloud_rgb, uv_coords, mesh_info);
}
void getAlignResults::saveOBJwithMTL(std::string path, std::string filename, std::string resultImgNamePattern,
                                     pcl::PointCloud<pcl::PointXYZRGB> cloud,
                                     std::vector<cv::Point2f> uv_coords,
                                     std::map<size_t, std::vector<struct face_info>> mesh_info)
{
    std::ofstream out;
    char tmp[32]; std::string img_filename;

    // save MTL
    out.open( path + "/" + filename + ".mtl" );
    for ( size_t i : kfIndexs ) {
        sprintf(tmp, resultImgNamePattern.c_str(), i); img_filename = tmp;
        out << "newmtl " << img_filename << std::endl
            << "Ka 1.000000 1.000000 1.000000" << std::endl
            << "Kd 1.000000 1.000000 1.000000" << std::endl
            << "Ks 0.000000 0.000000 0.000000" << std::endl
            << "Tr 1.000000" << std::endl
            << "illum 1" << std::endl
            << "Ns 1.000000" << std::endl
            << "map_Kd " << img_filename + ".jpg" << std::endl << std::endl;
    }
    out.close();

    // save OBJ
    out.open ( path + "/" + filename + ".obj" );
    out << "mtllib " << filename + ".mtl" << std::endl;
    //  output vertices
    for (size_t i = 0; i < cloud.size(); ++i) {
        out << "v " << cloud.points[i].x << " "
            << cloud.points[i].y << " "
            << cloud.points[i].z << std::endl;
    }
    //  output uv-coords // discard the first invalid coord
    for (size_t i = 1; i < uv_coords.size(); ++i) {
        out << "vt " << uv_coords[i].x << " "
            << 1.0f - uv_coords[i].y << std::endl;
    }
    //  output normals
    for (size_t i = 0; i < vertex_normal.size(); ++i) {
        out << "vn " << vertex_normal[i](0) << " "
            << vertex_normal[i](1) << " "
            << vertex_normal[i](2) << std::endl;
    }
    //  output faces
    for ( size_t i : kfIndexs ) {
        if (mesh_info[i].size() == 0)
            continue;
        sprintf(tmp, resultImgNamePattern.c_str(), i); img_filename = tmp;
        out << "usemtl " << img_filename << std::endl;
        for (std::size_t j = 0; j < mesh_info[i].size(); ++j) {
            struct face_info * info = &mesh_info[i][j];
            out << "f";
            for (std::size_t k = 0; k < 3; ++k) {
                out << " " << info->v_index[k] + 1 // start from 1
                    << "/" << info->uv_index[k]
                    << "/" << info->n_index[k] + 1;
            }
            out << std::endl;
        }
    }
    out.close();
}
