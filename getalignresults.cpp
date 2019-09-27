#include "getalignresults.h"

/*----------------------------------------------
 *  Log Settings
 * ---------------------------------------------*/
#define LOG_INIT_N true
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
    LOG("[ From Path: " + settings.keyFramesPath + " ] ");
    LOG("[ Alpha U: " + std::to_string(settings.alpha_u) + " Alpha V: " + std::to_string(settings.alpha_v) + " Lambda: " + std::to_string(settings.lambda) + " ] ");

    // get all keyframe imgs' full path
    sourcesPath = settings.keyFramesPath;
    EAGLE::checkPath(sourcesPath);
    cv::glob(sourcesPath + "/" + settings.kfRGBMatch, sourcesFiles, false);
    // range of all frames
    kfStart = 0; kfTotal = sourcesFiles.size();
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
    // read the model with mesh
    LOG("[ Read PLY Model ] ");
    //pcl::PolygonMesh mesh;
    pcl::io::loadPLYFile(settings.plyWorldFile, mesh);
    // init remappings
    calcVertexMapping();
    LOG("[ Init Success. " + std::to_string(kfIndexs.size()) + " / " + std::to_string(kfTotal) + " Images " + "]");
    doIterations();
    generateColoredPLY(resultsPath);
    LOG( "[ End ]" );
}
getAlignResults::~getAlignResults()
{
    log.close();

    sourcesImgs.clear();
    targetsImgs.clear();
    texturesImgs.clear();

    weights.clear();
    for( size_t t : kfIndexs ) {
        uvs[t].clear();
        mappings[t].clear();
    }
    uvs.clear();
    mappings.clear();
    img_valid_mesh.clear();
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
std::string getAlignResults::getImgFile(size_t img_i)
{
    char buf[18];
    sprintf(buf, (settings.kfRGBNamePattern).c_str(), img_i);
    return sourcesPath + "/" + std::string(buf);
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

// project the point to the (id)th camera's coordinate system
cv::Mat getAlignResults::projectToCamera(cv::Mat X_w, size_t id)
{
    cv::Mat R = cameraPoses[id]; // from camera to world
    cv::Mat RT = R.inv(); //  from world to camera
    return RT * X_w;
}

// project the point to the (id)th image's plane (on origin-resolution)
//   X_w is the point's world position [x_w, y_w, z_w, 1]
//   return 3*1 matrix [x_img, y_img, z_c]
cv::Mat getAlignResults::projectToImg(cv::Mat X_w, size_t id)
{
    cv::Mat X_c = projectToCamera(X_w, id);
    cv::Mat1f X_img = (cv::Mat_<float>(3, 1) << X_c.at<float>(0), X_c.at<float>(1), X_c.at<float>(2));
    X_img = settings.cameraK * X_img / X_c.at<float>(2);
    X_img.at<float>(2) = X_c.at<float>(2);
    return X_img;
}

bool getAlignResults::pointValid(cv::Point2i p_img)
{
    if(p_img.x < 0 || p_img.x >= settings.originImgW)
        return false;
    if(p_img.y < 0 || p_img.y >= settings.originImgH)
        return false;
    return true;
}
bool getAlignResults::pointValid(cv::Point2f p_img)
{
    pointValid( cv::Point2i( std::round(p_img.x), std::round(p_img.y)) );
}

// apply the scale to the point p (p is on the origin-resolution img)
cv::Point2i getAlignResults::imgToScale(cv::Point2i p_img)
{
    cv::Point2i p_img_s(0,0);
    p_img_s.x = std::round( p_img.x / scaleF );
    p_img_s.y = std::round( p_img.y / scaleF );
    return p_img_s;
}

// project the position on the scaled img to the origin-resolution img
cv::Point2i getAlignResults::scaleToImg(cv::Point2i p_img_s)
{
    cv::Point2i p_img(0,0);
    p_img.x = std::round( p_img_s.x * scaleF );
    p_img.y = std::round( p_img_s.y * scaleF );
    return p_img;
}

/*----------------------------------------------
 *  Remapping
 * ---------------------------------------------*/
void getAlignResults::calcVertexMapping()
{
    // read the camera's world positions of keyframes
    LOG("[ Read Camera Matrixs ] ");
    readCameraTraj(settings.kfCameraTxtFile);

    // create a RGB point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // convert to PointCloud
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_rgb);
    size_t point_num = cloud_rgb->points.size();

    // for each vertex, calculate its normal
    std::vector<cv::Vec3f> vertex_normal(point_num); // vertex id => cv::Vec3f
    LOG("[ Calculating Normals of each Vertex ]");
    // for each vertex, calculate its normal
    // ( source: https://blog.csdn.net/wolfcsharp/article/details/93711068 )
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //  using kdtree to search NN
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_rgb);
    n.setInputCloud(cloud_rgb);
    n.setSearchMethod(tree);
    //  set the NN value
    n.setKSearch(20);
    n.compute(*normals);
    // output the normals
#pragma omp parallel for
    for ( size_t i = 0; i < cloud_rgb->points.size(); i++)
        vertex_normal[i] = cv::Vec3f(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);

    // for each vertex, calculate its uv coordinate on each source image
    LOG("[ Calculating UVs of each Vertex on every Si ]");
    //std::map<size_t, std::vector<cv::Point3f>> uvs;
    for( size_t t : kfIndexs )
        uvs[t] = std::vector<cv::Point3f>(point_num); // vertex id => cv::Point3f
#pragma omp parallel for
    for (size_t i = 0; i < point_num; i++) {
        cv::Mat X_w = (cv::Mat_<float>(4, 1) << cloud_rgb->points[i].x, cloud_rgb->points[i].y, cloud_rgb->points[i].z, 1);
        for( size_t t : kfIndexs ) {
            // get the vertex's position on S(t) at origin resolution
            cv::Mat X_img = projectToImg(X_w, t);
            uvs[t][i] = cv::Point3f(X_img.at<float>(0), X_img.at<float>(1), X_img.at<float>(2));
        }
    }

    // for each vertex, calculate its weight of each image
    LOG("[ Calculating Weight of each Vertex ]");
    std::map<size_t, std::vector<float>> vertex_weight;
    for( size_t t : kfIndexs )
        vertex_weight[t] = std::vector<float>(point_num);
#pragma omp parallel for
    for(size_t i = 0; i < point_num; i++) {
        cv::Mat X_w = (cv::Mat_<float>(4, 1) << cloud_rgb->points[i].x, cloud_rgb->points[i].y, cloud_rgb->points[i].z, 1);
        for( size_t t : kfIndexs ) {
            cv::Mat X_c = projectToCamera(X_w, t);
            float d2 = X_c.at<float>(0) * X_c.at<float>(0) + X_c.at<float>(1) * X_c.at<float>(1) + X_c.at<float>(2) * X_c.at<float>(2);
            vertex_weight[t][i] = vertex_normal[i](2) * vertex_normal[i](2) / d2;
        }
    }

    // interpolate vertex's weight to every pixel
    LOG("[ Calculating weight of every pixel on each Si ]");
    //std::map<size_t, cv::Mat> weights;
    weights.clear();
    LOG( " Weights << ", false );
    for( size_t t : kfIndexs ) {
        weights[t] = cv::Mat1f( cv::Size(settings.originImgW, settings.originImgH) );
        for( int i = 0; i < settings.originImgW; i++ )
            for( int j = 0; j < settings.originImgH; j++ )
                weights[t].at<float>(j, i) = 0;
        calcImgWeight(t, vertex_weight[t]);
        LOG( std::to_string(t) + " ", false );
    }
    LOG( "<< Done" );

    // calculate valid mesh for each pixel on every image
    LOG("[ Calculating valid mesh of every pixel on each Si ]");
    //std::map<size_t, cv::Mat> img_valid_mesh;
    img_valid_mesh.clear();
    LOG( " Valid Mesh << ", false );
    for( size_t t : kfIndexs ) {
        img_valid_mesh[t] = cv::Mat2i( cv::Size(settings.originImgW, settings.originImgH) );
        for( int i = 0; i < settings.originImgW; i++ )
            for( int j = 0; j < settings.originImgH; j++ )
                img_valid_mesh[t].at<cv::Vec2i>(j, i) = cv::Vec2i(-1, 0);
        calcImgValidMesh(t);
        LOG( std::to_string(t) + " ", false );
    }
    LOG( "<< Done" );

    // do remapping
    //std::map<size_t, std::map<size_t, cv::Mat>> mappings;
    mappings.clear();
    // for every triangle mesh, do projection from i to j
    LOG("[ Image Remapping ]");
    for( size_t img_i : kfIndexs) {
        mappings[img_i] = std::map<size_t, cv::Mat>();
        LOG( " " + std::to_string(img_i) + " to ", false );
        for( size_t img_j : kfIndexs ) {
            if( img_i != img_j ) {
                mappings[img_i][img_j] = cv::Mat3i( cv::Size(settings.originImgW, settings.originImgH) );
                for( int i = 0; i < settings.originImgW; i++ )
                    for( int j = 0; j < settings.originImgH; j++ )
                        mappings[img_i][img_j].at<cv::Vec3i>(j, i) = cv::Vec3i(0,0,0);
                calcRemapping(img_i, img_j);
            }
            LOG( std::to_string(img_j) + " ", false );
        }
        LOG( "<< Done" );
    }
}

void getAlignResults::calcImgWeight(size_t img_i, std::vector<float> vertex_weight)
{
#pragma omp parallel for
    for( size_t i = 0; i < mesh.polygons.size(); i++ ) {
        size_t p1 = mesh.polygons[i].vertices[0];
        size_t p2 = mesh.polygons[i].vertices[1];
        size_t p3 = mesh.polygons[i].vertices[2];

        cv::Point3f i_uv1 = uvs[img_i][p1];
        cv::Point3f i_uv2 = uvs[img_i][p2];
        cv::Point3f i_uv3 = uvs[img_i][p3];

        float w1 = vertex_weight[p1];
        float w2 = vertex_weight[p2];
        float w3 = vertex_weight[p3];

        cv::Rect pos(0,0,0,0);
        cv::Mat3f lamdas = calcPosCoord(i_uv1, i_uv2, i_uv3, pos);
        int total = pos.width * pos.height;
        int dx, dy; float w;
        for ( int index = 0; index < total; index++ ) {
            dy = index / pos.width;
            dx = index % pos.width;
            cv::Point2i p_img( pos.x + dx, pos.y + dy );
            if( ! pointValid(p_img) )
                continue;
            cv::Vec3f lamda = lamdas.at<cv::Vec3f>(dy, dx);
            if( lamda(0) >= 0 && lamda(1) >= 0 && lamda(2) >= 0 ) {
                w = w1*lamda(0) + w2*lamda(1) + w3*lamda(2);
                weights[img_i].at<float>(p_img.y, p_img.x) = w;
            }
        }
    }
}

// Barycentric coordinate system
//   https://blog.csdn.net/silangquan/article/details/21990713
//   https://www.zhihu.com/question/38356223
cv::Mat3f getAlignResults::calcPosCoord(cv::Point3f uv1, cv::Point3f uv2, cv::Point3f uv3, cv::Rect &pos)
{
    float x1 = uv1.x, y1 = uv1.y, x2 = uv2.x, y2 = uv2.y, x3 = uv3.x, y3 = uv3.y;
    int max_x = std::ceil( EAGLE_MAX(EAGLE_MAX(x1,x2),x3) );
    int min_x = std::floor( EAGLE_MIN(EAGLE_MIN(x1,x2),x3) );
    int max_y = std::ceil( EAGLE_MAX(EAGLE_MAX(y1,y2),y3) );
    int min_y = std::floor( EAGLE_MIN(EAGLE_MIN(y1,y2),y3) );

    //cv::Rect pos(min_x, min_y, max_x-min_x+1, max_y-min_y+1);
    pos.x = min_x;
    pos.y = min_y;
    pos.width = max_x-min_x+1;
    pos.height = max_y-min_y+1;

    float detT = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);

    cv::Mat3f lamdas( cv::Size(pos.width, pos.height) );
    int total = pos.width * pos.height;
    for ( int index = 0; index < total; index++ ) {
        int dy = index / pos.width;
        int dx = index % pos.width;
        int x = pos.x + dx, y = pos.y + dy;
        lamdas.at<cv::Vec3f>(dy, dx)(0) = ((y2-y3)*(x-x3) + (x3-x2)*(y-y3)) / detT;
        lamdas.at<cv::Vec3f>(dy, dx)(1) = ((y3-y1)*(x-x3) + (x1-x3)*(y-y3)) / detT;
        lamdas.at<cv::Vec3f>(dy, dx)(2) = 1 - lamdas.at<cv::Vec3f>(dy, dx)(0) - lamdas.at<cv::Vec3f>(dy, dx)(1);
    }
    return lamdas;
}

// for each pixel, find the mesh from which it gets the color
void getAlignResults::calcImgValidMesh(size_t img_i)
{
    cv::Mat1i img_valid_z = cv::Mat1i ( cv::Size(settings.originImgW, settings.originImgH) );
    for( int i = 0; i < settings.originImgW; i++ )
        for( int j = 0; j < settings.originImgH; j++ )
            img_valid_z.at<int>(j, i) = -1;
    for( size_t i = 0; i < mesh.polygons.size(); i++ ) {
        size_t p1 = mesh.polygons[i].vertices[0];
        size_t p2 = mesh.polygons[i].vertices[1];
        size_t p3 = mesh.polygons[i].vertices[2];
        cv::Point3f i_uv1 = uvs[img_i][p1];
        cv::Point3f i_uv2 = uvs[img_i][p2];
        cv::Point3f i_uv3 = uvs[img_i][p3];

        cv::Rect pos(0,0,0,0);
        cv::Mat3f lamdas = calcPosCoord(i_uv1, i_uv2, i_uv3, pos);
        int total = pos.width * pos.height;
        for ( int index = 0; index < total; index++ ) {
            int dy = index / pos.width;
            int dx = index % pos.width;
            cv::Point2i p_img_i( pos.x + dx, pos.y + dy );
            if ( ! pointValid(p_img_i) )
                continue;
            cv::Vec3f lamda = lamdas.at<cv::Vec3f>(dy, dx);
            if ( lamda(0) >= 0 && lamda(1) >= 0 && lamda(2) >= 0 ) {
                int z_new = std::round( 10000 / (lamda(0)/i_uv1.z + lamda(1)/i_uv2.z + lamda(2)/i_uv3.z) );
//                int z_new = std::round( 10000 * (lamda(0)*i_uv1.z + lamda(1)*i_uv2.z + lamda(2)*i_uv3.z) );
                int z_old = img_valid_z.at<int>(p_img_i.y, p_img_i.x);
                if ( z_old < 0 || z_new <= z_old ) {
                    img_valid_z.at<int>(p_img_i.y, p_img_i.x) = z_new;
                    img_valid_mesh[img_i].at<cv::Vec2i>(p_img_i.y, p_img_i.x)(0) = i;
                    img_valid_mesh[img_i].at<cv::Vec2i>(p_img_i.y, p_img_i.x)(1) = index;
                }
            }
        }
    }
    //std::cout << img_valid_z << std::endl;
}

// for each pixel in img_i, remapping it to img_j only when the mesh is visible both in i and j
void getAlignResults::calcRemapping(size_t img_i, size_t img_j)
{
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int img_index = 0; img_index < total; img_index++) {
        int y = img_index / settings.imgW;
        int x = img_index % settings.imgW;
        int i = img_valid_mesh[img_i].at<cv::Vec2i>(y, x)(0);
        if( i < 0 )
            continue;

        size_t p1 = mesh.polygons[i].vertices[0];
        size_t p2 = mesh.polygons[i].vertices[1];
        size_t p3 = mesh.polygons[i].vertices[2];
        cv::Rect pos(0,0,0,0);
        cv::Mat3f lamdas = calcPosCoord( uvs[img_i][p1], uvs[img_i][p2], uvs[img_i][p3], pos);
        int index = img_valid_mesh[img_i].at<cv::Vec2i>(y, x)(1);
        cv::Vec3f lamda = lamdas.at<cv::Vec3f>( index / pos.width, index % pos.width );

        cv::Point3f j_uv1 = uvs[img_j][p1];
        cv::Point3f j_uv2 = uvs[img_j][p2];
        cv::Point3f j_uv3 = uvs[img_j][p3];
        cv::Point2i p_img_j;
        p_img_j.x = std::round( j_uv1.x * lamda(0) + j_uv2.x * lamda(1) + j_uv3.x * lamda(2) );
        p_img_j.y = std::round( j_uv1.y * lamda(0) + j_uv2.y * lamda(1) + j_uv3.y * lamda(2) );
        if ( ! pointValid( p_img_j ) )
            continue;
        if ( img_valid_mesh[img_j].at<cv::Vec2i>(p_img_j.y, p_img_j.x)(0) == i ) {
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(0) = p_img_j.x;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(1) = p_img_j.y;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) = 1;
        }
    }
}

/*----------------------------------------------
 *  Do Iterations
 * ---------------------------------------------*/
void getAlignResults::doIterations()
{
//    int scale = settings.scaleTimes-1;
    int scale = 0;
    std::vector<cv::String> originSourcesFiles(sourcesFiles); // origin sources
    for ( ; scale < settings.scaleTimes; scale++) {
        // downsample imgs
        settings.imgW = std::round(settings.scaleInitW * pow(settings.scaleFactor, scale));
        settings.imgH = std::round(settings.scaleInitH * pow(settings.scaleFactor, scale));
        scaleF = pow(settings.scaleFactor, settings.scaleTimes-1-scale);

        char tmp[10];
        sprintf(tmp, "%dx%d", settings.imgW, settings.imgH);
        std::string newResolution(tmp);
        LOG("[ Scale to " + newResolution + " ]");

        // get all keyframes' full path (after scale)
        sourcesPath = settings.keyFramesPath + "/" + newResolution;
        EAGLE::checkPath(sourcesPath);
        // generate source imgs with new resolution // [REQUIRE] ImageMagick
        for( size_t i = 0; i < kfTotal; i++ )
            system( ("convert " + originSourcesFiles[i] + " -resize " + newResolution + "! " + getImgFile(i)).c_str() );
        cv::glob(sourcesPath + "/" + settings.kfRGBMatch, sourcesFiles, false);
        // read Si
        sourcesImgs.clear();
        for ( size_t i = 0; i < kfTotal; i++ )
            sourcesImgs.push_back( cv::imread(sourcesFiles[i]) ); // img.at<cv::Vec3b>(y, x)(0)

        // init Ti and Mi or upsample
        if ( targetsFiles.size() == 0 ) {
            for( size_t i = 0; i < kfTotal; i++ ) {
                system( ("cp " + sourcesFiles[i] + " " + targetsPath+"/").c_str() );
                system( ("cp " + sourcesFiles[i] + " " + texturesPath+"/").c_str() );
            }
            cv::glob(targetsPath + "/" + settings.kfRGBMatch, targetsFiles, false);
            cv::glob(texturesPath + "/" + settings.kfRGBMatch, texturesFiles, false);
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
            for( size_t i = 0; i < kfTotal; i++ ) {
                targetsImgs.push_back( cv::imread(targetsFiles[i]) );
                texturesImgs.push_back( cv::imread(texturesFiles[i]) );
            }
        }
        // do iterations
        int iter_count = 100 - scale * 5;
        for ( int _count = 0; _count < iter_count; _count++) {
            LOG("[ Iteration " + std::to_string(_count+1) + " at " + newResolution + " ]");
            calcPatchmatch();
            generateTargets();
            generateTextures();
        }
        if( ! OUTPUT_T_M_INSTANT) {
            for( size_t i : kfIndexs ){
                cv::imwrite( targetsFiles[i], targetsImgs[i] );
                cv::imwrite( texturesFiles[i], texturesImgs[i] );
            }
        }

        // save results
        std::string texturesResultPath = resultsPath + "/" + newResolution;
        EAGLE::checkPath(texturesResultPath);
        for( size_t i : kfIndexs ){
            system( ("cp " + texturesFiles[i] + " " + texturesResultPath+"/" + getImgFilename(i, "M_", "."+settings.rgbNameExt)).c_str() );
            system( ("cp " + targetsFiles[i] + " " + texturesResultPath+"/" + getImgFilename(i, "T_", "."+settings.rgbNameExt)).c_str() );
        }
        LOG( "[ Results at " + newResolution + " Saving Success ]" );
    }
}

/*----------------------------------------------
 *  PatchMatch
 * ---------------------------------------------*/
void getAlignResults::calcPatchmatch()
{
    if(LOG_PM)
        LOG( " Patchmatchs << ", false );
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

        if(LOG_PM)
            LOG( std::to_string(*i) + " ", false);
    }
    if(LOG_PM)
        LOG( "<< Done" );
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
    if(OUTPUT_T_M_INSTANT){
        texturesImgs.clear();
        for( size_t i = 0; i < kfTotal; i++ )
            texturesImgs.push_back( cv::imread(texturesFiles[i]) );
    }
    for( size_t i : kfIndexs ) {
        generateTargetI(i, texturesImgs);
        if(LOG_SAVE_T)
            LOG( std::to_string(i) + " ", false);
    }
    if(LOG_SAVE_T)
        LOG( "<< Done" );
}
void getAlignResults::generateTargetI(size_t target_id, std::vector<cv::Mat3b> textures)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b target( cv::Size(settings.imgW, settings.imgH) );

    // similarity term
    std::string sourceFile = getImgFile( target_id );
    std::string ann_txt = pmResultPath + "/" + getAnnFilename(sourceFile, "s2t.txt");
    cv::Mat1i result_ann_s2t( settings.imgH, settings.imgW );
    readAnnTXT(ann_txt, result_ann_s2t);
    ann_txt = pmResultPath + "/" + getAnnFilename(sourceFile, "t2s.txt");
    cv::Mat1i result_ann_t2s( settings.imgH, settings.imgW );
    readAnnTXT(ann_txt, result_ann_t2s);

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

#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        cv::Point2i p_img = scaleToImg( cv::Point2i(i, j) );
        double weightJ = weights[target_id].at<float>(p_img.y, p_img.x);

        // consistency term
        cv::Vec3i sum_M(0,0,0); int count_M = 0;
        for( size_t t : kfIndexs ) {
            if ( t == target_id ) {
                sum_M += textures[t].at<cv::Vec3b>(j, i);
                count_M += 1;
            } else {
                cv::Vec3i Xij = mappings[target_id][t].at<cv::Vec3i>(p_img.y, p_img.x);
                if ( Xij(2) > 0 ){
                    cv::Point2i p_img_t(cv::Point2i(Xij(0), Xij(1)));
                    cv::Point2i p_img_ts = imgToScale(p_img_t);
                    sum_M += textures[t].at<cv::Vec3b>(p_img_ts.y, p_img_ts.x);
                    count_M += 1;
                }
            }
        }
        // generate the pixel of Ti
        cv::Vec3b bgr(0,0,0);
        double _factor = settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        _factor += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        _factor += settings.lambda * weightJ;
        for ( int p_i = 0; p_i < 3; p_i++ ) {
            double _tmp = settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            _tmp += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            _tmp += settings.lambda * weightJ * sum_M(p_i) / count_M;
            bgr(p_i) = std::round(_tmp / _factor);
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
        int x, y;
        // here, (i,j) is on Si, and (x,y) on Ti
        // calculating the completeness term
        int v = ann_s2t.at<int>(j, i);
        if( v == 0 ) {
            calcSuv(S, i, j, su, i, j, 1);
        } else {
            x = INT_TO_X(v); y = INT_TO_Y(v);
            calcSuv(S, i, j, su, x, y, settings.patchWidth);
        }
        // here, (i,j) is on Ti, and (x,y) on Si
        // calculating the coherence term
        v = ann_t2s.at<int>(j, i);
        if( v == 0 ) {
            calcSuv(S, i, j, sv, i, j, 1);
        } else {
            x = INT_TO_X(v); y = INT_TO_Y(v);
            calcSuv(S, x, y, sv, i, j, settings.patchWidth);
        }
    }
}

void getAlignResults::calcSuv(cv::Mat3b S, int i, int j, cv::Mat4i &s, int x, int y, int w)
{
    for ( int dy = 0; dy < w; dy++ ) {
        for ( int dx = 0; dx < w; dx++ ) {
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
    if(LOG_SAVE_M)
        LOG( " Textures << ", false );
    if(OUTPUT_T_M_INSTANT) {
        targetsImgs.clear();
        for( size_t i = 0; i < kfTotal; i++ )
           targetsImgs.push_back( cv::imread(targetsFiles[i]) );
    }
    for( size_t i : kfIndexs ) {
        generateTextureI(i, targetsImgs);
        if(LOG_SAVE_M)
            LOG( std::to_string(i) + " ", false);
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

        cv::Vec3b pixel; bool flag_valid;
        float weight = 0, sum_r = 0, sum_g = 0, sum_b = 0, sum_w = 0;
        cv::Point2i p_img = scaleToImg( cv::Point2i(i, j) );
        for ( size_t t : kfIndexs ) {
            flag_valid = true;
            if ( t == texture_id ) {
//                flag_valid = false;
                weight = weights[t].at<float>(p_img.y, p_img.x);
                pixel = targets[t].at<cv::Vec3b>(j, i);
            } else {
                cv::Vec3i Xij = mappings[texture_id][t].at<cv::Vec3i>(p_img.y, p_img.x);
                if ( Xij(2) > 0 ){
                    cv::Point2i p_img_t = cv::Point2i(Xij(0), Xij(1));
                    weight = weights[t].at<float>(p_img_t.y, p_img_t.x);
                    cv::Point2i p_img_ts = imgToScale(p_img_t);
                    pixel = targets[t].at<cv::Vec3b>(p_img_ts.y, p_img_ts.x);
                } else
                    flag_valid = false;
            }
            if(flag_valid == true) {
                sum_w += weight;
                sum_b = sum_b + weight * pixel(0);
                sum_g = sum_g + weight * pixel(1);
                sum_r = sum_r + weight * pixel(2);
            }
        }
        texture.at<cv::Vec3b>(j, i)(0) = std::round( sum_b / sum_w );
        texture.at<cv::Vec3b>(j, i)(1) = std::round( sum_g / sum_w );
        texture.at<cv::Vec3b>(j, i)(2) = std::round( sum_r / sum_w );
    }
    if(OUTPUT_T_M_INSTANT) {
        cv::imwrite( texturesFiles[texture_id], texture );
    } else {
        texturesImgs[texture_id] = texture;
    }
}

void getAlignResults::generateColoredPLY(std::string path)
{
    cv::glob(targetsPath + "/" + settings.kfRGBMatch, targetsFiles, false);
    targetsImgs.clear();
    for( size_t i = 0; i < kfTotal; i++ )
       targetsImgs.push_back( cv::imread(targetsFiles[i]) );

    // create a RGB point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // convert to PointCloud
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_rgb);

    for( size_t i = 0; i < mesh.polygons.size(); i++ ) {
        for ( size_t pi = 0; pi < 3; pi++ ){
            size_t p = mesh.polygons[i].vertices[pi];
            cv::Vec3i pixel_sum(0,0,0);
            int count = 0;
            for ( size_t t : kfIndexs ) {
                cv::Point3f pi_uv = uvs[t][p];
                cv::Point2i p_img(std::round(pi_uv.x), std::round(pi_uv.y));
                if ( pointValid(p_img) && img_valid_mesh[t].at<cv::Vec2i>(p_img.y, p_img.x)(0) == i ) {
                    cv::Vec3b pixel = targetsImgs[t].at<cv::Vec3b>(p_img.y, p_img.x);
                    pixel_sum += pixel;
                    count += 1;
                }
            }
            if ( count > 0 ) {
                cloud_rgb->points[p].b = pixel_sum(0) * 1.0 / count;
                cloud_rgb->points[p].g = pixel_sum(1) * 1.0 / count;
                cloud_rgb->points[p].r = pixel_sum(2) * 1.0 / count;
            }
        }
    }

    // convert to mesh
    pcl::toPCLPointCloud2(*cloud_rgb, mesh.cloud);
    pcl::io::savePLYFile( path + "/" + settings.plyTColorFilename, mesh );
}

/*----------------------------------------------
 *  Generate UV Map for the Mesh
 * ---------------------------------------------*/
//void getAlignResults::genereteUVMaps()
//{}
