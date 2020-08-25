#include "getalignresults.h"

/*----------------------------------------------
 *  Log Settings
 * ---------------------------------------------*/
#define OUTPUT_T_M_INSTANT false

/*----------------------------------------------
 *  Math
 * ---------------------------------------------*/
#define EPS 1e-10
#define EAGLE_MAX(x,y) (x > y ? x : y)
#define EAGLE_MIN(x,y) (x < y ? x : y)
#define EAGLE_EQU_F(a,b) (fabs(a-b) <= 1e-6)

/*----------------------------------------------
 *  Main
 * ---------------------------------------------*/
getAlignResults::getAlignResults(Settings &_settings)
{
    settings = _settings;
    cv::glob(settings.keyFramesPath + "/" + settings.kfRGBMatch, sourcesOrigin, false);
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
    // make the dir to store all files
    processPath = settings.keyFramesPath + "/results_Bi17" + settings.resultsPathSurfix;
    EAGLE::checkPath(processPath);
    // make the dir to store processing images
    sourcesPath = processPath + "/sources";
    EAGLE::checkPath(sourcesPath);
    targetsPath = processPath + "/targets";
    EAGLE::checkPath(targetsPath);
    texturesPath = processPath + "/textures";
    EAGLE::checkPath(texturesPath);
    weightsPath = processPath + "/weights";
    EAGLE::checkPath(weightsPath);
    // make the dir to store iteration results
    resultsPath = processPath + "/results";
    EAGLE::checkPath(resultsPath);
    log.open( resultsPath + "/LOG.log" );
    LOG("[ From Path: " + settings.keyFramesPath + " ]");
    LOG("[ To Path: ./results_Bi17" + settings.resultsPathSurfix + " ]" );
    LOG("[ Alpha U: " + std::to_string(settings.alpha_u) + " | Alpha V: " + std::to_string(settings.alpha_v) + " ] ");
    LOG("[ Patch Width: " + std::to_string(settings.patchWidth) +
        " | Patch Step: " + std::to_string(settings.patchStep) +
        " | Patch Random Search: " + std::to_string(settings.patchRandomSearchTimes) + " ]");
    LOG("[ Scale: " + std::to_string(settings.scaleTimes) +
        " | From " + std::to_string(settings.scaleInitW) + "x" + std::to_string(settings.scaleInitH) +
        " to " + std::to_string(settings.originImgW) + "x" + std::to_string(settings.originImgH) + " ]");

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

    readDepthImgs();

    // read the camera's world positions of keyframes
    LOG("[ Read Camera Matrixs ] ");
    if ( EAGLE::isFileExist(settings.keyFramesPath + "/" + settings.kfCameraTxtFile) )
        readCameraTraj(settings.keyFramesPath + "/" + settings.kfCameraTxtFile);
    else
        readCameraTraj();

    LOG("[ Init Success. " + std::to_string(kfIndexs.size()) + " / " + std::to_string(kfTotal) + " Images " + "]");

    time_t start, end;
    struct timeval tv;
    char time_start_str[32], time_end_str[32];

    gettimeofday(&tv, nullptr);
    auto time_tmp = localtime(&tv.tv_sec);
    strftime(time_start_str, 32, "%Y.%m.%d %H:%M:%S", time_tmp);
    time(&start);

    doIterations();

    gettimeofday(&tv, nullptr);
    time_tmp = localtime(&tv.tv_sec);
    strftime(time_end_str, 32, "%Y.%m.%d %H:%M:%S", time_tmp);
    time(&end);

    double all_seconds = difftime(end, start);
    LOG("[ Running from " + std::string(time_start_str) + " to " + std::string(time_end_str) + " ]");
    LOG("[ Finish in " + std::to_string(all_seconds) + " s ]");
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
 *  Depth File
 * ---------------------------------------------*/
void getAlignResults::readDepthImgs()
{
    char tmp[24];
    for ( size_t i : kfIndexs ) {
        sprintf(tmp, (settings.kfDNamePattern).c_str(), i);
        std::string file = settings.keyFramesPath + "/" + std::string(tmp);
        if (EAGLE::isFileExist(file))
            depthImgs[i] = cv::imread(file, cv::IMREAD_UNCHANGED);
    }
}
float getAlignResults::getDepthRaw(size_t img_i, int x, int y)
{
    if ( depthImgs.empty() == true ) {
        return -1.0f;
    }
    float v = 0;
    switch(settings.depthType) {
    case 'f' :
        v = depthImgs[img_i].at<float>(y, x);
        break;
    case 'i' : {
        int t_i = depthImgs[img_i].at<int>(y, x);
        v = static_cast<float>(t_i) * 1.0f / 1000;
        break;
    }
    case 's' : {
        unsigned short t_u = depthImgs[img_i].at<ushort>(y, x);
        v = static_cast<float>(t_u) * 1.0f / 1000;
        break;
    }
    case 'b' : {
        unsigned char t_u = depthImgs[img_i].at<uchar>(y, x);
        v = static_cast<float>(t_u) * 1.0f / 1000;
        break;
    }
    }
    //std::cout << "[(" << x << "," << y << ") " << v << "]" << std::flush;
    return v;
}
float getAlignResults::getDepth(size_t img_i, int x, int y)
{
    if ( depthImgs.empty() == true ) {
        return -1.0f;
    }
    x = static_cast<int>(round(1.0 * x / settings.imgW * settings.originDepthW));
    y = static_cast<int>(round(1.0 * y / settings.imgH * settings.originDepthH));
    return getDepthRaw(img_i, x, y);
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

/*----------------------------------------------
 *  Valid Check
 * ---------------------------------------------*/
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
    // depth image's depth
//    float depth = getDepth(img_id, x, y);
//    if ( depth > 0 && ( (depth - point_z) > 0.01f || (point_z - depth) > 0.01f) )
//        return false;
    // get the position's valid info
    size_t p_index = static_cast<size_t>(x + y * settings.imgW);
    struct valid_info * info = &img_valid_info[img_id][p_index];
    // check the depth (whether the point is occluded)
    if ( point_z > info->depth + 0.01f )
        return false;
    // check the angle, if the angle is too small, then it's a bad projection
    if ( info->cos_alpha > -0.5f )
        return false;
    // check the weight (if the weight is too small, then it's a bad projection)
    if ( weights[img_id].at<float>(y, x) < 0.01f )
        return false;
    // the point can be projected to the position on img
    return true;
}

/*----------------------------------------------
 *  Pre-Process
 * ---------------------------------------------*/
void getAlignResults::calcNormals()
{
    LOG("[ Calculating Normals of each Vertex ]");
    //std::vector<cv::Vec3f> vertex_normal(point_num); // vertex id => cv::Vec3f
    vertex_normal.resize(point_num);
    // store each vertex's total weight angle
    std::vector<float> vertex_angle(point_num);

    for( size_t i = 0; i < mesh_num; i++ ) {
        std::vector<cv::Vec3f> v(3);
        for( size_t v_i = 0; v_i < 3; v_i++ ) {
            size_t p_i = mesh.polygons[i].vertices[v_i];
            cv::Vec3f v_(cloud_rgb.points[p_i].x, cloud_rgb.points[p_i].y, cloud_rgb.points[p_i].z);
            v[v_i] = v_; // store the current mesh's points coords
        }
        cv::Vec3f e1 = v[1] - v[0];
        cv::Vec3f e2 = v[2] - v[1];
        cv::Vec3f fn = cv::normalize(e1.cross(e2)); // current mesh's normal

        // calc the triangle mesh's each vertex's angle
        double cos_t0 = (v[1] - v[0]).dot(v[2] - v[0]) / cv::norm(v[1] - v[0]) / cv::norm(v[2] - v[0]) * CV_PI / 180.0;
        double cos_t1 = (v[0] - v[1]).dot(v[2] - v[1]) / cv::norm(v[0] - v[1]) / cv::norm(v[2] - v[1]) * CV_PI / 180.0;
        double cos_t2 = (v[0] - v[2]).dot(v[1] - v[2]) / cv::norm(v[0] - v[2]) / cv::norm(v[1] - v[2]) * CV_PI / 180.0;
        double t[3] = {0, 0, 0};
        t[0] = acos(cos_t0); t[1] = acos(cos_t1); t[2] = acos(cos_t2);

        for( size_t v_i = 0; v_i < 3; v_i++ ) {
            size_t p_i = mesh.polygons[i].vertices[v_i];
            vertex_normal[p_i] += fn * t[v_i];
            vertex_angle[p_i] += static_cast<float>(t[v_i]);
        }
    }
    // normalize normals by angle weights
#pragma omp parallel for
    for( size_t i = 0; i < point_num; i++ )
        vertex_normal[i] /= vertex_angle[i];
}

// calculate valid mesh for each pixel on every image
void getAlignResults::calcValidMesh()
{
    LOG("[ Calculating Depth, Distance and Weight ]");

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
        LOG( " " + std::to_string(t) + " << ", false );
        calcImgValidMesh(t, bvhtree);
        LOG( " ", true );
    }
}
// using the ray intersection method to get the pixel's depth
void getAlignResults::calcImgValidMesh(size_t img_i, BVHTree &bvhtree)
{
    // calc the camera's position (in world coord)
    cv::Mat cam_c = (cv::Mat_<float>(4, 1) << 0, 0, 0, 1);
    cv::Mat cam_w = cameraToWorld(cam_c, img_i);
    math::Vec3f cam_world_p(cam_w.at<float>(0), cam_w.at<float>(1), cam_w.at<float>(2));

    // calc the camera's direction vector (in world coord)
    cv::Mat cam_v_c = (cv::Mat_<float>(4, 1) << 0, 0, 1, 0);
    cv::Mat cam_v_w = cameraToWorld(cam_v_c, img_i);
    math::Vec3f cam_world_v(cam_v_w.at<float>(0), cam_v_w.at<float>(1), cam_v_w.at<float>(2));
    cam_world_v = cam_world_v.normalized();

    float depth_min = FLT_MAX;
    float depth_max = 0.0;
    float d2_min = FLT_MAX;
    float d2_max = 0.0;
    float weight_min = FLT_MAX;
    float weight_max = 0.0;
    cv::Mat1f depth_f(settings.imgH, settings.imgW, 0.0f);

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

            float depth = cam_world_v.dot(hit.t * ray.dir);
            info->depth = depth;
            depth_f.at<float>(y,x) = depth;
            if ( depth < depth_min )
                depth_min = depth;
            if ( depth > depth_max )
                depth_max = depth;

            math::Vec3f const & w = hit.bcoords; // cv::Vec3f( w(0), w(1), w(2) );
            size_t v1_id = mesh.polygons[info->mesh_id].vertices[0];
            size_t v2_id = mesh.polygons[info->mesh_id].vertices[1];
            size_t v3_id = mesh.polygons[info->mesh_id].vertices[2];

            // calc world position
            float _x = cloud_rgb.points[v1_id].x * w(0) + cloud_rgb.points[v2_id].x * w(1) + cloud_rgb.points[v3_id].x * w(2);
            float _y = cloud_rgb.points[v1_id].y * w(0) + cloud_rgb.points[v2_id].y * w(1) + cloud_rgb.points[v3_id].y * w(2);
            float _z = cloud_rgb.points[v1_id].z * w(0) + cloud_rgb.points[v2_id].z * w(1) + cloud_rgb.points[v3_id].z * w(2);
            float d2 = (cam_world_p(0)-_x)*(cam_world_p(0)-_x) + (cam_world_p(1)-_y)*(cam_world_p(1)-_y) + (cam_world_p(2)-_z)*(cam_world_p(2)-_z);
            if ( d2 < d2_min )
                d2_min = d2;
            if ( d2 > d2_max )
                d2_max = d2;

            // calc normal
            cv::Vec3f n1 = vertex_normal[ v1_id ];
            cv::Vec3f n2 = vertex_normal[ v2_id ];
            cv::Vec3f n3 = vertex_normal[ v3_id ];
            math::Vec3f normal;
            normal(0) = n1(0) * w(0) + n2(0) * w(1) + n3(0) * w(2);
            normal(1) = n1(1) * w(0) + n2(1) * w(1) + n3(1) * w(2);
            normal(2) = n1(2) * w(0) + n2(2) * w(1) + n3(2) * w(2);
            normal = normal.normalize();
            math::Vec3f vert2view = -ray.dir;
            float cos_alpha = -vert2view.dot(normal); // the cos of angle between camera dir and vertex normal
            info->cos_alpha = cos_alpha;

            // calc weight
            float weight = cos_alpha * cos_alpha / d2;
            weights[img_i].at<float>(y, x) = weight;
            if ( weight > weight_max )
                weight_max = weight;
            if ( weight < weight_min && weight > 0 )
                weight_min = weight;
        }
    }
    LOG("depth: " + std::to_string(sqrt(depth_min)) + " to " + std::to_string(sqrt(depth_max)) + " | ", false );
    LOG("d: " + std::to_string(sqrt(d2_min)) + " to " + std::to_string(sqrt(d2_max)), false );

#pragma omp parallel for
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;
        weights[img_i].at<float>(y, x) *= d2_max;//d2_min; // normalize the distance
    }

    cv::Mat weight_out;
    weights[img_i].convertTo(weight_out, CV_8UC1, 255, 0);
    cv::imwrite(weightsPath + "/weight_"+std::to_string(img_i)+".png", weight_out);
}

// calculate valid patch to accelerate the patchmatch
void getAlignResults::calcValidPatch()
{
    LOG("[ Select valid patchs ]");
    LOG( " Valid Patch << ", false );
    img_valid_patch.clear(); // pixel_index => valid_info
    for( size_t t : kfIndexs ) {
        img_valid_patch[t] = cv::Mat1i(settings.imgH, settings.imgW);
        calcImgValidPatch(t);
        LOG( std::to_string(t) + " ", false );
    }
    LOG( "<< Done" );
}
void getAlignResults::calcImgValidPatch(size_t img_i)
{
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
#pragma omp parallel for
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;
        int result = 0;
        if( x < settings.imgW - settings.patchWidth + 1 && y < settings.imgH - settings.patchWidth + 1 )
            result = isPatchValid(img_i, x, y);
        img_valid_patch[img_i].at<int>(y, x) = result;
    }
}
int getAlignResults::isPatchValid(size_t img_i, int x, int y)
{
    // if the center is valid, then believe it's valid
    if ( weights[img_i].at<float>(y + settings.patchWidth/2, x + settings.patchWidth/2) > 0)
        return 1; // valid patch;
    return 0;
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
    showRemapping();
}
// for each pixel in img_i, remapping it to img_j only when the mesh is visible both in i and j
void getAlignResults::calcImgRemapping(size_t img_i, size_t img_j)
{
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
#pragma omp parallel for
    for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
        // if no depth, then no need to remapping
        struct valid_info * info = &img_valid_info[img_i][pixel_index];
        if( info->depth <= 0 )
            continue;

        int y = static_cast<int>(pixel_index) / settings.imgW;
        int x = static_cast<int>(pixel_index) % settings.imgW;
        if( img_i == img_j ){
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(0) = x;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(1) = y;
            mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) = 1;
            continue;
        }

        cv::Mat p_w = imgToWorld(x, y, info->depth, img_i);
        cv::Mat p_j = worldToImg(p_w, img_j);
        cv::Point2i p_img_j;
        p_img_j.x = static_cast<int>( round( static_cast<double>(p_j.at<float>(0)) ) );
        p_img_j.y = static_cast<int>( round( static_cast<double>(p_j.at<float>(1)) ) );
        if ( !pointProjectionValid(p_j.at<float>(2), img_j, p_img_j.x, p_img_j.y) )
            continue;

        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(0) = p_img_j.x;
        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(1) = p_img_j.y;
        mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) = 1;
    }
}
void getAlignResults::showRemapping()
{
    size_t total = static_cast<size_t>(settings.imgW * settings.imgH);
    for( size_t img_i : kfIndexs) {
        cv::Mat1d mat( settings.imgH, settings.imgW );
        for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
            int y = static_cast<int>(pixel_index) / settings.imgW;
            int x = static_cast<int>(pixel_index) % settings.imgW;
            mat.at<double>(y, x) = 0;
            for( size_t img_j : kfIndexs ) {
                if( mappings[img_i][img_j].at<cv::Vec3i>(y, x)(2) == 1 )
                    mat.at<double>(y, x) += 1;
            }
        }
#pragma omp parallel for
        for ( size_t pixel_index = 0; pixel_index < total; pixel_index++) {
            int y = static_cast<int>(pixel_index) / settings.imgW;
            int x = static_cast<int>(pixel_index) % settings.imgW;
            mat.at<double>(y, x) = mat.at<double>(y, x) * 1.0 / kfIndexs.size();
        }
        cv::Mat mat_out;
        mat.convertTo(mat_out, CV_8UC1, 255, 0);
        cv::imwrite(weightsPath + "/remap_"+std::to_string(img_i)+".png", mat_out);
    }
}

/*----------------------------------------------
 *  Do Iterations
 * ---------------------------------------------*/
void getAlignResults::doIterations()
{
    size_t scale = 0;
    //scale = settings.scaleTimes-1;
    bool init_T_M = true;
    char tmp_[16]; sprintf(tmp_, "%dx%d", settings.originImgW, settings.originImgH);
    std::string originResolution(tmp_);
    for ( ; scale < settings.scaleTimes; scale++) {
        // downsample imgs
        settings.imgW = static_cast<int>(std::round(settings.scaleInitW * 1.0 * pow(settings.scaleFactor, scale)));
        settings.imgH = static_cast<int>(std::round(settings.scaleInitH * 1.0 * pow(settings.scaleFactor, scale)));
        scaleF = settings.originImgW * 1.0 / settings.imgW;

        lamda = settings.lamda;
        patchRandomSearch = settings.patchRandomSearchTimes;

        char tmp[10]; sprintf(tmp, "%dx%d", settings.imgW, settings.imgH);
        std::string newResolution(tmp);
        LOG("[ Scale to " + newResolution + " (" + std::to_string(scale+1) + ") ]");
        LOG("[ Lamda: " + std::to_string(lamda) + " ]");

        // generate source imgs with new resolution // [REQUIRE] ImageMagick
        sourcesImgs.clear();
        for( size_t i : kfIndexs ) {
            std::string filename = EAGLE::getFilename(sourcesOrigin[i]);
            sourcesFiles[i] = sourcesPath + "/" + filename;
            system( ("convert " + sourcesOrigin[i] + " -resize " + newResolution + "! " + sourcesFiles[i]).c_str() );
            sourcesImgs[i] = cv::imread(sourcesFiles[i]);
        }
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
            for( size_t i : kfIndexs ){ // [REQUIRE] ImageMagick
                system( ("convert " + targetsFiles[i] + " -resize " + newResolution + "! " + targetsFiles[i]).c_str() );
                system( ("convert " + texturesFiles[i] + " -resize " + newResolution + "! " + texturesFiles[i]).c_str() );
            }
        }
        targetsImgs.clear(); texturesImgs.clear();
        for( size_t i : kfIndexs ) {
            targetsImgs[i] = cv::imread(targetsFiles[i]);
            texturesImgs[i] = cv::imread(texturesFiles[i]);
        }

        // using ray intersection method to get all pixels' depth and weight
        calcValidMesh();
        // calculate relative patchs to speed up the patchmatch
        calcValidPatch();
        // doing the remapping to project a pixel to other views
        calcRemapping();

        // do iterations
        for ( size_t _count = 0; _count < settings.scaleIters[scale]; _count++) {
            LOG("[ Iteration " + std::to_string(_count+1) + " at " + newResolution + " ]");
            E1 = 0; E2 = 0;
            LOG( " T << ", false );
            for( size_t i : kfIndexs ) {
                generateTargetI(i, texturesImgs);
                LOG(std::to_string(i) + " ", false);
            }
            LOG( "<< E1: " + std::to_string(E1), true );
            LOG( " M << ", false );
            for( size_t i : kfIndexs ) {
                generateTextureI(i, targetsImgs);
                LOG(std::to_string(i) + " ", false);
            }
            LOG( "<< E2: " + std::to_string(E2), true );
        }
        if( !OUTPUT_T_M_INSTANT )
            for( size_t i : kfIndexs ){
                cv::imwrite( targetsFiles[i], targetsImgs[i] );
                cv::imwrite( texturesFiles[i], texturesImgs[i] );
            }

        // save results
        for( size_t i : kfIndexs ){
            // [REQUIRE] ImageMagick
            system( ("convert " + targetsFiles[i] + " -resize " + originResolution + "! " + resultsPath+"/"+getImgFilename(i, "T_", "_"+std::to_string(scale+1)+"."+settings.rgbNameExt)).c_str() );
            system( ("convert " + texturesFiles[i] + " -resize " + originResolution + "! " + resultsPath+"/"+getImgFilename(i, "M_", "_"+std::to_string(scale+1)+"."+settings.rgbNameExt)).c_str() );
        }
        LOG( "[ Results at " + newResolution + " Saving Success ]" );
    }
    for( size_t i : kfIndexs ) {
        std::string s_file = resultsPath+"/" +getImgFilename(i, "S_", "."+settings.rgbNameExt);
        generateTextureIWithS(i, s_file);
        system( ("convert " + s_file + " -resize " + originResolution + "! " + s_file).c_str() );
    }
    generateTexturedOBJ(resultsPath, "S", "S_%03d");
    //generateTexturedOBJ(resultsPath, "T", "T_%03d_"+std::to_string(settings.scaleTimes));
    generateTexturedOBJ(resultsPath, "M", "M_%03d_"+std::to_string(settings.scaleTimes));
    LOG("[ Generate OBJ file Success ]");
}

/*----------------------------------------------
 *  PatchMatch
 * ---------------------------------------------*/
void getAlignResults::patchmatch(size_t img_id, cv::Mat3b a, cv::Mat3b b, cv::Mat3i &ann)
{
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        ann.at<cv::Vec3i>(j, i) = cv::Vec3i(i, j, 0);
        if( i < settings.imgW - settings.patchWidth + 1 && j < settings.imgH - settings.patchWidth + 1 )
            ann.at<cv::Vec3i>(j, i)(2) = dist(a, b, i, j, i, j, INT_MAX);
    }
    patchmatch_iter(img_id, a, b, ann, 0);
    patchmatch_iter(img_id, a, b, ann, 1);
    patchmatch_iter(img_id, a, b, ann, 0);
    patchmatch_iter(img_id, a, b, ann, 1);
    patchmatch_iter(img_id, a, b, ann, 0);
    patchmatch_iter(img_id, a, b, ann, 1);
}
void getAlignResults::patchmatch_iter(size_t img_id, cv::Mat3b a, cv::Mat3b b, cv::Mat3i &ann, int dir)
{
    int aew = settings.imgW - settings.patchWidth + 1, aeh = settings.imgH - settings.patchWidth + 1;
    int bew = aew, beh = aeh;
    int total_pm = aew * aeh;
    // Set search window when random searching
    int window_width = static_cast<int>( round(patchRandomSearch * sqrt(settings.imgW * settings.imgH)) );

    int xchange, ychange;
    if(dir == 0) { // from left-up to right-down
        xchange = -1;
        ychange = -1;
    } else { // from right-down to left-up
        xchange = 1;
        ychange = 1;
    }
    srand( static_cast<uint>(time(nullptr)) );

//#pragma omp parallel for
    for ( int index = 0; index < total_pm; index++) {
        int ax, ay, bx, by;
        if(dir == 0) {
            ay = index / aew;
            ax = index % aew;
        } else {
            ay = aeh-1 - index / aew;
            ax = aew-1 - index % aew;
        }
        // if it's not a valid patch, then continue
        if (img_valid_patch[img_id].at<int>(ay, ax) == 0)
            continue;

        /* Current (best) guess. */
        int xbest = ann.at<cv::Vec3i>(ay, ax)(0);
        int ybest = ann.at<cv::Vec3i>(ay, ax)(1);
        int dbest = ann.at<cv::Vec3i>(ay, ax)(2);

        /* Propagation: Improve current guess by trying instead correspondences from left and above (below and right on odd iterations). */
        int ax2 = ax + xchange;
        if (ax2 > -1 && ax2 < aew) {
            bx = ann.at<cv::Vec3i>(ay, ax2)(0) - xchange;
            by = ann.at<cv::Vec3i>(ay, ax2)(1);
            if (bx > -1 && bx < bew)
                improve_guess(a, b, ax, ay, xbest, ybest, dbest, bx, by);
        }
        int ay2 = ay + ychange;
        if (ay2 > -1 && ay2 < aeh) {
            bx = ann.at<cv::Vec3i>(ay2, ax)(0);
            by = ann.at<cv::Vec3i>(ay2, ax)(1) - ychange;
            if (by > -1 && by < beh)
                improve_guess(a, b, ax, ay, xbest, ybest, dbest, bx, by);
        }

        /* Random search: Improve current guess by searching in boxes of exponentially decreasing size around the current best guess. */
        for (int mag = window_width; mag >= 1; mag /= 2) {
            int xmin = MAX(xbest-mag, 0), xmax = MIN(xbest+mag+1, bew);
            int ymin = MAX(ybest-mag, 0), ymax = MIN(ybest+mag+1, beh);
            bx = xmin + rand() % (xmax - xmin);
            by = ymin + rand() % (ymax - ymin);
            improve_guess(a, b, ax, ay, xbest, ybest, dbest, bx, by);
        }

        ann.at<cv::Vec3i>(ay, ax)(0) = xbest;
        ann.at<cv::Vec3i>(ay, ax)(1) = ybest;
        ann.at<cv::Vec3i>(ay, ax)(2) = dbest;
    }
}
void getAlignResults::improve_guess(cv::Mat3b a, cv::Mat3b b, int ax, int ay, int &xbest, int &ybest, int &dbest, int bx, int by)
{
    int d = dist(a, b, ax, ay, bx, by, dbest);
    if (d < dbest) {
        dbest = d;
        xbest = bx;
        ybest = by;
    }
}
int getAlignResults::dist(cv::Mat3b a, cv::Mat3b b, int ax, int ay, int bx, int by, int cutoff)
{
    int ans = 0;
    for ( int index = 0; index < settings.patchSize; index++) {
        int j = index / settings.patchWidth;
        int i = index % settings.patchWidth;
        cv::Vec3b p_a = a.at<cv::Vec3b>(ay+j, ax+i);
        cv::Vec3b p_b = b.at<cv::Vec3b>(by+j, bx+i);
        for(int p_i = 0; p_i < 3; p_i++) {
            int d = static_cast<int>(p_a(p_i)) - static_cast<int>(p_b(p_i));
            ans += d * d;
        }
        if (ans >= cutoff)
            return cutoff;
    }
    return ans;
}

/*----------------------------------------------
 *  Generate Ti
 * ---------------------------------------------*/
void getAlignResults::generateTargetI(size_t target_id, std::map<size_t, cv::Mat3b> textures)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b target( cv::Size(settings.imgW, settings.imgH), cv::Vec3b(255,255,255) );

    // patchmatch
    cv::Mat3b sourceImg = sourcesImgs[target_id];
    cv::Mat3b targetImg = targetsImgs[target_id];
    cv::Mat3i result_ann_s2t( settings.imgH, settings.imgW ); // cv::Vec3i(x, y, d)
    patchmatch(target_id, sourceImg, targetImg, result_ann_s2t);
    cv::Mat3i result_ann_t2s( settings.imgH, settings.imgW );
    patchmatch(target_id, targetImg, sourceImg, result_ann_t2s);
    cv::Mat4i result_su( cv::Size(settings.imgW, settings.imgH) );
    cv::Mat4i result_sv( cv::Size(settings.imgW, settings.imgH) );
    getSimilarityTerm(sourceImg, result_ann_s2t, result_ann_t2s, result_su, result_sv);

    // calculate E1
    double E1_1 = 0, E1_2 = 0;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        if( i >= settings.imgW - (settings.patchWidth-1) || j >= settings.imgH - (settings.patchWidth-1))
            continue;
        E1_1 += result_ann_s2t.at<cv::Vec3i>(j, i)(2) * 1.0 / settings.patchSize;
        E1_2 += result_ann_t2s.at<cv::Vec3i>(j, i)(2) * 1.0 / settings.patchSize;
    }
    E1 += (settings.alpha_u * E1_1 + settings.alpha_v * E1_2) / 65025.0;

#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        // if the pixel is in bg, then no optimization
        cv::Vec3i Xij = mappings[target_id][target_id].at<cv::Vec3i>(j, i);
        if ( Xij(2) == 0 ) {
            target.at<cv::Vec3b>(j, i) = sourcesImgs[target_id].at<cv::Vec3b>(j, i);
            continue;
        }
        cv::Vec3d sum_bgr(0,0,0);

        // similarity term
        double _factor1;
        _factor1  = settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        _factor1 += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(3) / settings.patchSize;
        cv::Vec3d sum_S(0,0,0);
        for( int p_i = 0; p_i < 3; p_i++ ) {
            sum_S(p_i)  = settings.alpha_u * result_su.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            sum_S(p_i) += settings.alpha_v * result_sv.at<cv::Vec4i>(j,i)(p_i) / settings.patchSize;
            sum_bgr(p_i) = sum_S(p_i);
        }

        // consistency term
        double weight = static_cast<double>(weights[target_id].at<float>(j, i));
        double _factor2 = lamda * weight;
        cv::Vec3d sum_M(0,0,0); int sum_w = 0;
        for( size_t t : kfIndexs ) {
            cv::Vec3i Xij = mappings[target_id][t].at<cv::Vec3i>(j, i);
            if ( Xij(2) > 0 ){
                sum_M += textures[t].at<cv::Vec3b>(Xij(1), Xij(0)) * 1.0;
                sum_w += 1;
            }
        }
        sum_bgr += _factor2 * sum_M / sum_w;

        // generate the pixel of Ti
        cv::Vec3b bgr(0,0,0);
        for ( int p_i = 0; p_i < 3; p_i++ ) {
            sum_bgr(p_i) = sum_bgr(p_i) / (_factor1 + _factor2);
            bgr(p_i) = static_cast<uchar>( EAGLE_MAX(EAGLE_MIN(static_cast<int>(std::round(sum_bgr(p_i))), 255), 0) );
        }
        target.at<cv::Vec3b>(j, i) = bgr;
    }
    targetsImgs[target_id] = target;
    if ( OUTPUT_T_M_INSTANT ) {
        cv::imwrite( targetsFiles[target_id], target );
    }
}

void getAlignResults::getSimilarityTerm(cv::Mat3b S, cv::Mat3i ann_s2t, cv::Mat3i ann_t2s, cv::Mat4i &su, cv::Mat4i &sv)
{
    int total = settings.imgH * settings.imgW;
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        su.at<cv::Vec4i>(j, i) = cv::Vec4i(0,0,0,0);
        sv.at<cv::Vec4i>(j, i) = cv::Vec4i(0,0,0,0);
    }
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;
        // if in boundary, no need to check pm's results
        if( i >= settings.imgW - (settings.patchWidth-1) || j >= settings.imgH - (settings.patchWidth-1)) {
            calcSuv(S, i, j, su, i, j, 1);
            calcSuv(S, i, j, sv, i, j, 1);
            continue;
        }
        // set the patch's step
        if( i % settings.patchStep != 0 || j % settings.patchStep != 0 )
            continue;
        int x, y;
        // Su: completeness
        // here, (i,j) is on Si, and (x,y) on Ti
        x = ann_s2t.at<cv::Vec3i>(j, i)(0); y = ann_s2t.at<cv::Vec3i>(j, i)(1);
        calcSuv(S, i, j, su, x, y, settings.patchWidth);
        // Sv: coherence
        // here, (i,j) is on Ti, and (x,y) on Si
        x = ann_t2s.at<cv::Vec3i>(j, i)(0); y = ann_t2s.at<cv::Vec3i>(j, i)(1);
        calcSuv(S, x, y, sv, i, j, settings.patchWidth);
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
 *  Generate Mi
 * ---------------------------------------------*/
void getAlignResults::generateTextureI(size_t texture_id, std::map<size_t, cv::Mat3b> targets)
{
    int total = settings.imgH * settings.imgW;
    cv::Mat3b texture( cv::Size(settings.imgW, settings.imgH), cv::Vec3b(255,255,255) );
#pragma omp parallel for
    for ( int index = 0; index < total; index++) {
        int j = index / settings.imgW;
        int i = index % settings.imgW;

        cv::Vec3f sum(0,0,0);
        float weight = 0, sum_w = 0;
        cv::Vec3b pixel;

        // for E2 calculation
        std::vector<cv::Vec3b> E2_pixels;
        std::vector<float> E2_weights;

        for ( size_t t : kfIndexs ) {
            cv::Vec3i Xij = mappings[texture_id][t].at<cv::Vec3i>(j, i);
            if ( Xij(2) > 0 ){
                weight = weights[t].at<float>(Xij(1), Xij(0));
                pixel = targets[t].at<cv::Vec3b>(Xij(1), Xij(0));

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
                E2_1 += (E2_2 / 65025.0 * E2_weights[_i]);
            }
            double E2_TEST = (E2_1 / E2_pixels.size());
            if ( !std::isnan(E2_TEST) )
                E2 += E2_TEST;
        }
    }
    texturesImgs[texture_id] = texture;
    if ( OUTPUT_T_M_INSTANT ) {
        cv::imwrite( texturesFiles[texture_id], texture );
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

        cv::Vec3f sum(0,0,0);
        float weight = 0, sum_w = 0;
        cv::Vec3b pixel;
        for ( size_t t : kfIndexs ) {
            cv::Vec3i Xij = mappings[texture_id][t].at<cv::Vec3i>(j, i);
            if ( Xij(2) > 0 ) {
                weight = weights[t].at<float>(Xij(1), Xij(0));
                pixel = sourcesImgs[t].at<cv::Vec3b>(Xij(1), Xij(0));

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
            if (checkMeshMapImg(i, img_i, v_uv) == true) {
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
                    float uv_x = (v_uv[p_i].x + 0.5f) / settings.imgW;
                    float uv_y = (v_uv[p_i].y + 0.5f) / settings.imgH;
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

bool getAlignResults::checkMeshMapImg(size_t mesh_i, size_t img_i, std::vector<cv::Point2i> &v_uv)
{
    v_uv.clear();
    bool flag = true;
    for(size_t p_i = 0; p_i < 3; p_i++) {
        size_t v_index = mesh.polygons[mesh_i].vertices[p_i];
        cv::Mat X_w = (cv::Mat_<float>(4, 1) << cloud_rgb.points[v_index].x, cloud_rgb.points[v_index].y, cloud_rgb.points[v_index].z, 1);
        cv::Mat X_img = worldToImg(X_w, img_i);
        int _x = static_cast<int>( round(static_cast<double>(X_img.at<float>(0))) );
        int _y = static_cast<int>( round(static_cast<double>(X_img.at<float>(1))) );
        if( !pointProjectionValid(X_img.at<float>(2), img_i, _x, _y) ) {
            flag = false;
            break;
        } else {
            v_uv[p_i] = cv::Point2i(_x, _y);
        }
    }
    return flag;
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
