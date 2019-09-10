#include "getmeshcamera.h"

getMeshCamera::getMeshCamera(Settings &_settings)
{
    settings = _settings;
    std::cout << "----- Mesh Generation -----" << std::endl;

    float volume_size = settings.volumeSize;
    Eigen::Vector3f volume(volume_size, volume_size, volume_size);
    pcl::gpu::kinfuLS::KinfuTracker kft(volume, volume_size * 0.8f, settings.originImgH, settings.originImgW);
    // lab data  //(527.3f, 527.08f, 323.73f, 277.25f);
    kft.setDepthIntrinsics(settings.cameraFx, settings.cameraFy, settings.cameraCx, settings.cameraCy);
    kft.setDepthTruncationForICP(4.5);
    kft.setIcpCorespFilteringParams( 0.1f/*meter*/, (float)sin(20.0f / 180 * 3.14159) );

    pcl::device::PtrStepSz<const unsigned short> depth;
    pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_map;

    std::vector <Eigen::Matrix4f> cameraTrajs; // storing camera's world positions

    int shiftCount = 0; //
    for( int i = settings.frameStart; i <= settings.frameEnd; i++ ){
        std::cout << "[ Frame " << i << " ]" << std::endl;
        getDVector(i, depth);
        depth_map.upload( depth.data, depth.step, depth.rows, depth.cols );
        kft(depth_map); // process the next frame

        Eigen::Affine3f pos = kft.getCameraPose(i);
        Eigen::Matrix4f posMat = pos.matrix();
        std::cout << posMat << std::endl;
        cameraTrajs.push_back( posMat );

        if( kft.hasShifted() == true ){ // return whether the last update resulted in a shift.
            shiftCount += 1;
            std::cout << "< Frame " << i << " Shift >" << std::endl;
        }
        if( kft.icpIsLost() == true ){ // return true if ICP is currently lost.
            std::cout << "< Frame " << i << " Failed >" << std::endl;
            break;
        }
    }
    std::cout << "[ Shift Count: " << shiftCount << " ]" << std::endl;

    writeCameraTraj(settings.cameraTxtFile, cameraTrajs);
    std::cout << "[ Camera Trajectory Success ]" << std::endl;

    // save the pcd file
    kft.extractAndSaveWorld();
    system( ("cp world.pcd " + settings.pcdWorldFile).c_str() );
    std::cout << "[ PCD File Success ]" << std::endl;

    // show the pcd
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCDFile<pcl::PointXYZ>(settings.pcdWorldFile, *cloud);
//    pcl::visualization::CloudViewer viewer("DEMO");
//    viewer.showCloud(cloud);
//    while(!viewer.wasStopped()){}

    std::cout << "------------------------------------" << std::endl;
    return;
}

void getMeshCamera::getRGBVector(int index, pcl::device::PtrStepSz<const pcl::gpu::kinfuLS::KinfuTracker::PixelRGB> &rgb24_)
{
    char buf[256];
//   sprintf(buf, "%s/%05d.jpg", settings.allFramesPath.c_str(), index);
    sprintf(buf, (settings.allFramesPath + "/" + settings.rgbNamePattern).c_str(), index);
    std::string imgFile = buf;
    cv::Mat3b rgbData = cv::imread(imgFile);
    std::cout << "--- RGB : " << imgFile << std::endl;

    pcl::gpu::kinfuLS::KinfuTracker::PixelRGB * _rgbData = new pcl::gpu::kinfuLS::KinfuTracker::PixelRGB[ rgbData.size().width * rgbData.size().height ];

    // 2 * 5 image converted into a vector
    // 0 1 2 3 4
    // 5 6 7 8 9
    //int pixel_index;
    for( int j = 0; j < rgbData.size().height; j++ ){
        for( int i = 0; i < rgbData.size().width; i++ ){
            //pixel_index = j * rgbData.size().width + i;
            _rgbData->r = rgbData.at<cv::Vec3b>(j, i)[0];
            _rgbData->g = rgbData.at<cv::Vec3b>(j, i)[1];
            _rgbData->b = rgbData.at<cv::Vec3b>(j, i)[2];
        }
    }

    rgb24_.cols = rgbData.size().width;
    rgb24_.rows = rgbData.size().height;
    rgb24_.step = rgb24_.cols * rgb24_.elemSize();
    rgb24_.data = &_rgbData[0];
}

void getMeshCamera::getDVector(int index, pcl::device::PtrStepSz<const unsigned short> &depth_)
{
    char buf[256];
    sprintf(buf, (settings.allFramesPath + "/" + settings.dNamePattern).c_str(), index);
    //    sprintf(buf, "%s/%05d.png", settings.allFramesPath.c_str(), index);
    std::string depthFile = buf;
    cv::Mat1w dData = cv::imread(depthFile, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
    std::cout << "---  D  : " << depthFile << std::endl;

    // 2 * 5 image converted into a vector
    // 0 1 2 3 4
    // 5 6 7 8 9
    unsigned short * _dData = new unsigned short[ dData.size().width * dData.size().height ];
    int pixel_index;
    for( int j = 0; j < dData.size().height; j++ ){
        for( int i = 0; i < dData.size().width; i++ ){
            pixel_index = j * dData.size().width + i;
            _dData[pixel_index] = dData.at<ushort>(j, i);
        }
    }

    depth_.cols = dData.size().width;
    depth_.rows = dData.size().height;
    depth_.step = depth_.cols * depth_.elemSize();
    depth_.data = &_dData[0];
}

void getMeshCamera::writeCameraTraj(std::string filename, std::vector<Eigen::Matrix4f> mats)
{
    std::ofstream ofs( filename.c_str() );
    Eigen::Matrix4f tmpMat;
    for( size_t i = 0; i < mats.size(); i++ ){
        tmpMat = mats[i];
        ofs << i << " " << i << " " << i << std::endl;
        ofs<<tmpMat(0,0)<<" "<<tmpMat(0,1)<<" "<<tmpMat(0,2)<<" "<<tmpMat(0,3)<<std::endl;
        ofs<<tmpMat(1,0)<<" "<<tmpMat(1,1)<<" "<<tmpMat(1,2)<<" "<<tmpMat(1,3)<<std::endl;
        ofs<<tmpMat(2,0)<<" "<<tmpMat(2,1)<<" "<<tmpMat(2,2)<<" "<<tmpMat(2,3)<<std::endl;
        ofs<<tmpMat(3,0)<<" "<<tmpMat(3,1)<<" "<<tmpMat(3,2)<<" "<<tmpMat(3,3)<<std::endl;
    }
    ofs.close();
}
