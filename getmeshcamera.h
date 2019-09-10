#ifndef GETMESHCAMERA_H
#define GETMESHCAMERA_H

#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/pixel_rgb.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_buffer.h>
#include <pcl/gpu/kinfu_large_scale/tsdf_volume.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "settings.h"
#include "Eagle_Utils.h"

class getMeshCamera
{
private:

public:
    Settings settings;

    getMeshCamera(Settings &_settings);

    void getRGBVector(int, pcl::device::PtrStepSz<const pcl::gpu::kinfuLS::KinfuTracker::PixelRGB> &rgb24_);
    void getDVector(int, pcl::device::PtrStepSz<const unsigned short> &depth_);

    void writeCameraTraj(std::string filename, std::vector<Eigen::Matrix4f> mats);
};

#endif // GETMESHCAMERA_H
