#include "open3dexamples.h"

Open3dExamples::Open3dExamples()
{

}

void Open3dExamples::ReadShowWrite_RGB(const char* srcname, const char* dstname, bool write_filtered)
{
    // read
    auto image_ptr = std::make_shared<open3d::geometry::Image>();
    if(!open3d::io::ReadImage(srcname, *image_ptr))
    {
        open3d::utility::LogError("Failed to read {}\n\n", srcname);
        return;
    }
    LogImageDimension(image_ptr, "color image");

    // show
    open3d::visualization::DrawGeometries({image_ptr}, "Image",
                                          image_ptr->width_, image_ptr->height_);

    // filtering
    auto image_gray_ptr = image_ptr->CreateFloatImage();
    auto blur_gray = image_gray_ptr->Filter(open3d::geometry::Image::FilterType::Gaussian3);
    LogImageDimension(blur_gray, "filtered gray");

    // write
    if(write_filtered)
    {
        if(!blur_gray->IsEmpty())
            open3d::io::WriteImage(dstname, *blur_gray->CreateImageFromFloatImage<uint8_t>());
    }
    else
        open3d::io::WriteImage(dstname, *image_ptr);
}

void Open3dExamples::ReadShowWrite_Depth(const char* srcname, const char* dstname, bool write_scaled)
{
    // read
    std::shared_ptr<open3d::geometry::Image> depth_ptr =
            open3d::io::CreateImageFromFile(srcname);
    if(depth_ptr->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {}\n\n", srcname);
        return;
    }
    LogImageDimension(depth_ptr, "depth image");

    // convert depth scale
    auto depth_gray_ptr = depth_ptr->ConvertDepthToFloatImage(10000.0);
    LogImageDimension(depth_gray_ptr, "depth float image");
    int raw_depth = *depth_ptr->PointerAt<uint16_t>(400, 320, 0);
    float float_depth = depth_gray_ptr->FloatValueAt(400, 320).second;
    open3d::utility::LogInfo("raw depth={}, float depth={}\n", raw_depth, float_depth);

    // show depth
    open3d::visualization::DrawGeometries({depth_gray_ptr}, "Depth",
                                          depth_ptr->width_, depth_ptr->height_);

    // convert depth to point cloud
    open3d::camera::PinholeCameraIntrinsic camera;
    camera.SetIntrinsics(640, 480, 575.0, 575.0, 319.5, 239.5);
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr =
            open3d::geometry::PointCloud::CreateFromDepthImage(*depth_ptr, camera);

    // show point cloud
    open3d::visualization::DrawGeometries({pointcloud_ptr},
            "geometry::PointCloud from Depth geometry::Image", 1000, 700);

    // write
    if(write_scaled)
    {
        open3d::utility::LogInfo("save scaled depth");
        if(!depth_gray_ptr->IsEmpty())
            open3d::io::WriteImage(dstname, *depth_gray_ptr->CreateImageFromFloatImage<uint8_t>());
        return;
    }
    else
        open3d::io::WriteImage(dstname, *depth_ptr);

    // check the saved image has the same depth
    std::shared_ptr<open3d::geometry::Image> result_ptr =
            open3d::io::CreateImageFromFile(dstname);
    bool same = true;
    for(int v=0; v<depth_ptr->height_; v++)
    {
        for(int u=0; u<depth_ptr->width_; u++)
        {
            if(*depth_ptr->PointerAt<uint16_t>(u, v, 0)
                    != *result_ptr->PointerAt<uint16_t>(u, v, 0))
            {
                same = false;
                open3d::utility::LogInfo("depth at ({}.{}) {} != {}\n", u, v,
                                         *depth_ptr->PointerAt<uint16_t>(u, v, 0),
                                         *result_ptr->PointerAt<uint16_t>(u, v, 0));
            }
        }
    }
    if(same)
        open3d::utility::LogInfo("the saved image has the same depth\n");
}

void Open3dExamples::ReadShowWrite_PointCloud(const char* colorname, const char* depthname,
                                              const char* pcdname)
{
    // read color and depth
    std::shared_ptr<open3d::geometry::Image> color_ptr =
            open3d::io::CreateImageFromFile(colorname);
    LogImageDimension(color_ptr, "color image");
    std::shared_ptr<open3d::geometry::Image> depth_ptr =
            open3d::io::CreateImageFromFile(depthname);
    LogImageDimension(depth_ptr, "depth image");
    if(color_ptr->IsEmpty() || depth_ptr->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {} or {}\n\n", colorname, depthname);
        return;
    }

    // convert to rgbd image
    double depth_scale = 10000.0, depth_trunc = 3.0;
    bool convert_rgb_to_intensity = true;
    std::shared_ptr<open3d::geometry::RGBDImage> rgbd_ptr =
            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *color_ptr, *depth_ptr, depth_scale, depth_trunc, convert_rgb_to_intensity);

    // conver rgbd image to point cloud
    open3d::camera::PinholeCameraIntrinsic intrinsic =
            open3d::camera::PinholeCameraIntrinsic(
                open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr =
            open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd_ptr, intrinsic);

    // access to point and color
    // point cloud is listed in column-first order
    uint32_t index = 240*color_ptr->height_ + 400;
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             pcd_ptr->points_[index].transpose(), pcd_ptr->colors_[index].transpose());

    // show point cloud
    open3d::visualization::DrawGeometries({pcd_ptr}, "point cloud from rgbd");

    // write point cloud in ascii format without compression
    bool write_ascii = true, compressed = false;
    open3d::io::WritePointCloud(pcdname, *pcd_ptr, write_ascii, compressed);

    // read point cloud again
    std::shared_ptr<open3d::geometry::PointCloud> neo_pcd_ptr;
    neo_pcd_ptr = open3d::io::CreatePointCloudFromFile(pcdname, "pcd", true);
    open3d::utility::LogInfo("loaded point cloud values: point={} | color={}\n",
                             neo_pcd_ptr->points_[index].transpose(), neo_pcd_ptr->colors_[index].transpose());

    // show loaded point cloud
    open3d::visualization::DrawGeometries({neo_pcd_ptr}, "loaded point cloud");
}

void Open3dExamples::LogImageDimension(std::shared_ptr<open3d::geometry::Image> img_ptr,
                                       std::string name)
{
    open3d::utility::LogInfo("{} size: {:d} x {:d} x {:d} ({:d} bytes per channel)\n",
                             name, img_ptr->width_, img_ptr->height_, img_ptr->num_of_channels_,
                             img_ptr->bytes_per_channel_);
}
