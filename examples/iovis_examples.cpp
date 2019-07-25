#include "iovis_examples.h"

IoVis_Examples::IoVis_Examples()
{

}

void IoVis_Examples::ReadShowWrite_RGB(const char* srcname, const char* dstname, bool write_filtered)
{
    // read
    auto image_ptr = std::make_shared<o3Image>();
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
    auto blur_gray = image_gray_ptr->Filter(o3Image::FilterType::Gaussian3);
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

void IoVis_Examples::ReadShowWrite_Depth(const char* srcname, const char* dstname, bool write_scaled)
{
    // read
    o3ImagePtr depth_ptr = open3d::io::CreateImageFromFile(srcname);
    if(depth_ptr->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {}\n", srcname);
        return;
    }
    LogImageDimension(depth_ptr, "depth image");

    // convert depth scale
    auto depth_float = depth_ptr->ConvertDepthToFloatImage(10000.0);
    if(depth_float->IsEmpty())
    {
        open3d::utility::LogError("Failed to convert to float image\n");
        return;
    }
    LogImageDimension(depth_float, "depth float image");

    // compare depth
    int raw_depth = *depth_ptr->PointerAt<uint16_t>(400, 320, 0);
    float single_depth_float = depth_float->FloatValueAt(400, 320).second;
    open3d::utility::LogInfo("raw depth={}, float depth={}\n", raw_depth, single_depth_float);

    // show depth
    open3d::visualization::DrawGeometries({depth_float}, "Depth",
                                          depth_ptr->width_, depth_ptr->height_);

    // write
    if(write_scaled)
        open3d::io::WriteImage(dstname, *depth_float->CreateImageFromFloatImage<uint8_t>());
    else
        open3d::io::WriteImage(dstname, *depth_ptr);

    // convert depth to point cloud
    open3d::camera::PinholeCameraIntrinsic camera;
    camera.SetIntrinsics(640, 480, 575.0, 575.0, 319.5, 239.5);
    o3PointCloudPtr pointcloud_ptr = o3PointCloud::CreateFromDepthImage(*depth_ptr, camera);

    // show point cloud
    open3d::visualization::DrawGeometries({pointcloud_ptr}, "point cloud");
}

void IoVis_Examples::ReadShowWrite_PointCloud(const char* colorname, const char* depthname,
                                              const char* pcdname)
{
    // read color and depth
    o3ImagePtr color_ptr = open3d::io::CreateImageFromFile(colorname);
    LogImageDimension(color_ptr, "color image");
    o3ImagePtr depth_ptr = open3d::io::CreateImageFromFile(depthname);
    LogImageDimension(depth_ptr, "depth image");
    if(color_ptr->IsEmpty() || depth_ptr->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {} or {}\n\n", colorname, depthname);
        return;
    }

    // convert to rgbd image
    double depth_scale = 5000.0, depth_trunc = 3.0;
    bool convert_rgb_to_intensity = false;
    std::shared_ptr<open3d::geometry::RGBDImage> rgbd_ptr =
            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *color_ptr, *depth_ptr, depth_scale, depth_trunc,
                convert_rgb_to_intensity);

    // conver rgbd image to point cloud
    open3d::camera::PinholeCameraIntrinsic intrinsic(
                open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    o3PointCloudPtr ptcd_ptr = o3PointCloud::CreateFromRGBDImage(*rgbd_ptr, intrinsic);

    // access to point and color
    uint32_t index = 240*480 + 400;
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             ptcd_ptr->points_[index].transpose(),
                             ptcd_ptr->colors_[index].transpose());

    // show point cloud
    open3d::visualization::DrawGeometries({ptcd_ptr}, "point cloud from rgbd");

    // write point cloud in ascii format without compression
    bool write_ascii = true, compressed = false;
    open3d::io::WritePointCloud(pcdname, *ptcd_ptr, write_ascii, compressed);

    // read point cloud again
    o3PointCloudPtr neo_ptcd_ptr;
    neo_ptcd_ptr = open3d::io::CreatePointCloudFromFile(pcdname, "pcd", true);

    // show loaded point cloud
    open3d::visualization::DrawGeometries({neo_ptcd_ptr}, "loaded point cloud");
}

void IoVis_Examples::LogImageDimension(o3ImagePtr img_ptr, std::string name)
{
    open3d::utility::LogInfo("{} size: {:d} x {:d} x {:d} ({:d} bytes per channel)\n",
                             name, img_ptr->width_, img_ptr->height_, img_ptr->num_of_channels_,
                             img_ptr->bytes_per_channel_);
}
