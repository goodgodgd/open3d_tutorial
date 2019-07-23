#include "open3dexamples.h"
#include "Open3D/Open3D.h"
#include <typeinfo>

Open3dExamples::Open3dExamples()
{

}

void Open3dExamples::ReadShowWrite_RGB(const char* srcname, const char* dstname)
{
    // read
    auto image_ptr = std::make_shared<open3d::geometry::Image>();
    if(open3d::io::ReadImage(srcname, *image_ptr))
    {
        open3d::utility::LogInfo("size: {:d} x {:d} x {:d} ({:d} bytes per channel)\n",
            image_ptr->width_, image_ptr->height_, image_ptr->num_of_channels_,
            image_ptr->bytes_per_channel_);
        open3d::utility::LogInfo("data type: {} == {} (uint8)\n",
                                 typeid(image_ptr->data_[0]).name(), typeid(uint8_t).name());
    }
    else
    {
        open3d::utility::LogError("Failed to read {}\n\n", srcname);
        return;
    }

    // visualize
    open3d::visualization::DrawGeometries({image_ptr}, "Image",
                                          image_ptr->width_, image_ptr->height_);

    // filtering
    auto image_gray_ptr = image_ptr->CreateFloatImage();
    auto blur_gray = image_gray_ptr->Filter(open3d::geometry::Image::FilterType::Gaussian3);
    open3d::utility::LogInfo("blur_gray size: {:d} x {:d} x {:d} ({:d} bytes per channel)\n",
        blur_gray->width_, blur_gray->height_, blur_gray->num_of_channels_,
        blur_gray->bytes_per_channel_);

    // save
    if(!blur_gray->IsEmpty())
        open3d::io::WriteImage(dstname, *blur_gray->CreateImageFromFloatImage<uint8_t>());
}

void Open3dExamples::ReadShowWrite_Depth(const char* srcname, const char* dstname)
{
    std::shared_ptr<open3d::geometry::Image> image_ptr =
            open3d::io::CreateImageFromFile(srcname);
    if(!image_ptr->IsEmpty())
    {
        open3d::utility::LogInfo("size: {:d} x {:d} x {:d} ({:d} bytes per channel)\n",
            image_ptr->width_, image_ptr->height_, image_ptr->num_of_channels_,
            image_ptr->bytes_per_channel_);
        open3d::utility::LogInfo("data type: {} == {} (uint8)\n",
                                 typeid(image_ptr->data_[0]).name(), typeid(uint8_t).name());
    }
    else
    {
        open3d::utility::LogError("Failed to read {}\n\n", srcname);
        return;
    }

    open3d::camera::PinholeCameraIntrinsic camera;
    camera.SetIntrinsics(640, 480, 575.0, 575.0, 319.5, 239.5);
    std::shared_ptr<open3d::geometry::PointCloud> pointcloud_ptr =
            open3d::geometry::PointCloud::CreateFromDepthImage(*image_ptr, camera);
    open3d::visualization::DrawGeometries({pointcloud_ptr},
            "geometry::PointCloud from Depth geometry::Image", 1000, 700);
}
