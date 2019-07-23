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
        open3d::utility::LogInfo("Successfully read {}\n", srcname);
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
