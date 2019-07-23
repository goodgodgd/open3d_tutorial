#ifndef OPEN3DEXAMPLES_H
#define OPEN3DEXAMPLES_H

#include <memory>

#include "Open3D/Open3D.h"


class Open3dExamples
{
public:
    Open3dExamples();
    static void ReadShowWrite_RGB(const char* srcname, const char* dstname,
                                  bool write_filtered=false);
    static void ReadShowWrite_Depth(const char* srcname, const char* dstname,
                                    bool write_scaled=false);
    static void ReadShowWrite_PointCloud(const char* colorname, const char *depthname,
                                         const char* pcdname);
    static void LogImageDimension(std::shared_ptr<open3d::geometry::Image> img_ptr,
                                  std::string name);
};

#endif // OPEN3DEXAMPLES_H
