#ifndef OPEN3DEXAMPLES_H
#define OPEN3DEXAMPLES_H

#include "definitions.h"


class IoVis_Examples
{
public:
    IoVis_Examples();
    static void ReadShowWrite_RGB(const char* srcname, const char* dstname,
                                  bool write_filtered=false);
    static void ReadShowWrite_Depth(const char* srcname, const char* dstname,
                                    bool write_scaled=false);
    static void ReadShowWrite_PointCloud(const char* colorname, const char *depthname,
                                         const char* pcdname);
    static void LogImageDimension(o3ImagePtr img_ptr,
                                  std::string name);
};

#endif // OPEN3DEXAMPLES_H
