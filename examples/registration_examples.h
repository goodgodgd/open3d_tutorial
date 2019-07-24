#ifndef REGISTRATION_EXAMPLES_H
#define REGISTRATION_EXAMPLES_H

#include "definitions.h"


class RegistrationExamples
{
public:
    RegistrationExamples();
    static void IcpPointCloud(const char* srcdepthfile, const char* tgtdepthfile);
    static void RgbDepthToPCD(const char* colorfile, const char* depthfile, const char* pcdfile);
    static void IcpColoredPointCloud(const char* srcpcdfile, const char* tgtpcdfile);
    static void VisualOdometryRgbDepth(const char* srccolorfile, const char* srcdepthfile,
                                       const char* tgtcolorfile, const char* tgtdepthfile);
    static void ShowTwoPointClouds(o3PointCloudPtr source, o3PointCloudPtr target,
                                          Eigen::Matrix4d_u transform, const std::string title="point clouds");
};

#endif // REGISTRATION_EXAMPLES_H
