#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <memory>
#include <Open3D/Open3D.h>

typedef open3d::geometry::Image o3Image;
typedef std::shared_ptr<o3Image> o3ImagePtr;
typedef open3d::geometry::PointCloud o3PointCloud;
typedef std::shared_ptr<o3PointCloud> o3PointCloudPtr;

#endif // DEFINITIONS_H
