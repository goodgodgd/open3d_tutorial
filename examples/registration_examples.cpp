#include "registration_examples.h"
#include <cassert>
#include <iostream>
#include <Open3D/Registration/ColoredICP.h>

struct RegPar   // registration params
{
    static double depth_scale;
    static double depth_trunc;
    static double max_correspondence_dist;
    static Eigen::Matrix4d init_pose;
    static open3d::geometry::KDTreeSearchParamHybrid kdtree;
    static open3d::camera::PinholeCameraIntrinsic intrinsic;
};
double RegPar::depth_scale = 5000.0;
double RegPar::depth_trunc = 4.0;
double RegPar::max_correspondence_dist = 0.1;
Eigen::Matrix4d RegPar::init_pose = Eigen::Matrix4d::Identity();
open3d::geometry::KDTreeSearchParamHybrid RegPar::kdtree(0.01, 20);
open3d::camera::PinholeCameraIntrinsic RegPar::intrinsic(
        open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);


RegistrationExamples::RegistrationExamples()
{
}

void RegistrationExamples::IcpPointCloud(const char* srcdepthfile, const char* tgtdepthfile)
{
    // read
    o3ImagePtr src_depth = open3d::io::CreateImageFromFile(srcdepthfile);
    o3ImagePtr tgt_depth = open3d::io::CreateImageFromFile(tgtdepthfile);
    if(src_depth->IsEmpty() || tgt_depth->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {} or {}", srcdepthfile, tgtdepthfile);
        return;
    }

    // convert depth to point cloud
    o3PointCloudPtr src_cloud = o3PointCloud::CreateFromDepthImage(
                *src_depth, RegPar::intrinsic, RegPar::init_pose, RegPar::depth_scale, RegPar::depth_trunc);
    o3PointCloudPtr tgt_cloud = o3PointCloud::CreateFromDepthImage(
                *tgt_depth, RegPar::intrinsic, RegPar::init_pose, RegPar::depth_scale, RegPar::depth_trunc);

    // estimate normal
    src_cloud->EstimateNormals(RegPar::kdtree, true);
    tgt_cloud->EstimateNormals(RegPar::kdtree, true);

    // check point and normal
    uint32_t index = 240*src_depth->height_ + 400;
    open3d::utility::LogInfo("check point and normal: point={} | normal={}\n",
                             src_cloud->points_[index].transpose(), src_cloud->normals_[index].transpose());
    open3d::utility::LogInfo("check point and normal: point={} | normal={}\n",
                             src_cloud->points_[index+1].transpose(), src_cloud->normals_[index+1].transpose());

    // draw results
    ShowTwoPointClouds(src_cloud, tgt_cloud, Eigen::Matrix4d_u::Identity(), "point to plane ICP");

    // point-to-plane ICP
    open3d::registration::TransformationEstimationPointToPlane point_to_plane;
    open3d::registration::ICPConvergenceCriteria convergence(1e-6, 1e-6, 20);
    open3d::registration::RegistrationResult result =
            open3d::registration::RegistrationICP(*src_cloud, *tgt_cloud,
                                                  RegPar::max_correspondence_dist,
                                                  RegPar::init_pose,
                                                  point_to_plane, convergence);

    open3d::utility::LogInfo("point-to-plane ICP transformation result: \n  fitness={}, transformation=\n{}\n",
                             result.fitness_, result.transformation_);
    // draw results
    ShowTwoPointClouds(src_cloud, tgt_cloud, result.transformation_, "point to plane ICP");
}

void RegistrationExamples::RgbDepthToPCD(const char* colorfile, const char* depthfile, const char* pcdfile)
{
    // read color and depth
    o3ImagePtr color_ptr = open3d::io::CreateImageFromFile(colorfile);
    o3ImagePtr depth_ptr = open3d::io::CreateImageFromFile(depthfile);
    if(color_ptr->IsEmpty() || depth_ptr->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {} or {}\n\n", colorfile, depthfile);
        return;
    }

    // convert to rgbd image
    bool convert_rgb_to_intensity = false;
    std::shared_ptr<open3d::geometry::RGBDImage> rgbd_ptr =
            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *color_ptr, *depth_ptr, RegPar::depth_scale, RegPar::depth_trunc, convert_rgb_to_intensity);

    // conver rgbd image to point cloud
    o3PointCloudPtr pcd_ptr = o3PointCloud::CreateFromRGBDImage(*rgbd_ptr, RegPar::intrinsic);
    open3d::utility::LogInfo("point cloud size: point={} | color={} | normal={}\n",
                             pcd_ptr->points_.size(), pcd_ptr->colors_.size(), pcd_ptr->normals_.size());

    // estimate normal
    pcd_ptr->EstimateNormals(RegPar::kdtree, false);
    open3d::utility::LogInfo("point cloud size: point={} | color={} | normal={}\n",
                             pcd_ptr->points_.size(), pcd_ptr->colors_.size(), pcd_ptr->normals_.size());

    // access to point and color
    uint32_t index = 240*color_ptr->height_ + 400;
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             pcd_ptr->points_[index].transpose(), pcd_ptr->colors_[index].transpose());
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             pcd_ptr->points_[index+1].transpose(), pcd_ptr->colors_[index+1].transpose());

    // show point cloud
    open3d::visualization::DrawGeometries({pcd_ptr}, "point cloud from rgbd");

    // write point cloud in ascii format without compression
    bool write_ascii = true, compressed = false;
    open3d::io::WritePointCloud(pcdfile, *pcd_ptr, write_ascii, compressed);
}

void RegistrationExamples::IcpColoredPointCloud(const char* srcpcdfile, const char* tgtpcdfile)
{
    // read
    o3PointCloudPtr src_cloud = open3d::io::CreatePointCloudFromFile(srcpcdfile);
    o3PointCloudPtr tgt_cloud = open3d::io::CreatePointCloudFromFile(tgtpcdfile);
    if(src_cloud->IsEmpty() || tgt_cloud->IsEmpty())
    {
        open3d::utility::LogError("Failed to read {} or {}", srcpcdfile, tgtpcdfile);
        return;
    }

    // draw results
    ShowTwoPointClouds(src_cloud, tgt_cloud, Eigen::Matrix4d_u::Identity(), "colored ICP");

    // colored ICP
    open3d::registration::ICPConvergenceCriteria convergence(1e-6, 1e-6, 20);
    open3d::registration::RegistrationResult result =
            open3d::registration::RegistrationColoredICP(*src_cloud, *tgt_cloud,
                                                  RegPar::max_correspondence_dist, RegPar::init_pose,
                                                  convergence);

    open3d::utility::LogInfo("colored ICP transformation result: \n  fitness={}, transformation=\n{}\n",
                             result.fitness_, result.transformation_);
    // draw results
    ShowTwoPointClouds(src_cloud, tgt_cloud, result.transformation_, "colored ICP");
}

void RegistrationExamples::VisualOdometryRgbDepth(const char* srccolorfile, const char* srcdepthfile,
                                                  const char* tgtcolorfile, const char* tgtdepthfile)
{
    // read
    o3ImagePtr src_color = open3d::io::CreateImageFromFile(srccolorfile);
    o3ImagePtr src_depth = open3d::io::CreateImageFromFile(srcdepthfile);
    o3ImagePtr tgt_color = open3d::io::CreateImageFromFile(tgtcolorfile);
    o3ImagePtr tgt_depth = open3d::io::CreateImageFromFile(tgtdepthfile);
    if(src_color->IsEmpty() || src_depth->IsEmpty() || tgt_color->IsEmpty() || tgt_depth->IsEmpty())
    {
        open3d::utility::LogError("Failed to read files");
        return;
    }

    // convert to rgbd image
    // Note! rgb image MUST be converted to intensity for odometry use
    bool convert_rgb_to_intensity = true;
    std::shared_ptr<open3d::geometry::RGBDImage> src_rgbd =
            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *src_color, *src_depth, RegPar::depth_scale, RegPar::depth_trunc, convert_rgb_to_intensity);
    std::shared_ptr<open3d::geometry::RGBDImage> tgt_rgbd =
            open3d::geometry::RGBDImage::CreateFromColorAndDepth(
                *tgt_color, *tgt_depth, RegPar::depth_scale, RegPar::depth_trunc, convert_rgb_to_intensity);

    // visual odometry parameters
    open3d::odometry::OdometryOption option({20,10,5}, RegPar::max_correspondence_dist,
                                            0, RegPar::depth_trunc);
    open3d::odometry::RGBDOdometryJacobianFromHybridTerm jacobian;

    // visual odometry
    auto result = open3d::odometry::ComputeRGBDOdometry(*src_rgbd, *tgt_rgbd, RegPar::intrinsic,
                                                        RegPar::init_pose, jacobian, option);
    bool success = std::get<0>(result);
    Eigen::Matrix4d motion = std::get<1>(result);
    Eigen::Matrix6d inform = std::get<2>(result);
    open3d::utility::LogInfo("RGBD Odometry result: success={}, motion=\n{}\n information matrix=\n{}\n",
                             success, motion, inform);

    // visualize result
    o3PointCloudPtr src_cloud = o3PointCloud::CreateFromRGBDImage(*src_rgbd, RegPar::intrinsic);
    o3PointCloudPtr tgt_cloud = o3PointCloud::CreateFromRGBDImage(*tgt_rgbd, RegPar::intrinsic);
    ShowTwoPointClouds(src_cloud, tgt_cloud, motion, "RGBD Odometry");
}

void RegistrationExamples::ShowTwoPointClouds(o3PointCloudPtr source, o3PointCloudPtr target,
                                              Eigen::Matrix4d_u transform, const std::string title)
{
    // since there is no deep copy function, use crop instead
    Eigen::Vector3d range(10000.0, 10000.0, 10000.0);
    o3PointCloudPtr source_tmp = source->Crop(-range, range);
    o3PointCloudPtr target_tmp = target->Crop(-range, range);
    // set uniform color to point cloud
    source_tmp->PaintUniformColor(Eigen::Vector3d(1, 0.7, 0));
    target_tmp->PaintUniformColor(Eigen::Vector3d(0, 0.7, 1));
    source_tmp->Transform(transform);
    open3d::visualization::DrawGeometries({source_tmp, target_tmp}, title);
}
