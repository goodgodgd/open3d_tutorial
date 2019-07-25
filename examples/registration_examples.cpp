#include "registration_examples.h"
#include <cassert>
#include <iostream>
#include <Open3D/Registration/ColoredICP.h>

namespace RegPar
{
const double depth_scale = 5000.0;  // from TUM RGBD format
const double depth_trunc = 4.0;
const double max_correspondence_dist = 0.3;
const Eigen::Matrix4d init_pose = Eigen::Matrix4d::Identity();
const open3d::geometry::KDTreeSearchParamHybrid kdtree(0.01, 20);
const open3d::camera::PinholeCameraIntrinsic intrinsic(
        open3d::camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
const open3d::registration::ICPConvergenceCriteria convergence(
        1e-6, 1e-6, 20);
}


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

    // draw two point clouds
    ShowTwoPointClouds(src_cloud, tgt_cloud, RegPar::init_pose, "point to plane ICP");

    // check point and normal
    uint32_t index = 240*src_depth->height_ + 400;
    open3d::utility::LogInfo("check point and normal: point={} | normal={}\n",
                             src_cloud->points_[index].transpose(), src_cloud->normals_[index].transpose());
    open3d::utility::LogInfo("check point and normal: point={} | normal={}\n",
                             src_cloud->points_[index+1].transpose(), src_cloud->normals_[index+1].transpose());
    // point-to-plane ICP
    open3d::registration::TransformationEstimationPointToPlane point_to_plane;
    open3d::registration::RegistrationResult result =
            open3d::registration::RegistrationICP(*src_cloud, *tgt_cloud,
                                                  RegPar::max_correspondence_dist,
                                                  RegPar::init_pose,
                                                  point_to_plane, RegPar::convergence);

    open3d::utility::LogInfo("point-to-plane ICP result: \n"
                             "fitness={}, transformation=\n{}\n",
                             result.fitness_, result.transformation_);
    // draw registration result
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
    o3PointCloudPtr ptcd_ptr = o3PointCloud::CreateFromRGBDImage(*rgbd_ptr, RegPar::intrinsic);
    open3d::utility::LogInfo("point cloud size: point={} | color={} | normal={}\n",
                             ptcd_ptr->points_.size(), ptcd_ptr->colors_.size(), ptcd_ptr->normals_.size());

    // estimate normal
    ptcd_ptr->EstimateNormals(RegPar::kdtree, false);
    open3d::utility::LogInfo("point cloud size: point={} | color={} | normal={}\n",
                             ptcd_ptr->points_.size(), ptcd_ptr->colors_.size(), ptcd_ptr->normals_.size());

    // access to point and color
    uint32_t index = 240*color_ptr->height_ + 400;
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             ptcd_ptr->points_[index].transpose(), ptcd_ptr->colors_[index].transpose());
    open3d::utility::LogInfo("check point cloud values: point={} | color={}\n",
                             ptcd_ptr->points_[index+1].transpose(), ptcd_ptr->colors_[index+1].transpose());

    // show point cloud
    open3d::visualization::DrawGeometries({ptcd_ptr}, "point cloud from rgbd");

    // write point cloud in ascii format without compression
    bool write_ascii = true, compressed = false;
    open3d::io::WritePointCloud(pcdfile, *ptcd_ptr, write_ascii, compressed);
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

    // draw initial state
    ShowTwoPointClouds(src_cloud, tgt_cloud, Eigen::Matrix4d_u::Identity(), "Before colored ICP");

    // access to point and color
    uint32_t index = 320*480 + 400;
    open3d::utility::LogInfo("check point cloud values: point={} | normal={}\n",
                             src_cloud->points_[index].transpose(), src_cloud->normals_[index].transpose());
    open3d::utility::LogInfo("check point cloud values: point={} | normal={}\n",
                             src_cloud->points_[index+1].transpose(), src_cloud->normals_[index+1].transpose());

    // colored ICP
    open3d::registration::RegistrationResult result =
            open3d::registration::RegistrationColoredICP(*src_cloud, *tgt_cloud,
                                                         RegPar::max_correspondence_dist,
                                                         RegPar::init_pose, RegPar::convergence);

    open3d::utility::LogInfo("colored ICP result: \nfitness={}, transformation=\n{}\n",
                             result.fitness_, result.transformation_);
    // draw results
    ShowTwoPointClouds(src_cloud, tgt_cloud, result.transformation_, "After colored ICP");
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
    open3d::odometry::OdometryOption option({20,10,10}, 0.5, 0, RegPar::depth_trunc);
    open3d::odometry::RGBDOdometryJacobianFromHybridTerm jacobian;
    // create point clouds
    o3PointCloudPtr src_cloud = o3PointCloud::CreateFromRGBDImage(*src_rgbd, RegPar::intrinsic);
    o3PointCloudPtr tgt_cloud = o3PointCloud::CreateFromRGBDImage(*tgt_rgbd, RegPar::intrinsic);
    // visualize initial state
    ShowTwoPointClouds(src_cloud, tgt_cloud, RegPar::init_pose, "RGBD Odometry");

    // visual odometry with recursive refinements
    std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> result;
    bool success;
    Eigen::Matrix4d motion = RegPar::init_pose;
    Eigen::Matrix6d inform;
    std::vector<double> max_depth_diffs = {0.5, 0.03, 0.01};
    const uint loop_iter = max_depth_diffs.size();
    for(uint i=0; i<loop_iter; i++)
    {
        option.max_depth_diff_ = max_depth_diffs[i];
        result = open3d::odometry::ComputeRGBDOdometry(*src_rgbd, *tgt_rgbd, RegPar::intrinsic,
                                                       motion, jacobian, option);
        success = std::get<0>(result);
        motion = std::get<1>(result);
        inform = std::get<2>(result);
        open3d::utility::LogInfo("RGBD Odometry result in iter {}/{}: success={}, motion=\n{}\n",
                                 i+1, loop_iter, success, motion);
    }
    // visualize result
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
    if(!source_tmp->HasColors())
        source_tmp->PaintUniformColor(Eigen::Vector3d(1, 0.7, 0));
    if(!target_tmp->HasColors())
        target_tmp->PaintUniformColor(Eigen::Vector3d(0, 0.7, 1));
    source_tmp->Transform(transform);
    open3d::visualization::DrawGeometries({source_tmp, target_tmp}, title);
}
