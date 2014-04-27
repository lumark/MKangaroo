#pragma once

#include "Mat.h"
#include "Image.h"
#include "ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "BoundedVolume.h"
#include "Sdf.h"
#include "SDFPointCloud.h"

namespace roo
{

void SdfFuse(BoundedVolume<SDF_t> vol, Image<float> depth, Image<float4> norm,
             Mat<float,3,4> T_cw, ImageIntrinsics K, float trunc_dist, float maxw, float mincostheta );

void SdfFuse(BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol, Image<float> depth,
             Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics K, Image<uchar3> img,
             Mat<float,3,4> T_iw, ImageIntrinsics Kimg,float trunc_dist, float max_w, float mincostheta);

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SdfFuseDirectGrey(BoundedVolume<SDF_t> vol, BoundedVolume<float> colorVol,
                       Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
                       ImageIntrinsics Kdepth, Image<float> grey, Mat<float,3,4> T_iw,
                       ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta);

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SDFInitGreyGrid(int* pNextInitSDFs,
                     BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                     BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                     Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                     Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                     float trunc_dist, float max_w, float mincostheta
                     );

void SdfFuseDirectGreyGrid(roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage> vol,
                           roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                           Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
                           ImageIntrinsics Kdepth, Image<float> grey, Mat<float,3,4> T_iw,
                           ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta);


// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
void SdfFuseDirectGreyGridAutoInit(int* pNextInitSDFs,
                                   BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                                   BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                                   Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                   Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                   float trunc_dist, float max_w, float mincostheta
                                   );

void SdfFuseDirectGreyGridDesireIndex(int* pNextInitSDFs,
                                      BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                                      BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                                      Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                      Image<float> grey, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                      float trunc_dist, float max_w, float mincostheta
                                      );

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SdfFuseColor(BoundedVolume<SDF_t> vol, BoundedVolume<uchar3> colorVol,
                  Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
                  ImageIntrinsics K, Image<float> img, Image<uchar3> Imgrgb,
                  Mat<float,3,4> T_iw, ImageIntrinsics Kimg, float trunc_dist,
                  float max_w, float mincostheta);

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void SdfReset(BoundedVolume<SDF_t> vol, float trunc_dist = 0);

void SdfReset(BoundedVolume<float> vol);

void SdfReset(BoundedVolume<uchar3> vol);

void SdfReset(VolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol);

void SdfReset(VolumeGrid<float, roo::TargetDevice, roo::Manage> vol);

void SdfSphere(BoundedVolume<SDF_t> vol, float3 center, float r);

void SdfDistance(Image<float> dist, Image<float> depth, BoundedVolume<SDF_t> vol, const Mat<float,3,4> T_wc, ImageIntrinsics K, float trunc_distance);

}
