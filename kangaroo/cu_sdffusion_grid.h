#pragma once

#include "Mat.h"
#include "Image.h"
#include "ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "Sdf.h"

namespace roo
{
// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SDFInitGrayGrid(int* pNextInitSDFs,
                     BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                     BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                     Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                     Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                     float trunc_dist, float max_w, float mincostheta
                     );

void SdfFuseDirectGrayGrid(roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage> vol,
                           roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                           Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
                           ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
                           ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta);

void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    );

// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
void SdfFuseDirectGrayGridAutoInit(int* pNextInitSDFs,
                                   BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                                   BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                                   Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                   Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                   float trunc_dist, float max_w, float mincostheta, bool bWeight);

void SdfFuseDirectGrayGridDesireIndex(int* pNextInitSDFs,
                                      BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
                                      BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
                                      Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
                                      Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
                                      float trunc_dist, float max_w, float mincostheta, bool bWeight);

void SdfReset(VolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol);

void SdfReset(VolumeGrid<float, roo::TargetDevice, roo::Manage> vol);

void SdfReset(BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> vol);

void SdfReset(BoundedVolumeGrid<roo::SDF_t,roo::TargetDevice, roo::Manage> vol);

}
