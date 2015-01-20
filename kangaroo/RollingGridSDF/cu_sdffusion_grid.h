#pragma once

#include "kangaroo/platform.h"
#include "kangaroo/Mat.h"
#include "kangaroo/Image.h"
#include "kangaroo/ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "kangaroo/Sdf.h"
#include "SdfSmart.h"
#include "cu_sdf_reset.h"

namespace roo
{
// -----------------------------------------------------------------------------
// min_depth is 0.5 meter for the kinect sensor
KANGAROO_EXPORT
void SDFInitGrayGrid(
    int* pNextInitSDFs, BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth);

KANGAROO_EXPORT
void SdfFuseDirectGrayGrid(
    roo::BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
    ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta, float min_depth);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    );

// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
KANGAROO_EXPORT
void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs, BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs, BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight);

/////////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
void SDFInitGrayGrid(
    int* pNextInitSDFs, BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth
    );

KANGAROO_EXPORT
void SdfFuseDirectGrayGrid(
    roo::BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
    ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta, float min_depth);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth );

// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
KANGAROO_EXPORT
void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs, BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs,  BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, float min_depth, bool bWeight);
}
