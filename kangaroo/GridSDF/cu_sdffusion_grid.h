#pragma once

#include "kangaroo/platform.h"
#include "Kangaroo/Mat.h"
#include "Kangaroo/Image.h"
#include "Kangaroo/ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "Kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"
#include "kangaroo/GridSDF/cu_sdf_reset.h"

namespace roo
{
// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
KANGAROO_EXPORT
void SDFInitGrayGrid(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    );

KANGAROO_EXPORT
void SdfFuseDirectGrayGrid(
    roo::BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
    ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    );

// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
KANGAROO_EXPORT
void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, bool bWeight);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, bool bWeight);

/////////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
void SDFInitGrayGrid(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    );

KANGAROO_EXPORT
void SdfFuseDirectGrayGrid(
    roo::BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw,
    ImageIntrinsics Kdepth, Image<float> gray, Mat<float,3,4> T_iw,
    ImageIntrinsics Krgb, float trunc_dist, float max_w, float mincostheta);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridSafe(
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta
    );

// -----------------------------------------------------------------------------
// the following function combine init and fuse together, save time
KANGAROO_EXPORT
void SdfFuseDirectGrayGridAutoInit(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, bool bWeight);

KANGAROO_EXPORT
void SdfFuseDirectGrayGridDesireIndex(
    int* pNextInitSDFs,
    BoundedVolumeGrid<SDF_t_Smart, roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol,
    Image<float> depth, Image<float4> norm, Mat<float,3,4> T_cw, ImageIntrinsics Kdepth,
    Image<float> gray, Mat<float,3,4> T_iw, ImageIntrinsics Krgb,
    float trunc_dist, float max_w, float mincostheta, bool bWeight);
}
