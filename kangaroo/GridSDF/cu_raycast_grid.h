#pragma once

#include "kangaroo/platform.h"
#include "kangaroo/Mat.h"
#include "kangaroo/Image.h"
#include "BoundedVolumeGrid.h"
#include "kangaroo/ImageIntrinsics.h"
#include "kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"

namespace roo
{

// -----------------------------------------------------------------------------
KANGAROO_EXPORT
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix = true);

// raycast grid sdf
KANGAROO_EXPORT
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
                const BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> grayVol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix );


// -----------------------------------------------------------------------------
// for smart SDF
KANGAROO_EXPORT
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix = true);

// for smart SDF
KANGAROO_EXPORT
void RaycastSdf(Image<float> depth, Image<float4> norm, Image<float> img,
                const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
                const BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> grayVol,
                const Mat<float,3,4> T_wc, ImageIntrinsics K, float near, float far,
                float trunc_dist, bool subpix );
}
