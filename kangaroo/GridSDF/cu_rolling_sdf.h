#pragma once

#include "kangaroo/platform.h"
#include "Kangaroo/Mat.h"
#include "Kangaroo/Image.h"
#include "Kangaroo/ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "Kangaroo/BoundedVolume.h"
#include "Kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"

namespace roo
{
KANGAROO_EXPORT
void SdfResetPartial(BoundedVolume<SDF_t> vol, float3 shift);

// move grid sdf around which enable it to fuse new pixels
KANGAROO_EXPORT
void RollingGridSdfCuda(int* pNextInitSDFs, BoundedVolumeGrid<SDF_t> vol, int3 shift);
void RollingGridSdfCuda(int* pNextInitSDFs, BoundedVolumeGrid<SDF_t_Smart> vol, int3 shift);


// get shift param for rolling sdf
KANGAROO_EXPORT
void RollingDetShift(
    float3& positive_shift, float3& negative_shift, Image<float> depth,
    const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
    const Mat<float,3,4> T_wc, ImageIntrinsics K);

KANGAROO_EXPORT
void RollingDetShift(
    float3& positive_shift, float3& negative_shift, Image<float> depth,
    const BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
    const Mat<float,3,4> T_wc, ImageIntrinsics K);
}
