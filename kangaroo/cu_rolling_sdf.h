#pragma once

#include "Mat.h"
#include "Image.h"
#include "ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "BoundedVolume.h"
#include "Sdf.h"

namespace roo
{
void SdfResetPartial(BoundedVolume<SDF_t> vol, float3 shift);

// move grid sdf around which enable it to fuse new pixels
void RollingGridSdfCuda(BoundedVolume<SDF_t> vol, float3 shift);

// get shift param for rolling sdf
void RollingDetShift(float3& positive_shift, float3& negative_shift, Image<float> depth,
                     const BoundedVolumeGrid<SDF_t,roo::TargetDevice, roo::Manage> vol,
                     const Mat<float,3,4> T_wc, ImageIntrinsics K);
}
