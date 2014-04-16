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

}
