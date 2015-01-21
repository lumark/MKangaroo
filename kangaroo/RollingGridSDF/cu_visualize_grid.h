#pragma once

#include "kangaroo/platform.h"
#include "kangaroo/Mat.h"
#include "kangaroo/Image.h"
#include "kangaroo/ImageIntrinsics.h"
#include "BoundedVolumeGrid.h"
#include "kangaroo/MatUtils.h"
#include "kangaroo/launch_utils.h"
#include "SdfSmart.h"

namespace roo
{
KANGAROO_EXPORT
void VisualizeGrid(
    BoundedVolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol,
    BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage> colorVol );
}
