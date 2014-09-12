#pragma once

#include "BoundedVolumeGrid.h"
#include "Kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"

namespace roo
{

// -----------------------------------------------------------------------------
void SdfReset(VolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol);
void SdfReset(VolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol);
void SdfReset(VolumeGrid<float, roo::TargetDevice, roo::Manage> vol);

void SdfReset(BoundedVolumeGrid<roo::SDF_t,roo::TargetDevice, roo::Manage> vol);
void SdfReset(BoundedVolumeGrid<roo::SDF_t_Smart,roo::TargetDevice, roo::Manage> vol);
void SdfReset(BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> vol);

}
