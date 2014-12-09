#pragma once

#include "kangaroo/platform.h"
#include "BoundedVolumeGrid.h"
#include "kangaroo/Sdf.h"
#include "kangaroo/GridSDF/SdfSmart.h"

namespace roo
{

// -----------------------------------------------------------------------------
KANGAROO_EXPORT
void SdfReset(VolumeGrid<SDF_t, roo::TargetDevice, roo::Manage> vol);

KANGAROO_EXPORT
void SdfReset(VolumeGrid<SDF_t_Smart,roo::TargetDevice, roo::Manage> vol);

KANGAROO_EXPORT
void SdfReset(VolumeGrid<float, roo::TargetDevice, roo::Manage> vol);

KANGAROO_EXPORT
void SdfReset(BoundedVolumeGrid<roo::SDF_t,roo::TargetDevice, roo::Manage> vol);

KANGAROO_EXPORT
void SdfReset(BoundedVolumeGrid<roo::SDF_t_Smart,roo::TargetDevice, roo::Manage> vol);

KANGAROO_EXPORT
void SdfReset(BoundedVolumeGrid<float,roo::TargetDevice, roo::Manage> vol);

// grid sdf in host memory
void SdfReset(VolumeGrid<SDF_t, roo::TargetHost, roo::Manage> vol);

void SdfReset(VolumeGrid<float, roo::TargetHost, roo::Manage> vol);

void SdfReset(BoundedVolumeGrid<float,roo::TargetHost, roo::Manage> vol);

void SdfReset(BoundedVolumeGrid<roo::SDF_t,roo::TargetHost, roo::Manage> vol);

}
