#pragma once

#include <kangaroo/Sdf.h>
#include <kangaroo/extra/SaveGIL.h>
#include "MarchingCubesGrid.h"
#include "SaveRollingGridSDF.h"
#include "SavePPMGrid.h"
#include <boost/ptr_container/ptr_vector.hpp>

KANGAROO_EXPORT
void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage>& vol);

KANGAROO_EXPORT
void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t_Smart, roo::TargetDevice, roo::Manage>& vol);

KANGAROO_EXPORT
void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t, roo::TargetDevice, roo::Manage>& vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>& GreyVol,
    std::string                                                    sFileName = "mesh");

KANGAROO_EXPORT
void SaveMeshlabGrid(
    roo::BoundedVolumeGrid<roo::SDF_t_Smart, roo::TargetDevice, roo::Manage>& vol,
    roo::BoundedVolumeGrid<float, roo::TargetDevice, roo::Manage>&            GrayVol,
    std::string                                                               sFileName = "mesh");

KANGAROO_EXPORT
// generate a single mesh from several PPMs
void SaveMeshlabFromPXMs(
    std::string                 sDirName,
    std::string                 sBBFileHead,
    int3                        VolRes,
    int                         nGridRes,
    std::vector<std::string>    vfilename,
    std::string                 sFinalMeshFileName);
