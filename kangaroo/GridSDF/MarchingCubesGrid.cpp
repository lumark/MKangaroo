#include <kangaroo/Sdf.h>
#include <kangaroo/GridSDF/cu_sdffusion_grid.h>
#include <kangaroo/GridSDF/SdfSmart.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>
#include <kangaroo/GridSDF/LoadPPMGrid.h>
#include <kangaroo/GridSDF/MarchingCubesGrid.h>

namespace roo
{

template void SaveMeshGrid<roo::SDF_t_Smart, float, Manage>
(
std::string,
BoundedVolumeGrid<SDF_t_Smart, TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor
);

}
