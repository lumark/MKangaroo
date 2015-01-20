#include "SaveMeshGrid.h"

namespace roo
{

template void SaveMeshGrid<roo::SDF_t_Smart, float, Manage>
(
std::string,
BoundedVolumeGrid<SDF_t_Smart, TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor
);

}
