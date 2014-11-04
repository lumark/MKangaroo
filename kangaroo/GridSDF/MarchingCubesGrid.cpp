// Marching cubes ASSIMP exporter based on Marching Cubes Example Program
// by Cory Bloyd with additional source from Paul Bourke (public domain)
// http://paulbourke.net/geometry/polygonise/
//
// Marching Cubes Example Program
// by Cory Bloyd (corysama@yahoo.com)
//
// A simple, portable and complete implementation of the Marching Cubes
// and Marching Tetrahedrons algorithms in a single source file.
// There are many ways that this code could be made faster, but the
// intent is for the code to be easy to understand.
//
// For a description of the algorithm go to
// http://astronomy.swin.edu.au/pbourke/modelling/polygonise/
//
// This code is public domain.
//

#include "stdio.h"
#include "math.h"

#include <kangaroo/Sdf.h>
#include <kangaroo/GridSDF/SdfSmart.h>
#include <kangaroo/GridSDF/MarchingCubesGrid.h>

namespace roo
{
// Instantiate templates
//template void SaveMeshGrid<roo::SDF_t,float>(std::string, BoundedVolumeGrid<SDF_t,TargetHost, Manage> vol,
//BoundedVolumeGrid<float,TargetHost, Manage> volColor);

template void SaveMeshGrid<roo::SDF_t_Smart,float, Manage>(std::string,
BoundedVolumeGrid<SDF_t_Smart,TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor);

}
