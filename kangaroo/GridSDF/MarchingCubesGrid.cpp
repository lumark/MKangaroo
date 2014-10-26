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



//template<typename T, typename TColor>
//void SaveMeshGrid(std::string filename, BoundedVolumeGrid<T, TargetHost, Manage> vol,
//                  BoundedVolumeGrid<TColor, TargetHost, Manage> volColor )
//{
//  printf("in SaveMeshGrid/cpp..\n");

//  std::vector<aiVector3D> verts;
//  std::vector<aiVector3D> norms;
//  std::vector<aiFace> faces;
//  std::vector<aiColor4D> colors;

//  for(GLint iX = 0; iX < vol.Voxels().x-1; iX++) {
//    for(GLint iY = 0; iY < vol.Voxels().y-1; iY++) {
//      for(GLint iZ = 0; iZ < vol.Voxels().z-1; iZ++) {

//        const unsigned int nIndex =
//            int(floorf(iX/vol.m_nVolumeGridRes)) +
//            vol.m_nWholeGridRes* (int(floorf(iY/vol.m_nVolumeGridRes))+
//                                  vol.m_nWholeGridRes* int(floorf(iZ/vol.m_nVolumeGridRes)));

//        if(vol.CheckIfBasicSDFActive(nIndex) == true)
//        {
//          float val = vol(int(iX),int(iY),int(iZ)).val;
//          if(std::isfinite(val) && val !=0)
//          {
//            if(val > 0.0000001 || val > -0.0000001 )
//            {
//              roo::vMarchCubeGrid(vol, volColor, iX,iY,iZ, verts, norms, faces, colors);
//              printf("Finish march cube grid for (%d). val %f\n",int(iX), val);
//            }
//          }
//        }
//      }
//    }
//  }

//  printf("finish march cube grid..\n");

//  aiMesh* mesh = MeshFromLists(verts,norms,faces,colors);
//  SaveMeshGrid(filename, mesh);
//}




// Instantiate templates
//template void SaveMeshGrid<roo::SDF_t,float>(std::string, BoundedVolumeGrid<SDF_t,TargetHost, Manage> vol,
//BoundedVolumeGrid<float,TargetHost, Manage> volColor);

template void SaveMeshGrid<roo::SDF_t_Smart,float, Manage>(std::string,
BoundedVolumeGrid<SDF_t_Smart,TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor);
}
