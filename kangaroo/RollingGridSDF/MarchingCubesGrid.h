// by lu.ma@colorado.edu

#pragma once

#include <sys/time.h>
#include <kangaroo/platform.h>
#include "BoundedVolumeGrid.h"
#include <kangaroo/MarchingCubesTables.h>
#include <assimp/cexport.h>
#include <assimp/scene.h>

namespace roo {

class MarchingCUBERst
{
public:
  std::vector<aiVector3D>   verts;
  std::vector<aiVector3D>   norms;
  std::vector<aiFace>       faces;
  std::vector<aiColor4D>    colors;
};

// fGetOffset finds the approximate point of intersection of the surface
// between two points with the values fValue1 and fValue2
KANGAROO_EXPORT
float fGetOffset(
    float                                             fValue1,
    float                                             fValue2,
    float                                             fValueDesired);

// vMarchCube performs the Marching Cubes algorithm on a single cube
// USER SHOULD MAKE SURE VOXEL EXIST!
KANGAROO_EXPORT
template<typename T, typename TColor>
void vMarchCubeGrid(
    BoundedVolumeGrid<T,roo::TargetHost, Manage>&       vol,
    BoundedVolumeGrid<TColor,roo::TargetHost, Manage>&  volColor,
    const float3&                                       p,
    const float3&                                       fScale,
    int x, int y, int z,
    std::vector<aiVector3D>&                            verts,
    std::vector<aiVector3D>&                            norms,
    std::vector<aiFace>&                                faces,
    std::vector<aiColor4D>&                             colors,
    float                                               fTargetValue = 0.0f
    )
{
  // Make a local copy of the values at the cube's corners
  float afCubeValue[8];
  for(int iVertex = 0; iVertex < 8; iVertex++)
  {
    if(vol.CheckIfVoxelExist(
         static_cast<int>(x + a2fVertexOffset[iVertex][0]),
         static_cast<int>(y + a2fVertexOffset[iVertex][1]),
         static_cast<int>(z + a2fVertexOffset[iVertex][2])))
    {
      afCubeValue[iVertex] =
          vol(static_cast<int>(x+a2fVertexOffset[iVertex][0]),
          static_cast<int>(y+a2fVertexOffset[iVertex][1]),
          static_cast<int>(z+a2fVertexOffset[iVertex][2]));
    }
    else
    {
      return;
    }

    if(!std::isfinite(afCubeValue[iVertex])) return;
  }

  // --------------------------------------------------------------------------
  //Find which vertices are inside of the surface and which are outside
  int iFlagIndex = 0;
  for(int iVertexTest = 0; iVertexTest < 8; iVertexTest++)
  {
    if(afCubeValue[iVertexTest] <= fTargetValue)
      iFlagIndex |= 1<<iVertexTest;
  }

  //Find which edges are intersected by the surface
  int iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];

  // If the cube is entirely inside or outside of the surface,
  // then there will be no intersections
  if(iEdgeFlags == 0) {
    return;
  }

  //Find the point of intersection of the surface with each edge
  //Then find the normal to the surface at those points
  float3 asEdgeVertex[12];
  float3 asEdgeNorm[12];

  for(int iEdge = 0; iEdge < 12; iEdge++)
  {
    //if there is an intersection on this edge
    if(iEdgeFlags & (1<<iEdge))
    {
      float fOffset = fGetOffset(afCubeValue[ a2iEdgeConnection[iEdge][0] ],
          afCubeValue[ a2iEdgeConnection[iEdge][1] ], fTargetValue);

      asEdgeVertex[iEdge] = make_float3(
            p.x + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][0]  +
          fOffset * a2fEdgeDirection[iEdge][0]) * fScale.x,
          p.y + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][1]  +
          fOffset * a2fEdgeDirection[iEdge][1]) * fScale.y,
          p.z + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][2]  +
          fOffset * a2fEdgeDirection[iEdge][2]) * fScale.z );

      const float3 deriv = vol.GetUnitsBackwardDiffDxDyDz( asEdgeVertex[iEdge] );
      asEdgeNorm[iEdge] = deriv / length(deriv);

      if( !std::isfinite(asEdgeNorm[iEdge].x) ||
          !std::isfinite(asEdgeNorm[iEdge].y) ||
          !std::isfinite(asEdgeNorm[iEdge].z) )
      {
        asEdgeNorm[iEdge] = make_float3(0,0,0);
      }
    }
  }

  // ----------------------------------------------------------------------------
  // Draw the triangles that were found.  There can be up to five per cube
  for(int iTriangle = 0; iTriangle < 5; iTriangle++)
  {
    if(a2iTriangleConnectionTable[iFlagIndex][3*iTriangle] < 0)
      break;

    aiFace face;
    face.mNumIndices = 3;
    face.mIndices = new unsigned int[face.mNumIndices];

    for(int iCorner = 0; iCorner < 3; iCorner++)
    {
      int iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];

      face.mIndices[iCorner] = verts.size();
      verts.push_back(aiVector3D(asEdgeVertex[iVertex].x,
                                 asEdgeVertex[iVertex].y,
                                 asEdgeVertex[iVertex].z) );

      //      printf("add (%f,%f,%f);",asEdgeVertex[iVertex].x, asEdgeVertex[iVertex].y, asEdgeVertex[iVertex].z);

      norms.push_back(aiVector3D(asEdgeNorm[iVertex].x,
                                 asEdgeNorm[iVertex].y,
                                 asEdgeNorm[iVertex].z) );

      if(volColor.IsValid()) {
        const TColor c = volColor.GetUnitsTrilinearClamped(asEdgeVertex[iVertex]);
        float3 sColor = roo::ConvertPixel<float3,TColor>(c);
        colors.push_back(aiColor4D(sColor.x, sColor.y, sColor.z, 1.0f));
      }
      else
      {
//        std::cerr<<"[vMarchCubeGrid] warning! skip saving color."<<std::endl;
      }
    }

    faces.push_back(face);
  }
}

// vMarchCube performs the Marching Cubes algorithm on a single cube
/// USER SHOULD MAKE SURE VOXEL EXIST!
KANGAROO_EXPORT
template<typename T, typename TColor>
void vMarchCubeGridNOTexture(
    BoundedVolumeGrid<T,roo::TargetHost, Manage>&       vol,
    int x, int y, int z,
    std::vector<aiVector3D>&                            verts,
    std::vector<aiVector3D>&                            norms,
    std::vector<aiFace>&                                faces,
    std::vector<aiColor4D>&                             colors,
    float                                               fTargetValue = 0.0f
    ) {
  const float3 p = vol.VoxelPositionInUnits(x,y,z);
  const float3 fScale = vol.VoxelSizeUnits();

  //Make a local copy of the values at the cube's corners
  float afCubeValue[8];
  for(int iVertex = 0; iVertex < 8; iVertex++)
  {
    if(vol.CheckIfVoxelExist(int(x+a2fVertexOffset[iVertex][0]),
                             int(y+a2fVertexOffset[iVertex][1]),
                             int(z+a2fVertexOffset[iVertex][2])) == true)
    {
      afCubeValue[iVertex] = vol(int(x+a2fVertexOffset[iVertex][0]),
          int(y+a2fVertexOffset[iVertex][1]),
          int(z+a2fVertexOffset[iVertex][2]));
    }

    if(!std::isfinite(afCubeValue[iVertex])) return;
  }

  //Find which vertices are inside of the surface and which are outside
  int iFlagIndex = 0;
  for(int iVertexTest = 0; iVertexTest < 8; iVertexTest++) {
    if(afCubeValue[iVertexTest] <= fTargetValue)
      iFlagIndex |= 1<<iVertexTest;
  }

  //Find which edges are intersected by the surface
  int iEdgeFlags = aiCubeEdgeFlags[iFlagIndex];

  //If the cube is entirely inside or outside of the surface, then there will be no intersections
  if(iEdgeFlags == 0) {
    return;
  }

  //Find the point of intersection of the surface with each edge
  //Then find the normal to the surface at those points
  float3 asEdgeVertex[12];
  float3 asEdgeNorm[12];

  for(int iEdge = 0; iEdge < 12; iEdge++)
  {
    //if there is an intersection on this edge
    if(iEdgeFlags & (1<<iEdge))
    {
      float fOffset = fGetOffset(afCubeValue[ a2iEdgeConnection[iEdge][0] ],
          afCubeValue[ a2iEdgeConnection[iEdge][1] ], fTargetValue);

      asEdgeVertex[iEdge] = make_float3(
            p.x + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][0]  +  fOffset * a2fEdgeDirection[iEdge][0]) * fScale.x,
          p.y + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][1]  +  fOffset * a2fEdgeDirection[iEdge][1]) * fScale.y,
          p.z + (a2fVertexOffset[ a2iEdgeConnection[iEdge][0] ][2]  +  fOffset * a2fEdgeDirection[iEdge][2]) * fScale.z
          );

      const float3 deriv = vol.GetUnitsBackwardDiffDxDyDz( asEdgeVertex[iEdge] );
      asEdgeNorm[iEdge] = deriv / length(deriv);

      if( !std::isfinite(asEdgeNorm[iEdge].x) || !std::isfinite(asEdgeNorm[iEdge].y) || !std::isfinite(asEdgeNorm[iEdge].z) ) {
        asEdgeNorm[iEdge] = make_float3(0,0,0);
      }
    }
  }

  //Draw the triangles that were found.  There can be up to five per cube
  for(int iTriangle = 0; iTriangle < 5; iTriangle++)
  {
    if(a2iTriangleConnectionTable[iFlagIndex][3*iTriangle] < 0)
      break;

    aiFace face;
    face.mNumIndices = 3;
    face.mIndices = new unsigned int[face.mNumIndices];

    for(int iCorner = 0; iCorner < 3; iCorner++)
    {
      int iVertex = a2iTriangleConnectionTable[iFlagIndex][3*iTriangle+iCorner];

      face.mIndices[iCorner] = verts.size();
      verts.push_back(aiVector3D(asEdgeVertex[iVertex].x, asEdgeVertex[iVertex].y, asEdgeVertex[iVertex].z) );
      norms.push_back(aiVector3D(asEdgeNorm[iVertex].x, asEdgeNorm[iVertex].y, asEdgeNorm[iVertex].z) );

    }

    faces.push_back(face);
  }
}

}
