// by lu.ma@colorado.edu

#ifndef SAVEMESHGRID_H
#define SAVEMESHGRID_H

#include "MarchingCubesGrid.h"
#include "PLYIO.h"
#include <kangaroo/MarchingCubes.h>

namespace roo {

aiMesh* MeshFromListsVector(
    const std::vector<aiVector3D>&                    verts,
    const std::vector<aiVector3D>&                    norms,
    const std::vector<aiFace>&                        faces,
    const std::vector<aiColor4D>&                     colors);

// notice ply format is the only one which support color mesh inside the .ply file.
KANGAROO_EXPORT
bool SaveMeshGridToFileAssimp(
    std::string                                       sFilename,
    aiMesh*                                           pMesh,
    std::string                                       sFormat ="ply");

// notice ply format is the only one which support color mesh inside the .ply file.
KANGAROO_EXPORT
bool SaveMeshGridToFile(
    std::string                                       sFilename,
    std::vector<aiVector3D>&                          verts,
    std::vector<aiVector3D>&                          norms,
    std::vector<aiFace>&                              faces,
    std::vector<aiColor4D>&                           colors,
    std::string                                       sFormat ="ply");

////////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
aiMesh* GetMeshGrid(
    BoundedVolumeGrid<T, TargetHost, Manage>          vol,
    BoundedVolumeGrid<TColor, TargetHost, Manage>     volColor )
{
  std::vector<aiVector3D>   verts;
  std::vector<aiVector3D>   norms;
  std::vector<aiFace>       faces;
  std::vector<aiColor4D>    colors;

  // scan each grid..
  int nNumSkip =0;
  int nNumSave =0;

  for(int i=0;i!=vol.m_nGridNum_w;i++)
  {
    for(int j=0;j!=vol.m_nGridNum_h;j++)
    {
      for(int k=0;k!=vol.m_nGridNum_d;k++)
      {
        if(vol.CheckIfBasicSDFActive(vol.ConvertLocalIndexToRealIndex(i,j,k)) == true)
        {
          GenMeshSingleGrid(vol,volColor,make_int3(i,j,k),verts, norms, faces, colors);
          nNumSave++;
        }
        else
        {
          nNumSkip++;
        }
      }
    }
  }

  return MeshFromListsVector(verts,norms,faces,colors);
}

KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
aiMesh* GetMeshGrid(
    BoundedVolumeGrid<T,TargetDevice,Manage>&         vol,
    BoundedVolumeGrid<TColor,TargetDevice,Manage>&    volColor )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(volColor.m_w, volColor.m_h, volColor.m_d,
                 volColor.m_nVolumeGridRes,volColor.m_bbox);

  hvolcolor.CopyAndInitFrom(volColor);

  // save
  return GetMeshGrid<T,TColor, Manage>(hvol, hvolcolor);
}


//////////////////////////////////////////
/// Save Mesh
//////////////////////////////////////////
KANGAROO_EXPORT
template<typename T, typename Manage>
void SaveMeshGrid(
    std::string                                       filename,
    BoundedVolumeGrid<T,TargetDevice,Manage>&         vol )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes, vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(1,1,1, vol.m_nVolumeGridRes,vol.m_bbox );

  SaveMeshGrid<T,float>(filename, hvol, hvolcolor);
}

KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
void SaveMeshGrid(
    std::string                                       filename,
    BoundedVolumeGrid<T,TargetDevice,Manage>&         DVol,
    BoundedVolumeGrid<TColor,TargetDevice,Manage>&    DVolColor )
{
  // copy data from the device memory to the host memory
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hVol;
  hVol.Init(DVol.m_w, DVol.m_h, DVol.m_d, DVol.m_nVolumeGridRes,DVol.m_bbox);
  hVol.CopyAndInitFrom(DVol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hVolColor;
  hVolColor.Init(DVolColor.m_w, DVolColor.m_h, DVolColor.m_d,
                 DVolColor.m_nVolumeGridRes, DVolColor.m_bbox);
  hVolColor.CopyAndInitFrom(DVolColor);

  SaveMeshGrid<T,TColor, Manage>(filename, hVol, hVolColor);
}

KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
void SaveMeshGrid(
    std::string                                       filename,
    BoundedVolumeGrid<T, TargetHost, Manage>          hVol,
    BoundedVolumeGrid<TColor, TargetHost, Manage>     hVolColor )
{
  MarchingCUBERst ObjMesh;
  int nNumSkip =0; int nNumSave =0;

  // for each grid in the whole volume
  for(unsigned int i=0;i!=hVol.m_nGridNum_w;i++)
  {
    for(unsigned int j=0;j!=hVol.m_nGridNum_h;j++)
    {
      for(unsigned int k=0;k!=hVol.m_nGridNum_d;k++)
      {
        if(hVol.CheckIfBasicSDFActive(hVol.ConvertLocalIndexToRealIndex(i,j,k)))
        {
          int3 CurLocalIndex = make_int3(i,j,k);

          GenMeshSingleGrid(hVol, hVolColor, CurLocalIndex, ObjMesh.verts,
                            ObjMesh.norms, ObjMesh.faces, ObjMesh.colors);

          std::cout<<"Finish save grid "<<hVol.ConvertLocalIndexToRealIndex(i,j,k)<<
                     "("<<i<<","<<j<<","<<k<<")"<<"; vertes num: "<<ObjMesh.verts.size()<<
                     "; norms num: "<<ObjMesh.norms.size()<<"; faces num: "<<ObjMesh.faces.size()<<
                     "; colors num: "<<ObjMesh.colors.size()<<std::endl;
          nNumSave++;
        }
        else
        {
          nNumSkip++;
        }
      }
    }
  }

  aiMesh* mesh = MeshFromListsVector(ObjMesh.verts,ObjMesh.norms,
                                     ObjMesh.faces,ObjMesh.colors);

  SaveMeshGridToFileAssimp(filename, mesh, "obj");
}

// now do it for each grid instead of each voxel
KANGAROO_EXPORT
template<typename T, typename TColor>
void GenMeshSingleGrid(
    BoundedVolumeGrid<T, TargetHost, Manage>&         vol,
    BoundedVolumeGrid<TColor, TargetHost, Manage>&    volColor,
    int3                                              CurLocalIndex,
    std::vector<aiVector3D>&                          verts,
    std::vector<aiVector3D>&                          norms,
    std::vector<aiFace>&                              faces,
    std::vector<aiColor4D>&                           colors)
{
  // for each voxel in the grid
  for(unsigned int x=0; x!=vol.m_nVolumeGridRes; x++)
  {
    for(unsigned int y=0; y!=vol.m_nVolumeGridRes; y++)
    {
      for(unsigned int z=0; z!=vol.m_nVolumeGridRes; z++)
      {
        int3 Index = make_int3(
              CurLocalIndex.x * static_cast<int>(vol.m_nVolumeGridRes) + x,
              CurLocalIndex.y * static_cast<int>(vol.m_nVolumeGridRes) + y,
              CurLocalIndex.z * static_cast<int>(vol.m_nVolumeGridRes) + z);

        // check if the voxel in the grid sdf is active
        if(vol.CheckIfVoxelExist( Index.x, Index.y, Index.z ))
        {
          const float3 p = vol.VoxelPositionInUnits(Index.x, Index.y, Index.z);
          const float3 fScale = vol.VoxelSizeUnits();

          roo::vMarchCubeGrid( vol, volColor, p, fScale,
                               Index.x, Index.y, Index.z,
                               verts, norms, faces, colors);
        }
      }
    }
  }
}

KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
void SaveMeshGridSepreate(
    std::string                                       filename,
    const BoundedVolumeGrid<T,TargetHost,Manage>      vol,
    const BoundedVolumeGrid<TColor,TargetHost,Manage> volColor );

KANGAROO_EXPORT
template<typename T, typename Manage>
void SaveMeshGridSepreate(
    std::string                                       filename,
    BoundedVolumeGrid<T,TargetDevice,Manage>&         vol )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes, vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(1,1,1, vol.m_nVolumeGridRes,vol.m_bbox );

  SaveMeshGridSepreate<T,float>(filename, hvol, hvolcolor);
}

KANGAROO_EXPORT
template<typename T, typename TColor, typename Manage>
void SaveMeshGridSepreate(
    std::string                                       filename,
    BoundedVolumeGrid<T,TargetDevice,Manage>&         vol,
    BoundedVolumeGrid<TColor,TargetDevice,Manage>&    volColor )
{
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(volColor.m_w, volColor.m_h, volColor.m_d,
                 volColor.m_nVolumeGridRes,volColor.m_bbox);

  hvolcolor.CopyAndInitFrom(volColor);

  SaveMeshGridSepreate<T,TColor>(filename, hvol, hvolcolor);
}

}
#endif // SAVEMESHGRID_H
