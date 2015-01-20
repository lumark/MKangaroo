#ifndef SAVEMESHGRID_H
#define SAVEMESHGRID_H

#include "MarchingCubesGrid.h"

namespace roo {

////////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
inline aiMesh* MeshFromLists(
    const std::vector<aiVector3D>&                    verts,
    const std::vector<aiVector3D>&                    norms,
    const std::vector<aiFace>&                        faces,
    const std::vector<aiColor4D>&                     colors
    )
{
  aiMesh* mesh = new aiMesh();
  mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;

  mesh->mNumVertices = verts.size();
  mesh->mVertices = new aiVector3D[verts.size()];
  for(unsigned int i=0; i < verts.size(); ++i) {
    mesh->mVertices[i] = verts[i];
  }

  if(norms.size() == verts.size()) {
    mesh->mNormals = new aiVector3D[norms.size()];
    for(unsigned int i=0; i < norms.size(); ++i) {
      mesh->mNormals[i] = norms[i];
    }
  }else{
    mesh->mNormals = 0;
  }

  mesh->mNumFaces = faces.size();
  mesh->mFaces = new aiFace[faces.size()];
  for(unsigned int i=0; i < faces.size(); ++i) {
    mesh->mFaces[i] = faces[i];
  }

  if( colors.size() == verts.size()) {
    mesh->mColors[0] = new aiColor4D[colors.size()];
    for(unsigned int i=0; i < colors.size(); ++i) {
      mesh->mColors[0][i] = colors[i];
    }
  }

  std::cout<<"[MeshFromLists] Finished. verts num "<<verts.size()<<", norms num: "<<norms.size()<<
             ", faces num "<<faces.size()<<", color num: "<< colors.size()<<std::endl;

  return mesh;
}

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

  for(int i=0;i!=vol.m_nGridRes_w;i++)
  {
    for(int j=0;j!=vol.m_nGridRes_h;j++)
    {
      for(int k=0;k!=vol.m_nGridRes_d;k++)
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

  return MeshFromLists(verts,norms,faces,colors);
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

////////////////////////////////////////////////////////////////////////////////
// here also support .obj file (strongly suuggest use .obj instead of using .ply).
KANGAROO_EXPORT
inline bool SaveMeshGridToFile(
    std::string                                       sFilename,
    aiMesh*                                           pMesh,
    std::string                                       sFormat ="ply")
{
  // Create root node which indexes first mesh
  aiNode* root = new aiNode();
  root->mNumMeshes = 1;
  root->mMeshes = new unsigned int[root->mNumMeshes];
  root->mMeshes[0] = 0;
  root->mName = "root";

  aiMaterial* material = new aiMaterial();

  // Create scene to contain root node and mesh
  aiScene scene;
  scene.mRootNode = root;
  scene.mNumMeshes = 1;
  scene.mMeshes = new aiMesh*[scene.mNumMeshes];
  scene.mMeshes[0] = pMesh;
  scene.mNumMaterials = 1;
  scene.mMaterials = new aiMaterial*[scene.mNumMaterials];
  scene.mMaterials[0] = material;

  sFilename = sFilename + "." + sFormat;
  aiReturn res = aiExportScene(&scene, sFormat.c_str(), sFilename.c_str(), 0);
  if(res == 0)
  {
    std::cout << "[SaveMeshGridToFile] Mesh export success. File Name "<< sFilename <<std::endl;
    return true;
  }
  else
  {
    std::cerr << "[SaveMeshGridToFile] Mesh export fail." << std::endl;
    return false;
  }
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
    BoundedVolumeGrid<T,TargetDevice,Manage>&         vol,
    BoundedVolumeGrid<TColor,TargetDevice,Manage>&    volColor )
{
  // copy data from the device memory to the host memory
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(vol.m_w, vol.m_h, vol.m_d, vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  roo::BoundedVolumeGrid<TColor,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(volColor.m_w, volColor.m_h, volColor.m_d,
                 volColor.m_nVolumeGridRes,volColor.m_bbox);

  hvolcolor.CopyAndInitFrom(volColor);

  SaveMeshGrid<T,TColor, Manage>(filename, hvol, hvolcolor);
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
  for(unsigned int i=0;i!=hVol.m_nGridRes_w;i++)
  {
    for(unsigned int j=0;j!=hVol.m_nGridRes_h;j++)
    {
      for(unsigned int k=0;k!=hVol.m_nGridRes_d;k++)
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

  aiMesh* mesh = MeshFromLists(ObjMesh.verts,ObjMesh.norms,
                               ObjMesh.faces,ObjMesh.colors);

  SaveMeshGridToFile(filename, mesh, "obj");
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
  for(int x=0; x!=vol.m_nVolumeGridRes; x++)
  {
    for(int y=0; y!=vol.m_nVolumeGridRes; y++)
    {
      for(int z=0; z!=vol.m_nVolumeGridRes; z++)
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
