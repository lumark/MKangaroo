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

#include "Sdf.h"
#include "MarchingCubesGrid.h"
#include "MarchingCubesTables.h"
#include "MarchingCubes.h"

#include <assimp/cexport.h>
#include <assimp/scene.h>

namespace roo
{

//vMarchCube performs the Marching Cubes algorithm on a single cube
/// USER SHOULD MAKE SURE VOXEL EXIST!
template<typename T, typename TColor>
void vMarchCubeGrid(
    BoundedVolumeGrid<T,roo::TargetHost, Manage>&       vol,
    BoundedVolumeGrid<TColor,roo::TargetHost, Manage>&  volColor,
    int x, int y, int z,
    std::vector<aiVector3D>& verts,
    std::vector<aiVector3D>& norms,
    std::vector<aiFace>& faces,
    std::vector<aiColor4D>& colors,
    float fTargetValue = 0.0f
    ) {
  const float3 p = vol.VoxelPositionInUnits(x,y,z);
  const float3 fScale = vol.VoxelSizeUnits();

  //Make a local copy of the values at the cube's corners
  float afCubeValue[8];
  for(int iVertex = 0; iVertex < 8; iVertex++)
  {
    if(vol.CheckIfVoxelExist(int(x+a2fVertexOffset[iVertex][0]), int(y+a2fVertexOffset[iVertex][1]), int(z+a2fVertexOffset[iVertex][2])) == true)
    {
      afCubeValue[iVertex] = vol(int(x+a2fVertexOffset[iVertex][0]), int(y+a2fVertexOffset[iVertex][1]), int(z+a2fVertexOffset[iVertex][2]));
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
      norms.push_back(aiVector3D(asEdgeNorm[iVertex].x,   asEdgeNorm[iVertex].y,   asEdgeNorm[iVertex].z) );

      if(volColor.IsValid()) {
        const TColor c = volColor.GetUnitsTrilinearClamped(asEdgeVertex[iVertex]);
        float3 sColor = roo::ConvertPixel<float3,TColor>(c);
        colors.push_back(aiColor4D(sColor.x, sColor.y, sColor.z, 1.0f));
      }

    }

    faces.push_back(face);
  }
}




//vMarchCube performs the Marching Cubes algorithm on a single cube
/// USER SHOULD MAKE SURE VOXEL EXIST!
template<typename T, typename TColor>
void vMarchCubeGridNOTexture(
    BoundedVolumeGrid<T,roo::TargetHost, Manage>&       vol,
    int x, int y, int z,
    std::vector<aiVector3D>& verts,
    std::vector<aiVector3D>& norms,
    std::vector<aiFace>& faces,
    std::vector<aiColor4D>& colors,
    float fTargetValue = 0.0f
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




void SaveMeshGrid(std::string filename, aiMesh* mesh)
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
  scene.mMeshes[0] = mesh;
  scene.mNumMaterials = 1;
  scene.mMaterials = new aiMaterial*[scene.mNumMaterials];
  scene.mMaterials[0] = material;

  aiReturn res = aiExportScene(&scene, "ply", (filename + ".ply").c_str(), 0);
  std::cout << "[Kangaroo/SaveMeshGrid] Finish save mesh. Mesh export result: " << res << std::endl;
}



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





// now do it for each grid instead of each voxel
template<typename T, typename TColor>
void SaveMeshGridSingle(BoundedVolumeGrid<T, TargetHost, Manage>&      vol,
                        BoundedVolumeGrid<TColor, TargetHost, Manage>& volColor,
                        int i,int j,int k,
                        std::vector<aiVector3D>&                       verts,
                        std::vector<aiVector3D>&                       norms,
                        std::vector<aiFace>&                           faces,
                        std::vector<aiColor4D>&                        colors)
{
  for(GLint x=0;x!=vol.m_nVolumeGridRes;x++)
  {
    for(GLint y=0;y!=vol.m_nVolumeGridRes;y++)
    {
      for(GLint z=0;z!=vol.m_nVolumeGridRes;z++)
      {
        //        if(vol.CheckIfVoxelExist(i*vol.m_nVolumeGridRes + x,
        //                                 j*vol.m_nVolumeGridRes + y,
        //                                 k*vol.m_nVolumeGridRes + z) == true)
        //        {
        //           get voxel index for each grid.
        roo::vMarchCubeGrid(vol, volColor,
                            i*vol.m_nVolumeGridRes + x,
                            j*vol.m_nVolumeGridRes + y,
                            k*vol.m_nVolumeGridRes + z,
                            verts, norms, faces, colors);
        //        }
      }
    }
  }
}



//// now do it for each grid instead of each voxel
//template<typename T,typename TColor>
//void SaveMeshGridSingle(BoundedVolumeGrid<T, TargetHost, Manage>& vol,
//                        int i,int j,int k,
//                        std::vector<aiVector3D>& verts,
//                        std::vector<aiVector3D>& norms,
//                        std::vector<aiFace>& faces,
//                        std::vector<aiColor4D>& colors)
//{
//  for(GLint x=0;x!=vol.m_nVolumeGridRes;x++)
//  {
//    for(GLint y=0;y!=vol.m_nVolumeGridRes;y++)
//    {
//      for(GLint z=0;z!=vol.m_nVolumeGridRes;z++)
//      {
//        if(vol.CheckIfVoxelExist(i*vol.m_nVolumeGridRes + x,
//                                 j*vol.m_nVolumeGridRes + y,
//                                 k*vol.m_nVolumeGridRes + z) == true)
//        {
//          // get voxel index for each grid.
//          roo::vMarchCubeGrid(vol,
//                              i*vol.m_nVolumeGridRes + x,
//                              j*vol.m_nVolumeGridRes + y,
//                              k*vol.m_nVolumeGridRes + z,
//                              verts, norms, faces, colors);
//        }
//      }
//    }
//  }
//}


// now do it for each grid instead of each voxel
template<typename T, typename TColor>
void SaveMeshGrid(std::string                                   filename,
                  BoundedVolumeGrid<T, TargetHost, Manage>      vol,
                  BoundedVolumeGrid<TColor, TargetHost, Manage> volColor )
{
  printf("[SaveMeshGrid] In SaveMeshGrid/cpp..\n");

  std::vector<aiVector3D> verts;
  std::vector<aiVector3D> norms;
  std::vector<aiFace> faces;
  std::vector<aiColor4D> colors;

  // scan each grid..
  for(int i=0;i!=vol.m_nWholeGridRes_w;i++)
  {
    for(int j=0;j!=vol.m_nWholeGridRes_h;j++)
    {
      for(int k=0;k!=vol.m_nWholeGridRes_d;k++)
      {
        if(vol.CheckIfBasicSDFActive(vol.GetLocalIndex(i,j,k)) == true)
        {
          SaveMeshGridSingle(vol,volColor,i,j,k,verts, norms, faces, colors);
          printf("[SaveMeshGrid] Finish march cube grid for (%d,%d,%d).\n", i,j,k);
        }
      }
    }
  }

  printf("[SaveMeshGrid] Finish march cube grid Sepreate..\n");

  aiMesh* mesh = MeshFromLists(verts,norms,faces,colors);
  SaveMeshGrid(filename, mesh);
}


///////////////////////////////////////////////////////////////////////////////
///                 Save Single Mesh from Several BBVolumes                 ///
///////////////////////////////////////////////////////////////////////////////



// get global index and local index from file name
bool GetIndexFromFileName(std::string sFileName, int3& GlobalIndex, int3& LocalIndex )
{
  std::vector<int> vIndex;
  std::string sTempStr = sFileName;

  for(unsigned int i=0;i!=sTempStr.size();i++)
  {
    if(sTempStr.substr(i,1) == "-")
    {
      vIndex.push_back(i);
    }
  }

  // now get global index
  if(vIndex.size()!=7)
  {
    //    std::cerr<<"[GetIndexFromFileName] Skip file "<< sFileName<<std::endl;
    return false;
  }
  else
  {
    GlobalIndex.x = std::stoi(sFileName.substr(vIndex[1]+1, vIndex[2]-vIndex[1]-1));
    GlobalIndex.y = std::stoi(sFileName.substr(vIndex[2]+1, vIndex[3]-vIndex[2]-1));
    GlobalIndex.z = std::stoi(sFileName.substr(vIndex[3]+1, vIndex[4]-vIndex[3]-1));

    LocalIndex.x = std::stoi(sFileName.substr(vIndex[4]+1, vIndex[5]-vIndex[4]-1));
    LocalIndex.y = std::stoi(sFileName.substr(vIndex[5]+1, vIndex[6]-vIndex[5]-1));
    LocalIndex.z = std::stoi(sFileName.substr(vIndex[6]+1, sFileName.size() - vIndex[6]-1));

    //    printf("[GetIndexFromFileName] Global index %d,%d,%d; local index %d,%d,%d. \n",
    //           GlobalIndex.x, GlobalIndex.y, GlobalIndex.z,
    //           LocalIndex.x, LocalIndex.y, LocalIndex.z);
    return true;
  }
}



std::vector<SingleVolume> GetFilesNeedSaving(std::vector<std::string>& vfilename)
{
  std::vector<SingleVolume>  vVolumes;

  for(unsigned int i=0;i!=vfilename.size();i++)
  {
    std::string sFileName = vfilename[i];

    // get index from file name
    int3 GlobalIndex, LocalIndex;
    if(GetIndexFromFileName(sFileName, GlobalIndex, LocalIndex)==true)
    {
      bool bFlag = false;
      for(unsigned int i=0;i!=vVolumes.size();i++)
      {
        if(vVolumes[i].GlobalIndex.x ==GlobalIndex.x &&
           vVolumes[i].GlobalIndex.y == GlobalIndex.y &&
           vVolumes[i].GlobalIndex.z == GlobalIndex.z)
        {
          vVolumes[i].vLocalIndex.push_back(LocalIndex);
          vVolumes[i].vFileName.push_back(sFileName);
          bFlag=true;
        }
      }

      if(bFlag==false)
      {
        SingleVolume mSingVolume;
        mSingVolume.GlobalIndex = GlobalIndex;
        mSingVolume.vLocalIndex.push_back(LocalIndex);
        mSingVolume.vFileName.push_back(sFileName);
        vVolumes.push_back(mSingVolume);
      }
    }
  }

  return vVolumes;
}




// ================================================================================
// Generate one single mesh from several ppm files.
bool GenMeshFromPPM(std::string               sDirName,
                    std::string               sBBFileName,
                    int3                      nVolRes,
                    int                       nGridRes,
                    std::vector<std::string>  vfilename,
                    std::string               sMeshFileName)
{
  printf("[Kangaroo/GenMeshFromPPM] Start.\n");

  // read all grid sdf and sort them into volumes
  std::vector<SingleVolume>  vVolumes = GetFilesNeedSaving(vfilename);

  if(vVolumes.size()==0)
  {
    printf("cannot find any files for generating mesh\n");
    return false;
  }

  // load each single volume into BBVolume.
  std::vector<aiVector3D> verts;
  std::vector<aiVector3D> norms;
  std::vector<aiFace>     faces;
  std::vector<aiColor4D>  colors;

  // load mesh configure
  // to load it from disk, we need to use host volume
  roo::BoundingBox BBox;

  roo::BoundedVolumeGrid<roo::SDF_t,roo::TargetHost,roo::Manage> hvol;
  hvol.init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.init(1,1,1, nGridRes, BBox);

  // for each global volume
  int nNum = 0;
  for(unsigned int i=0;i!=vVolumes.size();i++)
  {
    // load bounding box
    std::string sBBFile = sDirName + sBBFileName+std::to_string(vVolumes[i].GlobalIndex.x) + "-" +
        std::to_string(vVolumes[i].GlobalIndex.y) + "-" + std::to_string(vVolumes[i].GlobalIndex.z);

    if(CheckIfBBfileExist(sBBFile) == true)
    {
      hvol.m_bbox      = LoadPXMBoundingBox(sBBFile);
      hvolcolor.m_bbox = hvol.m_bbox;

      // for each single local grid volume, load it
      for(unsigned int j=0;j!=vVolumes[i].vLocalIndex.size();j++)
      {
        int3 LocalIndex = vVolumes[i].vLocalIndex[j];
        int nIndex = hvol.GetLocalIndex(LocalIndex.x, LocalIndex.y, LocalIndex.z);
        std::string sFile = sDirName+vVolumes[i].vFileName[j];

        if(LoadPXMSingleGrid(sFile, hvol.m_GridVolumes[nIndex])==false)
        {
          printf("fatal error! load file fail.. exit\n");
          exit(-1);
        }
        else
        {
          if(hvol.CheckIfBasicSDFActive(nIndex) == true)
          {
            SaveMeshGridSingle(hvol, hvolcolor, LocalIndex.x, LocalIndex.y, LocalIndex.z,
                               verts, norms, faces, colors);
            nNum++;
          }
        }
      }

      // reset previous grid
      SdfReset(hvol);
      hvol.ResetAllGridVol();
    }
  }

  printf("[Kangaroo/GenMeshFromPPM] finish march cube for %d Grids.\n",nNum);

  aiMesh* mesh = MeshFromLists(verts,norms,faces,colors);
  SaveMeshGrid(sMeshFileName, mesh);

  return true;
}


// Instantiate templates
template void SaveMeshGrid<SDF_t,float>(std::string, BoundedVolumeGrid<SDF_t,TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor);

}
