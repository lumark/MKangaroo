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
#include <kangaroo/GridSDF/cu_sdffusion_grid.h>
#include <kangaroo/GridSDF/SdfSmart.h>
#include <kangaroo/GridSDF/SavePPMGrid.h>
#include <kangaroo/GridSDF/MarchingCubesGrid.h>

namespace roo
{
///////////////////////////////////////////////////////////////////////////////
///                 Save Single Mesh from Several BBVolumes                 ///
///////////////////////////////////////////////////////////////////////////////

// get global index and local index from file name
bool GetIndexFromFileName(
    std::string                sFileName,
    int3&                      GlobalIndex,
    int3&                      LocalIndex )
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
    return true;
  }
}

// ================================================================================
// get files need saving into mesh
std::vector<SingleVolume> GetFilesNeedSaving(
    std::vector<std::string>&  vfilename)
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
// get max and min global index that we works with
void GetMaxMinGlobalIndex(
    std::string                sDirName,
    std::string                sBBFileName,
    std::vector<SingleVolume>& rvVolumes,
    int3&                      rMaxGlobal,
    int3&                      rMinGlobal)
{
  for(unsigned int i=0;i!=rvVolumes.size();i++)
  {
    // load bounding box
    std::string sBBFile = sDirName + sBBFileName+std::to_string(rvVolumes[i].GlobalIndex.x) + "-" +
        std::to_string(rvVolumes[i].GlobalIndex.y) + "-" + std::to_string(rvVolumes[i].GlobalIndex.z);

    if(CheckIfBBfileExist(sBBFile) == true)
    {
      int3 CurGlobalIndex = rvVolumes[i].GlobalIndex;

      // for max
      if(CurGlobalIndex.x>rMaxGlobal.x)
      {
        rMaxGlobal.x = CurGlobalIndex.x;
      }

      if(CurGlobalIndex.y>rMaxGlobal.y)
      {
        rMaxGlobal.y = CurGlobalIndex.y;
      }

      if(CurGlobalIndex.z>rMaxGlobal.z)
      {
        rMaxGlobal.z = CurGlobalIndex.z;
      }

      // for min
      if(CurGlobalIndex.x<rMinGlobal.x)
      {
        rMinGlobal.x = CurGlobalIndex.x;
      }

      if(CurGlobalIndex.y<rMinGlobal.y)
      {
        rMinGlobal.y = CurGlobalIndex.y;
      }

      if(CurGlobalIndex.z<rMinGlobal.z)
      {
        rMinGlobal.z = CurGlobalIndex.z;
      }
    }
  }

  std::cout<<"[Kangaroo/MarchingCubesGrid] Generating mesh in max global index: ("
          <<rMaxGlobal.x<<","<<rMaxGlobal.y<<","<<rMaxGlobal.z<<")"
         <<"; min global index: ("<<rMinGlobal.x<<","<<rMinGlobal.y<<","<<rMinGlobal.z<<")"<<std::endl;
}

// ================================================================================
// Generate one single mesh from several ppm files.
bool GenMeshFromPPM(
    std::string                sDirName,
    std::string                sBBFileName,
    int3                       nVolRes,
    int                        nGridRes,
    std::vector<std::string>   vfilename,
    std::string                sMeshFileName)
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
  std::vector<aiVector3D>   verts;
  std::vector<aiVector3D>   norms;
  std::vector<aiFace>       faces;
  std::vector<aiColor4D>    colors;

  // load mesh configure
  // to load it from disk, we need to use host volume
  roo::BoundingBox BBox;

  roo::BoundedVolumeGrid<roo::SDF_t,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(1,1,1, nGridRes, BBox);

  // get max and min global index
  int3 MaxGlobalIndex = make_int3(-999999999, -999999999, -999999999);
  int3 MinGlobalIndex = make_int3(999999999, 999999999, 999999999);
  GetMaxMinGlobalIndex(sDirName, sBBFileName, vVolumes, MaxGlobalIndex, MinGlobalIndex);

  // for each global volume
  int nNum = 0;
  int nNumSkip = 0;

  for(unsigned int i=0; i!=vVolumes.size(); i++)
  {
    // load the corresponding bounding box
    std::string sBBFile = sDirName + sBBFileName+std::to_string(vVolumes[i].GlobalIndex.x) + "-" +
        std::to_string(vVolumes[i].GlobalIndex.y) + "-" + std::to_string(vVolumes[i].GlobalIndex.z);

    if(CheckIfBBfileExist(sBBFile) == true)
    {
      hvol.m_bbox      = LoadPXMBoundingBox(sBBFile);
      hvolcolor.m_bbox = hvol.m_bbox;

      // for each single local grid volume, load it
      for(unsigned int j=0;j!=vVolumes[i].vLocalIndex.size();j++)
      {
        int3 CurLocalIndex = vVolumes[i].vLocalIndex[j];
        int3 CurGlobalIndex = vVolumes[i].GlobalIndex;

        int nRealIndex = hvol.ConvertLocalIndexToRealIndex(CurLocalIndex.x,
                                                           CurLocalIndex.y,
                                                           CurLocalIndex.z);

        std::string sFile = sDirName+vVolumes[i].vFileName[j];

        if(LoadPXMSingleGrid(sFile, hvol.m_GridVolumes[nRealIndex])==false)
        {
          printf("[Kangaroo/GenMeshFromPPM] Error! load file fail.. exit\n");
          exit(-1);
        }
        else
        {
          if(hvol.CheckIfBasicSDFActive(nRealIndex) == true)
          {
            SaveMeshSingleGridGlobal(
                  hvol, hvolcolor, CurLocalIndex, CurGlobalIndex,
                  MaxGlobalIndex, MinGlobalIndex, verts, norms, faces, colors);

            //            GenMeshSingleGrid(hvol, hvolcolor, CurLocalIndex,
            //                              verts, norms, faces, colors);
            nNum++;
          }
          else
          {
            std::cerr<<"[Kangaroo/GenMeshFromPPM] Warnning! skip saving mesh."<<std::endl;
            nNumSkip++;
          }
        }
      }
    }
    else
    {
      std::cerr<<"[Kangaroo/GenMeshFromPPM] Warnning! cannot load bbox "<<sBBFile<<
                 ". Skip!"<<std::endl;

      nNumSkip = nNumSkip +vVolumes[i].vLocalIndex.size();
    }

    // reset previous grid
    SdfReset(hvol);
    hvol.ResetAllGridVol();
  }

  printf("[Kangaroo/GenMeshFromPPM] Finish marching cube for %d Grids. Skip %d\n",nNum, nNumSkip);

  // save the mesh in memory to mesh in hard disk
  aiMesh* mesh = MeshFromLists(verts, norms, faces, colors);
  return SaveMeshGridToFile(sMeshFileName, mesh);
}

template void SaveMeshGrid<roo::SDF_t_Smart, float, Manage>
(
std::string,
BoundedVolumeGrid<SDF_t_Smart, TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor
);

}
