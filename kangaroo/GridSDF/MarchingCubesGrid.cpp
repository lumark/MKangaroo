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
        if(vVolumes[i].GlobalIndex.x == GlobalIndex.x &&
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
// get max and min global index of current system
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
bool SaveMeshFromPPMs(
    std::string                sDirName,
    std::string                sBBFileHead,
    int3                       nVolRes,
    int                        nGridRes,
    std::vector<std::string>   vfilename,
    std::string                sMeshFileName)
{
  printf("\n---- [Kangaroo/SaveMeshFromPPMs] Start.\n");

  // read all grid sdf and sort them into volumes. vVolume index is global index
  std::vector<SingleVolume>  vVolumes = GetFilesNeedSaving(vfilename);

  if(vVolumes.size()<=0)
  {
    printf("Cannot find any files for generating mesh\n");
    return false;
  }

  // ---------------------------------------------------------------------------
  // Load each single volume into the BBVolume.
  std::vector<aiVector3D>   verts, norms;
  std::vector<aiFace>       faces;
  std::vector<aiColor4D>    colors;

  // Load mesh configure
  roo::BoundingBox BBox;

  // To load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<roo::SDF_t,roo::TargetHost,roo::Manage> hvol;
  hvol.Init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);

  roo::BoundedVolumeGrid<float,roo::TargetHost,roo::Manage> hvolcolor;
  hvolcolor.Init(1,1,1, nGridRes, BBox);

  // Get max and min global index
  int3 MaxGlobalIndex = make_int3(-999999999, -999999999, -999999999);
  int3 MinGlobalIndex = make_int3(999999999, 999999999, 999999999);
  GetMaxMinGlobalIndex(sDirName, sBBFileHead, vVolumes, MaxGlobalIndex, MinGlobalIndex);

  // ---------------------------------------------------------------------------
  // For each global volume we have, gen mesh with it
  int nNum = 0;

  for(unsigned int i=0; i!=vVolumes.size(); i++)
  {
    // load the corresponding bounding box
    std::string sBBFile =
        sDirName + sBBFileHead +
        std::to_string(vVolumes[i].GlobalIndex.x) + "-" +
        std::to_string(vVolumes[i].GlobalIndex.y) + "-" +
        std::to_string(vVolumes[i].GlobalIndex.z);

    std::cout<<"Load bb file "<<sBBFile<<std::endl;

    if( CheckIfBBfileExist(sBBFile) )
    {
      hvol.m_bbox      = LoadPXMBoundingBox(sBBFile);
      hvolcolor.m_bbox = hvol.m_bbox;

      // for each single local grid volume, load it
      for(unsigned int j=0; j!=vVolumes[i].vLocalIndex.size(); j++)
      {
        int3 CurLocalIndex = vVolumes[i].vLocalIndex[j];
        int3 CurGlobalIndex = vVolumes[i].GlobalIndex;

        int nRealIndex = hvol.ConvertLocalIndexToRealIndex(CurLocalIndex.x,
                                                           CurLocalIndex.y,
                                                           CurLocalIndex.z);

        std::string sPXMFile = sDirName + vVolumes[i].vFileName[j];

        if(LoadPXMSingleGrid(sPXMFile, hvol.m_GridVolumes[nRealIndex]) == false)
        {
          printf("[Kangaroo/GenMeshFromPPM] Error! load file fail.. exit\n");
          exit(-1);
        }
        else
        {
          if(hvol.CheckIfBasicSDFActive(nRealIndex) == true)
          {
            SaveMeshSingleGridGlobal( hvol, hvolcolor,
                                      CurLocalIndex, CurGlobalIndex,
                                      MaxGlobalIndex, MinGlobalIndex,
                                      verts, norms, faces, colors);

            nNum++;
          }
          else
          {
            std::cerr<<"[Kangaroo/GenMeshFromPPM] Error! mesh inactive!"<<std::endl;
            exit(-1);
          }
        }
      }
    }
    else
    {
      std::cerr<<"[Kangaroo/GenMeshFromPPM] Error! Fail loading bbox "<<sBBFile<<std::endl;
      exit(-1);
    }

    // reset all previous grid
    SdfReset(hvol);
    hvol.ResetAllGridVol();

    std::cout<<"Finish save vol "<<i<<";"<<std::endl;
  }

  std::cout<<"[Kangaroo/GenMeshFromPPM] Finish marching cube for " << nNum<< "Grids.\n";

  // ---------------------------------------------------------------------------
  // Save mesh from memory to hard disk
  aiMesh* mesh = MeshFromLists(verts, norms, faces, colors);
  return SaveMeshGridToFile(sMeshFileName, mesh, "obj");
}

template void SaveMeshGrid<roo::SDF_t_Smart, float, Manage>
(
std::string,
BoundedVolumeGrid<SDF_t_Smart, TargetHost, Manage> vol,
BoundedVolumeGrid<float,TargetHost, Manage> volColor
);

}
