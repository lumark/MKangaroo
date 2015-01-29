#include "SaveRollingGridSDF.h"

namespace roo {

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
    if(sTempStr.substr(i,1) == "#")
    {
      vIndex.push_back(i);
    }
  }

  // now get global index
  if(vIndex.size()!=6)
  {
    std::cerr<<"[GetIndexFromFileName] Error! Index size is "<<vIndex.size()<<", req: 6;\n";
    return false;
  }
  else
  {
    GlobalIndex.x = std::stoi(sFileName.substr(vIndex[0]+1, vIndex[1]-vIndex[0]-1));
    GlobalIndex.y = std::stoi(sFileName.substr(vIndex[1]+1, vIndex[2]-vIndex[1]-1));
    GlobalIndex.z = std::stoi(sFileName.substr(vIndex[2]+1, vIndex[3]-vIndex[2]-1));

    LocalIndex.x = std::stoi(sFileName.substr(vIndex[3]+1, vIndex[4]-vIndex[3]-1));
    LocalIndex.y = std::stoi(sFileName.substr(vIndex[4]+1, vIndex[5]-vIndex[4]-1));
    LocalIndex.z = std::stoi(sFileName.substr(vIndex[5]+1, sFileName.size() - vIndex[5]-1));
    return true;
  }
}

// get files need saving into mesh
std::vector<SingleVolume> GetFilesNeedSaving(
    std::vector<std::string>&  vfilename)
{
  std::vector<SingleVolume>  vVolumes;

  for(unsigned int i=0; i!=vfilename.size(); i++)
  {
    std::string sFileName = vfilename[i];

    // get index from file name
    int3 GlobalIndex, LocalIndex;

    if( GetIndexFromFileName(sFileName, GlobalIndex, LocalIndex) )
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
    else
    {
      std::cerr<<"  [Kangaroo/GetFilesNeedSaving]Fatal error! Invaild Files!"<<std::endl;
      exit(-1);
    }
  }

  return vVolumes;
}

// Generate one single mesh from several ppm files.
bool SaveMeshFromPXMs(
    std::string                sDirName,
    std::string                sBBFileHead,
    int3                       nVolRes,
    int                        nGridRes,
    std::vector<std::string>   vfilename,
    std::string                sMeshFileName)
{
  printf("\n---- [Kangaroo/SaveMeshFromPXMs] Start.\n");

  // 1 ---------------------------------------------------------------------------
  // read all grid sdf and sort them into volumes. vVolume index is global index
  std::vector<SingleVolume>  vVolumes = GetFilesNeedSaving(vfilename);

  if(vVolumes.size()<=0)
  {
    printf("[Kangaroo/SaveMeshFromPXMs] Cannot find any files for generating the mesh!\n");
    return false;
  }

  // prepare data structure for the single mesh
  MarchingCUBERst ObjMesh;

  // 2 ---------------------------------------------------------------------------
  // For each global volume we have, gen mesh with it
  int nTotalSaveGridNum = 0;

  for(unsigned int i=0; i!=vVolumes.size(); i++)
  {
      std::cout<<"[Kangaroo/SaveMeshFromPXMs] Merging grids in global bb area ("<<
                 std::to_string(vVolumes[i].GlobalIndex.x)<<","<<
                 std::to_string(vVolumes[i].GlobalIndex.y)<<","<<
                 std::to_string(vVolumes[i].GlobalIndex.z)<<")"<< std::endl;

      int nSingleLoopSaveGridNum = 0;

      // load the corresponding bounding box
      std::string sBBFileName =
          sDirName + sBBFileHead +
          std::to_string(vVolumes[i].GlobalIndex.x) + "#" +
          std::to_string(vVolumes[i].GlobalIndex.y) + "#" +
          std::to_string(vVolumes[i].GlobalIndex.z);

      if( CheckIfBBfileExist(sBBFileName) )
      {
        // 1, --------------------------------------------------------------------
        // load the bounxing box of the sdf.
        // NOTICE that this is the GLOBAL bounding box, not the local one.
        // To load it from disk, we need to use host volume
        roo::BoundingBox BBox = LoadPXMBoundingBox(sBBFileName);

        roo::BoundedVolumeGrid<roo::SDF_t_Smart,roo::TargetHost,roo::Manage> hVol;
        hVol.Init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);

        roo::BoundedVolumeGrid<float, roo::TargetHost, roo::Manage> hVolColor;
        hVolColor.Init(1,1,1, nGridRes, BBox);

        // 2, --------------------------------------------------------------------
        // for each single grid volume live in the global bounding box
        for(unsigned int j=0; j!=vVolumes[i].vLocalIndex.size(); j++)
        {
          int3 LocalIndex = vVolumes[i].vLocalIndex[j];

          int nRealIndex = hVol.ConvertLocalIndexToRealIndex(
                LocalIndex.x, LocalIndex.y,LocalIndex.z);

          std::string sPXMFile = sDirName + vVolumes[i].vFileName[j];

          // load the grid volume
          if(LoadPXMSingleGrid(sPXMFile, hVol.m_GridVolumes[nRealIndex]) == false )
          {
            std::cerr<<"[Kangaroo/SaveMeshFromPXMs] Error! load file fail.. exit."<<std::endl;
            return false;
          }
        }

        // 3, --------------------------------------------------------------------
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

                nTotalSaveGridNum++;
                nSingleLoopSaveGridNum ++;
              }
            }
          }
        }

        // 4, --------------------------------------------------------------------
        // reset grid
        roo::SdfReset(hVol);
        hVol.ResetAllGridVol();
      }
      else
      {
        std::cerr<<"[Kangaroo/SaveMeshFromPXMs] Error! Fail loading bbox "<<
                   sBBFileName<<std::endl;
        return false;
      }

      std::cout<<"[Kangaroo/SaveMeshFromPXMs] Finish merge "<<nSingleLoopSaveGridNum<<
                 " grids."<<std::endl;
  }

  std::cout<<"[Kangaroo/SaveMeshFromPXMs] Finish marching cube for " <<
             nTotalSaveGridNum<< " Grids.\n";

  // 3 ---------------------------------------------------------------------------
  // Save mesh from memory to hard disk
  aiMesh* mesh = MeshFromListsVector(ObjMesh.verts, ObjMesh.norms,
                                     ObjMesh.faces, ObjMesh.colors);

  return SaveMeshGridToFileAssimp(sMeshFileName, mesh, "obj");
}


// Generate one single mesh from several ppm files.
bool SaveMeshFromPXMs(
    std::string                sDirName,
    std::string                sBBFileHead,
    int3                       nVolRes,
    int                        nGridRes,
    std::vector<std::string>   vGridsFilename,
    std::vector<std::string>   vGridsGrayFilename,
    std::string                sMeshFileName)
{
  printf("\n---- [Kangaroo/SaveMeshFromPXMs] Start Color Version.\n");

  // 1 ---------------------------------------------------------------------------
  // read all grid sdf and sort them into volumes. vVolume index is global index
  std::vector<SingleVolume>  vGridVolumes = GetFilesNeedSaving(vGridsFilename);
  std::vector<SingleVolume>  vGridGrayVolumes = GetFilesNeedSaving(vGridsGrayFilename);

  if(vGridVolumes.size()<=0)
  {
    printf("[Kangaroo/SaveMeshFromPXMs] Cannot find any files for generating the mesh!\n");
    return false;
  }

  if(vGridVolumes.size() != vGridGrayVolumes.size())
  {
    printf("[Kangaroo/SaveMeshFromPXMs] Grid and Color Grid Size MisMatch!\n");
    return false;
  }

  // prepare data structure for the single mesh
  MarchingCUBERst ObjMesh;

  // 2 ---------------------------------------------------------------------------
  // For each global volume we have, gen mesh with it
  int nTotalSaveGridNum = 0;

  for(unsigned int i=0; i!=vGridVolumes.size(); i++)
  {
      std::cout<<"[Kangaroo/SaveMeshFromPXMs] Merging grids in global bb area ("<<
                 std::to_string(vGridVolumes[i].GlobalIndex.x)<<","<<
                 std::to_string(vGridVolumes[i].GlobalIndex.y)<<","<<
                 std::to_string(vGridVolumes[i].GlobalIndex.z)<<")"<< std::endl;

      int nSingleLoopSaveGridNum = 0;

      // load the corresponding bounding box
      std::string sBBFileName =
          sDirName + sBBFileHead +
          std::to_string(vGridVolumes[i].GlobalIndex.x) + "#" +
          std::to_string(vGridVolumes[i].GlobalIndex.y) + "#" +
          std::to_string(vGridVolumes[i].GlobalIndex.z);

      if( CheckIfBBfileExist(sBBFileName) )
      {
        // 1, --------------------------------------------------------------------
        // load the bounxing box of the sdf.
        // NOTICE that this is the GLOBAL bounding box, not the local one.
        // To load it from disk, we need to use host volume
        roo::BoundingBox BBox = LoadPXMBoundingBox(sBBFileName);
        GpuCheckErrors();
        roo::BoundedVolumeGrid<roo::SDF_t_Smart,roo::TargetHost,roo::Manage> hVol;
        hVol.Init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);
        GpuCheckErrors();
        roo::BoundedVolumeGrid<float, roo::TargetHost, roo::Manage> hColorVol;
        hColorVol.Init(nVolRes.x, nVolRes.y, nVolRes.z, nGridRes, BBox);
        GpuCheckErrors();
        // 2, --------------------------------------------------------------------
        // for each single grid volume live in the global bounding box
        for(unsigned int j=0; j!=vGridVolumes[i].vLocalIndex.size(); j++)
        {
          int3 LocalIndex = vGridVolumes[i].vLocalIndex[j];

          int nRealIndex = hVol.ConvertLocalIndexToRealIndex(
                LocalIndex.x, LocalIndex.y,LocalIndex.z);

          std::string sPXMFile = sDirName + vGridVolumes[i].vFileName[j];

          // load the grid volume
          if(LoadPXMSingleGrid(sPXMFile, hVol.m_GridVolumes[nRealIndex]) == false )
          {
            std::cerr<<"[Kangaroo/SaveMeshFromPXMs] Error! load "<<sPXMFile<<" fail. exit."<<std::endl;
            return false;
          }

          std::string sPXMGrayFile = sDirName + vGridGrayVolumes[i].vFileName[j];

          if(LoadPXMSingleGrid(sPXMGrayFile, hColorVol.m_GridVolumes[nRealIndex]) == false )
          {
            std::cerr<<"[Kangaroo/SaveMeshFromPXMs] Error! load "<<sPXMGrayFile<<" fail. exit."<<std::endl;
            return false;
          }
        }

        // 3, --------------------------------------------------------------------
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

                GenMeshSingleGrid(hVol, hColorVol, CurLocalIndex, ObjMesh.verts,
                                  ObjMesh.norms, ObjMesh.faces, ObjMesh.colors);

                nTotalSaveGridNum++;
                nSingleLoopSaveGridNum ++;
              }
            }
          }
        }

        // 4, --------------------------------------------------------------------
        // reset grid
        GpuCheckErrors();
        roo::SdfReset(hVol);
        hVol.ResetAllGridVol();

        roo::SdfReset(hColorVol);
        hColorVol.ResetAllGridVol();

        GpuCheckErrors();
      }
      else
      {
        std::cerr<<"[Kangaroo/SaveMeshFromPXMs] Error! Fail loading bbox "<<
                   sBBFileName<<std::endl;
        return false;
      }

      std::cout<<"[Kangaroo/SaveMeshFromPXMs] Finish merge "<<nSingleLoopSaveGridNum<<
                 " grids."<<std::endl;
  }

  std::cout<<"[Kangaroo/SaveMeshFromPXMs] Finish marching cube for " <<
             nTotalSaveGridNum<< " Grids.\n";

  // 3 ---------------------------------------------------------------------------
  // Save mesh from memory to hard disk
  aiMesh* mesh = MeshFromListsVector(ObjMesh.verts, ObjMesh.norms,
                                     ObjMesh.faces, ObjMesh.colors);

  // to keep color for the mesh, we have to save it as ply format
  return SaveMeshGridToFileAssimp(sMeshFileName, mesh, "ply");
}


}
