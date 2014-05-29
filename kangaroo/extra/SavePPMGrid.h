#ifndef SAVEPPMGRID_H
#define SAVEPPMGRID_H

#include "SavePPM.h"
#include <kangaroo/BoundedVolumeGrid.h>

// P1	Portable bitmap	ASCII
// P2	Portable graymap	ASCII
// P3	Portable pixmap	ASCII
// P4	Portable bitmap	Binary
// P5	Portable graymap	Binary
// P6	Portable pixmap	Binary

/////////////////////////////////////////////////////////////////////////////
// Save Volume types
/////////////////////////////////////////////////////////////////////////////

template<typename T, typename Manage>
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  if(vol.w<=0 || vol.h<=0 || vol.d<=0)
  {
    std::cerr<<"[Kangaroo/SavePXMGrid] Fatal error! Cannot save empty PXM file!"<<std::endl;
    exit(-1);
  }

  bFile << ppm_type << std::endl;
  bFile << vol.w << " " << vol.h << " " << vol.d << '\n';
  bFile << num_colors << '\n';

  for(unsigned int d=0; d<vol.d; ++d) {
    for(unsigned int r=0; r<vol.h; ++r) {
      bFile.write( (const char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
    }
  }
  bFile.close();
}

template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetHost,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile, vol, ppm_type, num_colors);
}


template<typename T, typename Manage>
void SavePXM(std::ofstream& bFile, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  roo::VolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.InitVolume(vol.w, vol.h, vol.d);
  hvol.CopyFrom(vol);
  SavePXM(bFile, hvol, ppm_type, num_colors);
}


template<typename T, typename Manage>
void SavePXM(const std::string filename, const roo::VolumeGrid<T,roo::TargetDevice,Manage>& vol,
             std::string ppm_type = "P5", int num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile,vol,ppm_type,num_colors);
}


inline void SavePXMBoundingBox(const std::string filename, roo::BoundingBox BBox)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  bFile << BBox.boxmin.x << " " <<  BBox.boxmin.y << " " << BBox.boxmin.z << std::endl;
  bFile << BBox.boxmax.x << " " <<  BBox.boxmax.y << " " << BBox.boxmax.z << std::endl;

  printf("save bb success File: %s\n",filename.c_str());
  bFile.close();
}



inline roo::BoundingBox LoadPXMBoundingBox(std::string filename)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  if(bFile.fail()==true)
  {
    std::cerr<<"[Kangaroo/LoadPXMBoundingBox] Fatal error! file "<<filename<<" does not exist."<<std::endl;
    exit(-1);
  }

  //read in the bounding volume bounds
  roo::BoundingBox BBox;

  bFile >> BBox.boxmin.x;
  bFile >> BBox.boxmin.y;
  bFile >> BBox.boxmin.z;
  bFile >> BBox.boxmax.x;
  bFile >> BBox.boxmax.y;
  bFile >> BBox.boxmax.z;
  bFile.ignore(1,'\n');

  std::cout<<"[SavePPMGrid] load bounding box "<<filename<<" success!"<<" Min ("<<
             BBox.boxmin.x<<","<<BBox.boxmin.y<<","<<BBox.boxmin.z<<
             ");Max ("<<BBox.boxmax.x<<","<<BBox.boxmax.y<<","<<BBox.boxmax.z<<")"<<std::endl;

  bFile.close();
  return BBox;
}


///================================ Save BB ====================================
template<typename T, typename Manage>
void SavePXM(const std::string                                      sFilename,
             roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol,
             bool                                                   bGlobalPose = false,
             std::string                                            ppm_type = "P5",
             int                                                    num_colors = 255)
{
  if(rDVol.GetActiveGridVolNum()==0)
  {
    std::cerr<<"[Kangaroo/SavePXMGrid] Cannot save model for void volume. Empty vol."<<std::endl;
  }
  else
  {
    // load data from device to host
    roo::BoundedVolumeGrid<T,roo::TargetHost, Manage> hvol;
    hvol.init(rDVol.m_w,rDVol.m_h,rDVol.m_d,rDVol.m_nVolumeGridRes,rDVol.m_bbox);
    hvol.CopyAndInitFrom(rDVol);
    hvol.m_global_shift = rDVol.m_global_shift;

    // First save bounding box to HardDisk.
    std::string sBBFileName = sFilename+"-BB";

    SavePXMBoundingBox(sBBFileName, rDVol.m_bbox);

    // Second save each active volume in BoundedVolumeGrid to HardDisk
    int nNum =0;

    for(int i=0;i!=rDVol.m_nWholeGridRes_w;i++)
    {
      for(int j=0;j!=rDVol.m_nWholeGridRes_h;j++)
      {
        for(int k=0;k!=rDVol.m_nWholeGridRes_d;k++)
        {
          int nGridIndex = hvol.GetLocalIndex(i,j,k);

          if(hvol.CheckIfBasicSDFActive(nGridIndex)==true)
          {
            // save local index (without rolling)
            std::string sFileName;
            if(bGlobalPose==false)
            {
              // only save local grid
              sFileName = sFilename+"-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }
            else
            {
              // save global index (with rolling)
              int3 GlobalIndex = rDVol.GetGlobalIndex(i,j,k);
              sFileName =sFilename+"-"+std::to_string(GlobalIndex.x)+
                  "-"+std::to_string(GlobalIndex.y)+
                  "-"+std::to_string(GlobalIndex.z)+
                  "-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }

            std::ofstream bFile( sFileName.c_str(), std::ios::out | std::ios::binary );
            SavePXM<T,Manage>(bFile, hvol.m_GridVolumes[nGridIndex], ppm_type,num_colors);

            std::cout<<"[Kangaroo/SavePXMGrid] Save "<<sFileName<<" success."<<std::endl;
            nNum++;
          }
        }
      }
    }

    printf("[Kangaroo/SavePXMGrid] Save %d grid sdf.\n", nNum);
  }

}


inline bool CheckIfBBfileExist(std::string filename)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  if(bFile.fail()==true)
  {
    return false;
  }

  bFile.close();
  return true;
}


template<typename T, typename Manage>
void CheckifSaveBB(const std::string                                      sFilename,
                   int3                                                   GlobalIndex,
                   roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol)
{
  std::string sBBFileName = sFilename +"-BB-"+ std::to_string(GlobalIndex.x) + "-"+
      std::to_string(GlobalIndex.y) + "-"+ std::to_string(GlobalIndex.z);

  if( CheckIfBBfileExist(sBBFileName) == false)
  {
    // save it directlly if global shift just updated
    if(GlobalIndex.x == rDVol.m_global_shift.x &&
       GlobalIndex.y == rDVol.m_global_shift.y &&
       GlobalIndex.z == rDVol.m_global_shift.z)
    {
      SavePXMBoundingBox(sBBFileName, rDVol.m_bbox);
    }
    // the bounding box is not totally updated yet. so we need to compute it
    else
    {
      // read origin bounding box.
      std::string sOriginBBFileName = sFilename +"-BB-"+ std::to_string(0) + "-"+
          std::to_string(0) + "-"+ std::to_string(0);

      roo::BoundingBox OriginBB = LoadPXMBoundingBox(sOriginBBFileName);

      roo::BoundingBox bbox;
      bbox.boxmin.x = OriginBB.boxmin.x+ (GlobalIndex.x)*rDVol.m_bbox.Size().x;
      bbox.boxmin.y = OriginBB.boxmin.y+ (GlobalIndex.y)*rDVol.m_bbox.Size().y;
      bbox.boxmin.z = OriginBB.boxmin.z+ (GlobalIndex.z)*rDVol.m_bbox.Size().z;

      bbox.boxmax.x = OriginBB.boxmax.x+ (GlobalIndex.x)*rDVol.m_bbox.Size().x;
      bbox.boxmax.y = OriginBB.boxmax.y+ (GlobalIndex.y)*rDVol.m_bbox.Size().y;
      bbox.boxmax.z = OriginBB.boxmax.z+ (GlobalIndex.z)*rDVol.m_bbox.Size().z;

      SavePXMBoundingBox(sBBFileName, bbox);
    }
  }


}

// -----------------------------------------------------------------------------
// only save desire bounding box.
template<typename T, typename Manage>
void SavePXM(const std::string                                      filename,
             int                                                    pGridNeedSave[],
             roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol,
             bool                                                   bGlobalPose = false,
             bool                                                   bSaveBBox = false,
             std::string                                            ppm_type = "P5",
             int                                                    num_colors = 255)
{
  if(rDVol.GetActiveGridVolNum()==0)
  {
    std::cerr<<"[Kangaroo/SavePXMGrid] Cannot save model for void volume. Empty vol."<<std::endl;
  }
  else
  {
    // load data from device to host
    roo::BoundedVolumeGrid<T,roo::TargetHost, Manage> hvol;
    hvol.init(rDVol.m_w,rDVol.m_h,rDVol.m_d,rDVol.m_nVolumeGridRes,rDVol.m_bbox);
    hvol.CopyAndInitFrom(rDVol);
    hvol.m_global_shift = rDVol.m_global_shift;

    // save each active volume in BoundedVolumeGrid to HardDisk
    int nNum =0;

    for(int i=0;i!=int(rDVol.m_nWholeGridRes_w);i++)
    {
      for(int j=0;j!=int(rDVol.m_nWholeGridRes_h);j++)
      {
        for(int k=0;k!=int(rDVol.m_nWholeGridRes_d);k++)
        {
          int nGridIndex = hvol.GetLocalIndex(i,j,k);

          // check if we need to save this vol
          if(pGridNeedSave[nGridIndex]==1 && hvol.CheckIfBasicSDFActive(nGridIndex)==true)
          {
            int3 GlobalIndex = rDVol.GetGlobalIndex(i,j,k);

            // -- save grid sdf ------------------------------------------------
            // save local index (without rolling)
            std::string sFileName;
            if(bGlobalPose==false)
            {
              // only save local grid
              sFileName = filename+"-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }
            else
            {
              // save grid in global index (with rolling)
              sFileName =filename+"-"+std::to_string(GlobalIndex.x)+
                  "-"+std::to_string(GlobalIndex.y)+"-"+std::to_string(GlobalIndex.z)+
                  "-"+std::to_string(i)+"-"+std::to_string(j)+"-"+std::to_string(k);
            }

            std::ofstream bFile( sFileName.c_str(), std::ios::out | std::ios::binary );
            SavePXM<T,Manage>(bFile, hvol.m_GridVolumes[nGridIndex], ppm_type,num_colors);

            // --- save bounding box if necessary ------------------------------
            // scan the disk and see if we need to save bb
            CheckifSaveBB(filename, GlobalIndex, rDVol);

//            std::cout<<"[Kangaroo/SavePXMGridDesire] Save "<<sFileName<<" success."<<std::endl;
            nNum++;
          }
        }
      }
    }

    printf("[Kangaroo/SavePXMGridDesire] Save %d grid sdf.\n", nNum);

  }

}


/////////////////////////////////////////////////////////////////////////////
//                           Load Volume types.
/////////////////////////////////////////////////////////////////////////////

template<typename T, typename Manage>
bool LoadPXMSingleGrid(const std::string                          filename,
                       roo::VolumeGrid<T,roo::TargetHost,Manage>& vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );

  if(bFile.fail()==true)
  {
    std::cout<<"[Kangaroo/LoadPXMSIngleGrid]error! cannot open file "<<filename<<std::endl;
    return false;
  }

  // Parse header
  std::string ppm_type = "P5";
  int num_colors = 0;
  int w = 0;
  int h = 0;
  int d = 0;

  bFile >> ppm_type;
  bFile >> w;
  bFile >> h;
  bFile >> d;
  bFile >> num_colors;
  bFile.ignore(1,'\n');

  bool success = !bFile.fail() && w > 0 && h > 0 && d > 0;

  if(success) {
    // init volume grid
    vol.InitVolume(w,h,d);
    GpuCheckErrors();

    // Read in data
    for(size_t d=0; d<vol.d; ++d)
    {
      for(size_t r=0; r<vol.h; ++r)
      {
        bFile.read( (char*)vol.RowPtr(r,d), vol.w * sizeof(T) );
      }
    }
    success = !bFile.fail();
  }
  else
  {
    std::cout<<"[Kangaroo/LoadPXMSIngleGrid] error! dim error. input dim are: "
            <<"w:"<<w<<",h"<<h<<",d"<<d<<std::endl;

    bFile.close();
    return false;
  }

  bFile.close();
  return true;
}

template<typename T>
bool LoadPXMGrid(std::string                                              sDirName,
                 const std::vector<std::string>&                          vfilename,
                 std::string                                              sBBFileName,
                 roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>& rDVol)
{
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;

  roo::BoundingBox BBox = LoadPXMBoundingBox(sDirName+sBBFileName);

  // init sdf
  hvol.init(rDVol.m_w, rDVol.m_h,rDVol.m_d,rDVol.m_nVolumeGridRes,BBox);

  // read bb box..
  int nNum = 0;

  // load each single VolumeGrid
  for(int i=0;i!=vfilename.size();i++)
  {
    // get index from file name
    std::string sFileName = vfilename[i];
    std::string sIndex = sFileName.substr(sFileName.find_last_of("-")+1,
                                          sFileName.size() - sFileName.find_last_of("-"));

    if(sIndex!="BB")
    {
      int nIndex = std::atoi(sIndex.c_str());

      if(LoadPXMSingleGrid(sDirName+ sFileName, hvol.m_GridVolumes[nIndex]) == false)
      {
        std::cout<<"[LoadPXMGrid] Fatal error! cannot read single volume grid "<<sFileName<<
                   " with index "<<nIndex<<" from hard disk."<<std::endl;
        exit(-1);
      }
      else
      {
        nNum ++;
      }
    }
  }

  // copy data from host to device
  rDVol.CopyAndInitFrom(hvol);
  GpuCheckErrors();

  printf("[LoadPXMGrid] Finish load %d data to device. Available memory is %d\n",nNum, GetAvailableGPUMemory());

  return true;
}


#endif // SAVEPPMGRID_H
