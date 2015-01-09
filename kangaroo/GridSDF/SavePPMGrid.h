#ifndef SAVEPPMGRID_H
#define SAVEPPMGRID_H

#include "kangaroo/extra/SavePPM.h"
#include "BoundedVolumeGrid.h"

// P1	Portable bitmap	ASCII
// P2	Portable graymap	ASCII
// P3	Portable pixmap	ASCII
// P4	Portable bitmap	Binary
// P5	Portable graymap	Binary
// P6	Portable pixmap	Binary

/////////////////////////////////////////////////////////////////////////////
// Save Volume types
/////////////////////////////////////////////////////////////////////////////

KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXM(
    std::ofstream&                                         bFile,
    const roo::VolumeGrid<T,roo::TargetHost,Manage>&       vol,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
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

KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXM(
    const std::string                                      filename,
    const roo::VolumeGrid<T,roo::TargetHost,Manage>&       vol,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile, vol, ppm_type, num_colors);
}

KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXM(
    std::ofstream& bFile,
    const roo::VolumeGrid<T,roo::TargetDevice,Manage>&     vol,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
  roo::VolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  hvol.InitVolume(vol.w, vol.h, vol.d);
  hvol.CopyFrom(vol);
  SavePXM(bFile, hvol, ppm_type, num_colors);
}

KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXM(
    const std::string                                      filename,
    const roo::VolumeGrid<T,roo::TargetDevice,Manage>&     vol,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  SavePXM<T,Manage>(bFile,vol,ppm_type,num_colors);
}

KANGAROO_EXPORT
inline void SavePXMBoundingBox(
    const std::string                                      filename,
    roo::BoundingBox                                       BBox)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  bFile << BBox.boxmin.x << " " <<  BBox.boxmin.y << " " << BBox.boxmin.z << std::endl;
  bFile << BBox.boxmax.x << " " <<  BBox.boxmax.y << " " << BBox.boxmax.z << std::endl;

  printf("save bb success.\n");
  bFile.close();
}

KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXM(
    const std::string                                      filename,
    roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   vol,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
  // load data from device to host
  roo::BoundedVolumeGrid<T,roo::TargetHost, Manage> hvol;
  hvol.Init(vol.m_w,vol.m_h,vol.m_d,vol.m_nVolumeGridRes,vol.m_bbox);
  hvol.CopyAndInitFrom(vol);

  // first save bounding box
  std::string sBBFileName = filename+"-BB";
  SavePXMBoundingBox(sBBFileName, vol.m_bbox);

  // save each active volume in BoundedVolumeGrid to HardDisk
  for(int i=0;i!=vol.GetTotalGridNum();i++)
  {
    if(hvol.CheckIfBasicSDFActive(i)==true)
    {
      // save
      std::string sFileName = filename + "-" + std::to_string(i);
      std::ofstream bFile( sFileName.c_str(), std::ios::out | std::ios::binary );
      SavePXM<T,Manage>(bFile,hvol.m_GridVolumes[i],ppm_type,num_colors);
    }
  }
}

///============================= Save Grid SDFs ================================
KANGAROO_EXPORT
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

KANGAROO_EXPORT
template<typename T, typename Manage>
void CheckifSaveBB(
    const std::string                                      sFilename,
    int3                                                   GlobalIndex,
    roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol)
{
  std::string sBBFileName = sFilename +"-BB#"+ std::to_string(GlobalIndex.x) + "#"+
      std::to_string(GlobalIndex.y) + "#"+ std::to_string(GlobalIndex.z);

  if( CheckIfBBfileExist(sBBFileName) == false)
  {
    SavePXMBoundingBox(sBBFileName, rDVol.GetDesireBB(GlobalIndex));
  }
}


// file name format ObjID-GlobalIndex-LocalIndex or ObjID-LocalIndex
KANGAROO_EXPORT
template<typename T, typename Manage>
void SavePXMGridDesire(
    const std::string                                      sPathName,
    int                                                    pGridNeedSave[],
    roo::BoundedVolumeGrid<T,roo::TargetDevice, Manage>&   rDVol,
    bool                                                   bSaveBBox,
    std::string                                            ppm_type = "P5",
    int                                                    num_colors = 255)
{
  if(rDVol.GetActiveGridVolNum()==0)
  {
    std::cerr<<"[Kangaroo/SavePXMGridDesire] Cannot save PXM for void volume."<<std::endl;
    exit(-1);
  }
  else
  {
    // load data from device to host
    roo::BoundedVolumeGrid<T,roo::TargetHost, Manage> hVol;
    hVol.Init(rDVol.m_w,rDVol.m_h,rDVol.m_d,rDVol.m_nVolumeGridRes,rDVol.m_bbox);
    hVol.CopyAndInitFrom(rDVol);
    hVol.m_global_shift = rDVol.m_global_shift;

    // save each active volume in BoundedVolumeGrid to HardDisk
    int nSaveGridNum =0;

    for(int i=0;i!=static_cast<int>(rDVol.m_nGridRes_w);i++)
    {
      for(int j=0;j!=static_cast<int>(rDVol.m_nGridRes_h);j++)
      {
        for(int k=0;k!=static_cast<int>(rDVol.m_nGridRes_d);k++)
        {
          int nGridIndex = i + rDVol.m_nGridRes_w* (j+ rDVol.m_nGridRes_h* k);

          // --- save vol if necessary
          if( pGridNeedSave[nGridIndex]==1 && hVol.CheckIfBasicSDFActive(nGridIndex) )
          {
            int3 GlobalIndex = rDVol.m_global_shift;

            // the actual lcoal index of the grid
            int3 LocalIndex  = make_int3(i,j,k);

            std::string sGridFileName = sPathName+"#"+
                std::to_string(GlobalIndex.x)+"#"+std::to_string(GlobalIndex.y)+"#"+
                std::to_string(GlobalIndex.z)+"#"+std::to_string(LocalIndex.x)+"#"+
                std::to_string(LocalIndex.y)+"#"+std::to_string(LocalIndex.z);

            std::ofstream bFile( sGridFileName.c_str(), std::ios::out | std::ios::binary );
            SavePXM<T,Manage>(bFile, hVol.m_GridVolumes[nGridIndex], ppm_type, num_colors);
            nSaveGridNum++;

            // scan the disk and see if we need to save the bb (in global pose)
            if(bSaveBBox == true)
            {
              CheckifSaveBB(sPathName, GlobalIndex, rDVol);
            }
          }
        }
      }
    }

    printf("\n[Kangaroo/SavePXMGridDesire] Save %d grid sdf in Global Pose.\n", nSaveGridNum);
  }

}

/////////////////////////////////////////////////////////////////////////////
// Load Volume types
/////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
template<typename T>
bool LoadPXMSingleGrid(
    const std::string                                      filename,
    roo::VolumeGrid<T,roo::TargetHost,roo::Manage>&        vol)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );

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
    bFile.close();
    return false;
  }

  bFile.close();
  return true;
}

KANGAROO_EXPORT
inline roo::BoundingBox LoadPXMBoundingBox(std::string filename)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  if(bFile.fail()==true)
  {
    std::cerr<<"Fatal error! file "<<filename<<" does not exist."<<std::endl;
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

  //  printf("Load BB success. BBMin:(%f,%f,%f), BBMax:(%f,%f,%f) \n",
  //         BBox.boxmin.x, BBox.boxmin.y,BBox.boxmin.z,
  //         BBox.boxmax.x, BBox.boxmax.y, BBox.boxmax.z);

  bFile.close();
  return BBox;
}

KANGAROO_EXPORT
template<typename T>
bool LoadPXMGrid(
    std::string                                               sDirName,
    const std::vector<std::string>&                           vfilename,
    std::string                                               sBBFileName,
    roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>&  vol)
{
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;

  roo::BoundingBox BBox = LoadPXMBoundingBox(sDirName+sBBFileName);

  // init sdf
  hvol.Init(vol.m_w, vol.m_h,vol.m_d,vol.m_nVolumeGridRes,BBox);

  // read bb box..
  int nNum = 0;

  // load each single VolumeGrid
  for(int i=0;i!=vfilename.size();i++)
  {
    // get index from file name
    std::string sFileName = vfilename[i];
    std::string sIndex = sFileName.substr(
          sFileName.find_last_of("-")+1,
          sFileName.size() - sFileName.find_last_of("-"));

    if(sIndex!="BB")
    {
      int nIndex = std::atoi(sIndex.c_str());

      if(LoadPXMSingleGrid(sDirName+ sFileName, hvol.m_GridVolumes[nIndex]) == false)
      {
        std::cout<<"[LoadPXMGrid] Fatal error! cannot read single volume grid "<<
                   sFileName<< " with index "<<nIndex<<" from hard disk."<<std::endl;
        exit(-1);
      }
      else
      {
        nNum ++;
      }
    }
  }

  // copy data from host to device
  vol.CopyAndInitFrom(hvol);
  GpuCheckErrors();

  return true;
}


#endif // SAVEPPMGRID_H
