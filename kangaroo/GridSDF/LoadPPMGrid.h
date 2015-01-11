#ifndef LOADPPMGRID_H
#define LOADPPMGRID_H

#include <kangaroo/Image.h>
#include <kangaroo/Volume.h>
#include <kangaroo/BoundedVolume.h>
#include "BoundedVolumeGrid.h"

// P1	Portable bitmap	ASCII
// P2	Portable graymap	ASCII
// P3	Portable pixmap	ASCII
// P4	Portable bitmap	Binary
// P5	Portable graymap	Binary
// P6	Portable pixmap	Binary

/////////////////////////////////////////////////////////////////////////////
// Load Volume types
/////////////////////////////////////////////////////////////////////////////
KANGAROO_EXPORT
template<typename T>
bool LoadPXMSingleGrid(
    const std::string                                      sFilename,
    roo::VolumeGrid<T,roo::TargetHost,roo::Manage>&        vol)
{
  std::ifstream bFile( sFilename.c_str(), std::ios::in | std::ios::binary );

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

  if(success)
  {
    // Make sure the vol is empty
    vol.CleanUp();

    // init volume grid
    vol.InitVolume(w, h, d);

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
    std::cerr<<"fatal error! read PPM File Fail!"<<std::endl;
    exit(-1);
  }

  bFile.close();
  return success;
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
    roo::BoundedVolumeGrid<T,roo::TargetDevice,roo::Manage>&  Dvol)
{
  // to load it from disk, we need to use host volume
  roo::BoundedVolumeGrid<T,roo::TargetHost,roo::Manage> hvol;
  roo::BoundingBox BBox = LoadPXMBoundingBox(sDirName + sBBFileName);

  // init sdf
  hvol.Init(Dvol.m_w, Dvol.m_h, Dvol.m_d, Dvol.m_nVolumeGridRes, BBox);

  std::cout<<"Finish init sdf. loading model.."<<std::endl;
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
        std::cout<<"[LoadPXMGrid] Error! cannot read single volume grid "<<
                   sFileName<< " with index "<<nIndex<<" from hard disk."<<std::endl;
        exit(-1);
      }
    }
  }

  // copy data from host to device
  Dvol.CopyAndInitFrom(hvol);
  GpuCheckErrors();

  return true;
}

#endif // LOADPPMGRID_H
