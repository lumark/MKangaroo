// by lu.ma@colorado.edu

#include "LoadPPMGrid.h"

KANGAROO_EXPORT
roo::BoundingBox LoadPXMBoundingBox(std::string filename)
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

