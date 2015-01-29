// by lu.ma@colorado.edu

#include "SavePPMGrid.h"

KANGAROO_EXPORT
void SavePXMBoundingBox(
    const std::string                                      filename,
    roo::BoundingBox                                       BBox)
{
  std::ofstream bFile( filename.c_str(), std::ios::out | std::ios::binary );
  bFile << BBox.boxmin.x << " " <<  BBox.boxmin.y << " " << BBox.boxmin.z << std::endl;
  bFile << BBox.boxmax.x << " " <<  BBox.boxmax.y << " " << BBox.boxmax.z << std::endl;
  bFile.close();
}


bool CheckIfBBfileExist(std::string filename)
{
  std::ifstream bFile( filename.c_str(), std::ios::in | std::ios::binary );
  if(bFile.fail()==true)
  {
    return false;
  }

  bFile.close();
  return true;
}
