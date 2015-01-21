#include "MarchingCubesGrid.h"


// fGetOffset finds the approximate point of intersection of the surface
// between two points with the values fValue1 and fValue2
KANGAROO_EXPORT
float fGetOffset(
    float                                             fValue1,
    float                                             fValue2,
    float                                             fValueDesired)
{
  const double fDelta = fValue2 - fValue1;
  if(fDelta == 0.0) {
    return 0.5;
  }
  return (fValueDesired - fValue1)/fDelta;
}
