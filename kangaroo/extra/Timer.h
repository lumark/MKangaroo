#include <sys/time.h>
#include <cuda_runtime.h>

namespace roo {
////////////////////////////////////////////////////////////////////////////////
inline double _Tic()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec  + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////
inline double _Toc( double dSec )
{
  return _Tic() - dSec;
}

inline double _TicCuda()
{
  cudaDeviceSynchronize();
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec  + 1e-6 * (tv.tv_usec);
}

////////////////////////////////////////////////////////////////////////////////
inline double _TocCuda( double dSec )
{
  cudaDeviceSynchronize();
  return _Tic() - dSec;
}


}

