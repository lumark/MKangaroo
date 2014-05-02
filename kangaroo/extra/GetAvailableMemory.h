#ifndef GETAVAILABLEMEMORY_H
#define GETAVAILABLEMEMORY_H

#include <kangaroo/kangaroo.h>

inline int GetAvailableGPUMemory()
{
  const unsigned bytes_per_mb = 1024*1000;
  size_t cu_mem_start, cu_mem_total;
  if(cudaMemGetInfo( &cu_mem_start, &cu_mem_total ) != cudaSuccess) {
    std::cerr << "[GetAvailableGPUMemory] Unable to get available memory" << std::endl;
    exit(-1);
  }

  return cu_mem_start/bytes_per_mb;
}

#endif // GETAVAILABLEMEMORY_H
