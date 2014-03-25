#pragma once

#include "BaseDisplay.h"

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

inline pangolin::View& SetupPangoGLWithCuda(int w, int h, int ui_width = 180, std::string window_title = "-", unsigned int min_gpu_mem_mb = 500 )
{
    pangolin::View& container = SetupPangoGL(w,h,ui_width, window_title);

    // Initialise CUDA, allowing it to use OpenGL context
    if( cudaGLSetGLDevice(0) != cudaSuccess ) {
        std::cerr << "Unable to get CUDA Device" << std::endl;
        exit(-1);
    }
    const unsigned bytes_per_mb = 1024*1000;
    size_t cu_mem_start, cu_mem_total;
    if(cudaMemGetInfo( &cu_mem_start, &cu_mem_total ) != cudaSuccess) {
        std::cerr << "Unable to get available memory" << std::endl;
        exit(-1);
    }

    std::cout << cu_mem_start/bytes_per_mb << " MB Video Memory Available. Require At least "<< min_gpu_mem_mb<<" mb Video Memory" << std::endl;

    if( (cu_mem_start/bytes_per_mb) < min_gpu_mem_mb ) {
        std::cerr << "Fatal Error! Not enough memory to run RTL. Require At least "<< min_gpu_mem_mb << std::endl;
        exit(-1);
    }
    return container;
}
