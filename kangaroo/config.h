#ifndef KANGAROO_CONFIG_H
#define KANGAROO_CONFIG_H

/// Platform
#define _UNIX_
/* #undef _WIN_ */
#define _OSX_
/* #undef _LINUX_ */

/// Compiler
#define _GCC_
/* #undef _CLANG_ */
/* #undef _MSVC_ */

/// Configured libraries
#define HAVE_EIGEN
#define HAVE_ASSIMP
#define HAVE_THRUST
#define HAVE_NPP
#define HAVE_OPENCV

/// CUDA Toolkit Version
#define CUDA_VERSION_MAJOR 5
#define CUDA_VERSION_MINOR 0

/// Defines generated when calling into Kangaroo API. Not to be
/// used in compiled library code, only inlined header code.
#if (__cplusplus > 199711L) || (_MSC_VER >= 1700)
#define CALLEE_HAS_CPP11
#define CALLEE_HAS_RVALREF
#endif

#endif // KANGAROO_CONFIG_H
