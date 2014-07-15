//
//  ik.h
//
//  Created by Jonathan Tompson on 2/23/13.
//
//  Top level header file.  Includes most of the useful utilities.
//  Everything is in the ik namespace!
//  

#pragma once

#include "ik/math/math_types.h"  // Lots of default matrix and vector types
#include "ik/math/noise.h"  // Generate continuously varying noisy keyframes
#include "ik/string_util/string_util.h"  // Common string functions
#include "ik/exceptions/wruntime_error.h"  // Breakpoint in DEBUG builds
#if defined( _WIN32 )
  #include "ik/string_util/win32_debug_buffer.h"
#endif
#include "ik/math/perlin_noise.h"
