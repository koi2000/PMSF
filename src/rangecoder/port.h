/*****************************************************************************
* Copyright (C) 2013 Adrien Maglo
*
* This file is part of POMAR.
*
* POMAR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* POMAR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with POMAR.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

#ifndef port_h
#define port_h
#include <limits.h>

#ifdef GCC
#define Inline inline
#else
#define Inline __inline
#endif

#if INT_MAX > 0x7FFF
typedef unsigned short uint2;  /* two-byte integer (large arrays)      */
typedef unsigned int   uint4;  /* four-byte integers (range needed)    */
#else
typedef unsigned int   uint2;
typedef unsigned long  uint4;
#endif

typedef unsigned int uint;     /* fast unsigned integer, 2 or 4 bytes  */

#endif
