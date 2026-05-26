/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _AVL_TYPES_H_
#define _AVL_TYPES_H_

#define nullptr     NULL
#define AVL_FALSE   0
#define AVL_TRUE    1

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

  typedef unsigned char AVL_bool;

  typedef char AVL_char;           ///< 8 bits signed char data type.
  typedef unsigned char AVL_uchar; ///< 8 bits unsigned char data type.

  typedef short AVL_int16;           ///< 16 bits signed char data type.
  typedef unsigned short AVL_uint16; ///< 16 bits unsigned char data type.

  typedef int AVL_int32;           ///< 32 bits signed char data type.
  typedef unsigned int AVL_uint32; ///< 32 bits unsigned char data type.

  typedef char *AVL_pchar;           ///< pointer to a 8 bits signed char data type.
  typedef unsigned char *AVL_puchar; ///< pointer to a 8 bits unsigned char data type.

  typedef short *AVL_pint16;           ///< pointer to a 16 bits signed char data type.
  typedef unsigned short *AVL_puint16; ///< pointer to a 16 bits unsigned char data type.

  typedef int *AVL_pint32;           ///< pointer to a 32 bits signed char data type.
  typedef unsigned int *AVL_puint32; ///< pointer to a 32 bits unsigned char data type.

  typedef unsigned char AVL_semaphore;   ///< the semaphore data type.
  typedef unsigned char *AVL_psemaphore; ///< the pointer to a semaphore data type.

  typedef AVL_uint16 AVL_ErrorCode; // Defines the error code

#ifdef AVL_CPLUSPLUS
}
#endif

#endif
