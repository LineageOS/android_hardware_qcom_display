/*
* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __DISP_COLOR_PROCESSING_H__
#define __DISP_COLOR_PROCESSING_H__

#ifdef __cplusplus
extern "C" {
#endif

#define LUT3D_ENTRIES_SIZE (17 * 17 * 17)

struct rgb {
  uint32_t r;
  uint32_t g;
  uint32_t b;
};

struct rgb_entry {
  struct rgb in;
  struct rgb out;
};

/*
struct lut3d_info
  flags     -- Reserved
  lut3d_id  -- Unique ID used to distinguish individual 3D Lut tables
  uniform   -- Identifies if coefficients are uniform (1) or non-uniform (0)
  num_entries-- Identifies the number of lut values in the table
  entries   -- Table buffer holding the lut values.

               All coefficients are expected to be tightly packed in the
               structure, any unused buffer space shall exist at the end
               of the buffer.
*/
struct lut3d_info {
  uint32_t flags;
  uint32_t lut3d_id;
  uint32_t uniform;
  uint32_t num_entries;
  struct rgb_entry entries[LUT3D_ENTRIES_SIZE];
};

/*
struct lut1d_info
  flags       -- Reserved
  bit_width   -- Bit width of each entry
  num_entries -- Identifies the number of lut values in the table
  entries     -- Table buffer holding the lut values
*/
#define LUT1D_ENTRIES_SIZE (256)
struct lut1d_info {
  uint32_t flags;
  uint32_t bit_width;
  uint32_t strength;
  uint32_t num_entries;
  struct rgb entries[LUT1D_ENTRIES_SIZE];
};

#ifdef __cplusplus
}
#endif
#endif  // __DISP_COLOR_PROCESSING_H__
