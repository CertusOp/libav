/*
 * This file is part of Libav.
 *
 * Libav is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Libav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "libavutil/attributes.h"
#include "libavutil/arm/cpu.h"
#include "libavcodec/vp8dsp.h"
#include "vp8dsp.h"

void ff_vp8_luma_dc_wht_neon(int16_t block[4][4][16], int16_t dc[16]);

void ff_vp8_idct_add_neon(uint8_t *dst, int16_t block[16], ptrdiff_t stride);
void ff_vp8_idct_dc_add_neon(uint8_t *dst, int16_t block[16], ptrdiff_t stride);
void ff_vp8_idct_dc_add4y_neon(uint8_t *dst, int16_t block[4][16], ptrdiff_t stride);
void ff_vp8_idct_dc_add4uv_neon(uint8_t *dst, int16_t block[4][16], ptrdiff_t stride);

VP8_LF(neon);

VP8_EPEL(16, neon);
VP8_EPEL(8,  neon);
VP8_EPEL(4,  neon);

VP8_BILIN(16, neon);
VP8_BILIN(8,  neon);
VP8_BILIN(4,  neon);

#if 0

static vp8_mc_func real_put_vp8_epel;

static void (*filter_c)(uint8_t *dst, ptrdiff_t stride,
			   int flim_E, int flim_I, int hev_thresh);



static void filter_wrapper(uint8_t *dst, ptrdiff_t stride,
			   int flim_E, int flim_I, int hev_thresh)
{
    static int count;
    const int len = 4 * 2 * stride  +  16;

    
    uint8_t *tmp = malloc(len);
    uint8_t *pre = malloc(len);
    memcpy(tmp, dst - (4 * stride), len);
    memcpy(pre, dst - (4 * stride), len);
    filter_c(dst, stride, flim_E, flim_I, hev_thresh);
    ff_vp8_v_loop_filter16_inner_neon(tmp + 4 * stride, stride, flim_E, flim_I, hev_thresh);
    
    count++;
    dst -= (4 * stride);
    if (memcmp(tmp, dst, len)) {
      int i, j;
      fprintf(stderr, "count %d - %d %d %d\n", count, flim_E, flim_I, hev_thresh);
      fprintf(stderr, "init :\n");
      for (j = 0; j < len; j += stride) {
	for (i = 0; i < 16; i++) {
	  fprintf(stderr, "0x%02x, ", pre[i]);
	}
	fprintf(stderr, "\n");
	pre += stride;
      }
      fprintf(stderr, "correct :\n");
      for (j = 0; j < len; j += stride) {
	for (i = 0; i < 16; i++) {
	  fprintf(stderr, "0x%02x, ", dst[i]);
	}
	fprintf(stderr, "\n");
	dst += stride;
      }
      fprintf(stderr, "faulty :\n");
      for (j = 0; j < len; j += stride) {
	for (i = 0; i < 16; i++) {
	  fprintf(stderr, "0x%02x,", tmp[i]);
	}
	fprintf(stderr, "\n");
	tmp += stride;
      }
      exit(1);
    }
    free(tmp);
}

#endif
av_cold void ff_vp78dsp_init_aarch64(VP8DSPContext *dsp)
{
    dsp->put_vp8_epel_pixels_tab[0][0][0] = ff_put_vp8_pixels16_neon;
    dsp->put_vp8_epel_pixels_tab[0][2][2] = ff_put_vp8_epel16_h6v6_neon;

    dsp->put_vp8_epel_pixels_tab[1][2][2] = ff_put_vp8_epel8_h6v6_neon;

    dsp->put_vp8_epel_pixels_tab[1][0][0] = ff_put_vp8_pixels8_neon;
    dsp->put_vp8_epel_pixels_tab[1][2][2] = ff_put_vp8_epel8_h6v6_neon;
    dsp->put_vp8_epel_pixels_tab[1][2][1] = ff_put_vp8_epel8_h4v6_neon;
    dsp->put_vp8_epel_pixels_tab[1][1][2] = ff_put_vp8_epel8_h6v4_neon;
}

av_cold void ff_vp8dsp_init_aarch64(VP8DSPContext *dsp)
{
#if 0
    dsp->vp8_luma_dc_wht    = ff_vp8_luma_dc_wht_neon;

    dsp->vp8_idct_add       = ff_vp8_idct_add_neon;
#endif
    dsp->vp8_idct_dc_add    = ff_vp8_idct_dc_add_neon;
#if 0
    dsp->vp8_idct_dc_add4y  = ff_vp8_idct_dc_add4y_neon;
    dsp->vp8_idct_dc_add4uv = ff_vp8_idct_dc_add4uv_neon;
#endif
    dsp->vp8_h_loop_filter16y = ff_vp8_h_loop_filter16_neon;
    dsp->vp8_v_loop_filter16y = ff_vp8_v_loop_filter16_neon;
    dsp->vp8_v_loop_filter8uv = ff_vp8_v_loop_filter8uv_neon;
    dsp->vp8_h_loop_filter8uv = ff_vp8_h_loop_filter8uv_neon;

    dsp->vp8_v_loop_filter16y_inner = ff_vp8_v_loop_filter16_inner_neon;
    dsp->vp8_h_loop_filter16y_inner = ff_vp8_h_loop_filter16_inner_neon;
    dsp->vp8_v_loop_filter8uv_inner = ff_vp8_v_loop_filter8uv_inner_neon;
    dsp->vp8_h_loop_filter8uv_inner = ff_vp8_h_loop_filter8uv_inner_neon;

    dsp->vp8_v_loop_filter_simple = ff_vp8_v_loop_filter16_simple_neon;
    dsp->vp8_h_loop_filter_simple = ff_vp8_h_loop_filter16_simple_neon;
}
