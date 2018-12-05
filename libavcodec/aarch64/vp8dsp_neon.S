/*
 * VP8 NEON optimisations
 *
 * Copyright (c) 2010 Rob Clark <rob@ti.com>
 * Copyright (c) 2011 Mans Rullgard <mans@mansr.com>
 *
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

#include "libavutil/aarch64/asm.S"
#include "neon.S"


function ff_vp8_idct_dc_add_neon, export=1
        mov             w3,       #0
        ld1r            {v2.8h},  [x1]
        strh            w3,       [x1]
        srshr           v2.8h,  v2.8h,  #3
        ld1             {v0.s}[0],  [x0], x2
        ld1             {v0.s}[1],  [x0], x2
        uaddw           v3.8h,  v2.8h,  v0.8b
        ld1             {v1.s}[0],  [x0], x2
        ld1             {v1.s}[1],  [x0], x2
        uaddw           v4.8h,  v2.8h,  v1.8b
        sqxtun          v0.8b,  v3.8h
        sqxtun          v1.8b,  v4.8h
        sub             x0,  x0,  x2, lsl #2
        st1             {v0.s}[0],  [x0], x2
        st1             {v0.s}[1],  [x0], x2
        st1             {v1.s}[0],  [x0], x2
        st1             {v1.s}[1],  [x0], x2
        ret
endfunc

.macro  vp8_epel16_h6   d0,  s0, v0,  v1
        ext             v14.16b, \v0\().16b, \v1\().16b, #3
        ext             v15.16b, \v0\().16b, \v1\().16b, #4
        uxtl        	v11.8h, v14.8b
        uxtl2        	v14.8h, v14.16b
        ext             v3.16b,  \v0\().16b, \v1\().16b, #2
        uxtl	        v12.8h, v15.8b
        uxtl2           v15.8h, v15.16b
        ext	        v8.16b,  \v0\().16b, \v1\().16b, #1
        uxtl            v10.8h, v3.8b
        uxtl2           v3.8h,  v3.16b
        ext             v2.16b,  \v0\().16b, \v1\().16b, #5
        uxtl        	v13.8h, v2.8b
        uxtl2        	v2.8h,  v2.16b
        uxtl        	v9.8h,  v8.8b
        uxtl2        	v8.8h,  v8.16b
        mul        	v11.8h, v11.8h, v0.8h[3]
        mul        	v10.8h, v10.8h, v0.8h[2]
        mul        	v3.8h,  v3.8h,  v0.8h[2]
        mul        	v14.8h, v14.8h, v0.8h[3]
        mls	        v11.8h, v12.8h, v0.8h[4]
        uxtl	        v12.8h, \s0\().8b
        uxtl2	        v1.8h,  \s0\().16b
        mls	        v10.8h, v9.8h,  v0.8h[1]
        mls	        v3.8h,  v8.8h,  v0.8h[1]
        mls	        v14.8h, v15.8h, v0.8h[4]
        mls	        v10.8h, v12.8h, v0.8h[0]
        mls	        v11.8h, v13.8h, v0.8h[5]
        mls	        v3.8h,  v1.8h,  v0.8h[0]
        mls	        v14.8h, v2.8h,  v0.8h[5]
        sqadd	      	v11.8h, v10.8h, v11.8h
        sqadd		v14.8h, v3.8h,  v14.8h
        sqrshrun    	\d0\().8b, v11.8h, #7
        sqrshrun2    	\d0\().16b, v14.8h, #7
.endm

.macro  vp8_epel8_v6    d0,  s0,  s1,  s2
        uxtl            v10.8h, \s1\().8b
        uxtl2           v11.8h, \s1\().16b
        uxtl2           v9.8h,  \s0\().16b
        uxtl            v12.8h, \s2\().8b
        uxtl            v8.8h,  \s0\().8b
        uxtl2           v13.8h, \s2\().16b
        mul            v10.8h, v10.8h, v0.8h[2]
        mul            v11.8h, v11.8h, v0.8h[3]
        mls            v10.8h, v9.8h,  v0.8h[1]
        mls            v11.8h, v12.8h, v0.8h[4]
        mla            v10.8h, v8.8h,  v0.8h[0]
        mla            v11.8h, v13.8h, v0.8h[5]
        sqadd	       v11.8h, v10.8h, v11.8h
        sqrshrun       \d0\().8b, v11.8h, #7
.endm

// note: worst case sum of all 6-tap filter values * 255 is 0x7f80 so 16 bit
// arithmatic can be used to apply filters
const   subpel_filters, align=4
        .short     0,   6, 123,  12,   1,   0,   0,   0
        .short     2,  11, 108,  36,   8,   1,   0,   0
        .short     0,   9,  93,  50,   6,   0,   0,   0
        .short     3,  16,  77,  77,  16,   3,   0,   0
        .short     0,   6,  50,  93,   9,   0,   0,   0
        .short     1,   8,  36, 108,  11,   2,   0,   0
        .short     0,   1,  12, 123,   6,   0,   0,   0
endconst
	
	
function ff_put_vp8_epel16_h6v6_neon, export=1
        sub             x2,  x2,  x3,  lsl #1
        sub             x2,  x2,  #2

        // first pass (horizontal):
        movrel          x16,  subpel_filters-16
        sxtw            x4,  w4
        add             x16,  x16,  x4, lsl #4 // h
        sub             sp,  sp,  #336+16
        ld1             {v0.16b},  [x16]
        add             x7,  sp,  #15
        add             w5, w5, #5
        bic             x7,  x7,  #15
1:
        ld1             {v1.16b, v2.16b}, [x2], x3
	vp8_epel16_h6   v1, v1, v1, v2
        st1		{v1.16b, v2.16b}, [x7]
        subs            w5, w5, #1
        bne             1b

        // second pass (vertical):
        sxtw            x6,  w6	
        add             x6,  x16,  x6, lsl #4
        add             x7,  sp,  #15
        ld1             {v0.16b},     [x6]
        bic             x7,  x7,  #15
2:
        ld1        	{v1.16b, v2.16b},  [x7]
        ld1          	{v3.16b, v4.16b},  [x7]
        ld1          	{v14.16b, v15.16b},[x7]
        sub             x7,  x7,  #48

	ext		v5.16b, v2.16b, v1.16b, #8
	ext 		v2.16b, v1.16b, v2.16b, #8

	ext		v1.16b, v4.16b, v3.16b, #8
	ext		v4.16b, v3.16b, v4.16b, #8

	ext		v3.16b, v15.16b, v14.16b, #8
	ext		v15.16b, v14.16b, v15.16b, #8
	
        vp8_epel8_v6    v5, v5, v1, v3
        vp8_epel8_v6    v2, v2, v4, v15

        st1             {v5.d}[0], [x0], x1
        st1             {v2.d}[0], [x0], x1
        subs            x4, x4, #1
        bne             2b

        add             sp,  sp,  #336+16
	ret
endfunc
	
