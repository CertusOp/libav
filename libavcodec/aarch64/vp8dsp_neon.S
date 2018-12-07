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

.macro  vp8_epel16_h6   d0,  v0,  v1
        ext             v22.16b, \v0\().16b, \v1\().16b, #3
        ext             v23.16b, \v0\().16b, \v1\().16b, #4
        uxtl            v19.8h,  v22.8b
        uxtl2           v22.8h,  v22.16b
        ext             v3.16b,  \v0\().16b, \v1\().16b, #2
        uxtl            v20.8h,  v23.8b
        uxtl2           v23.8h,  v23.16b
        ext             v16.16b, \v0\().16b, \v1\().16b, #1
        uxtl            v18.8h,  v3.8b
        uxtl2           v3.8h,   v3.16b
        ext             v2.16b,  \v0\().16b, \v1\().16b, #5
        uxtl            v21.8h,  v2.8b
        uxtl2           v2.8h,   v2.16b
        uxtl            v17.8h,  v16.8b
        uxtl2           v16.8h,  v16.16b
        mul             v19.8h,  v19.8h, v0.8h[3]
        mul             v18.8h,  v18.8h, v0.8h[2]
        mul             v3.8h,   v3.8h,  v0.8h[2]
        mul             v22.8h,  v22.8h, v0.8h[3]
        mls             v19.8h,  v20.8h, v0.8h[4]
        uxtl            v20.8h,  \v0\().8b
        uxtl2           v1.8h,   \v0\().16b
        mls             v18.8h,  v17.8h, v0.8h[1]
        mls             v3.8h,   v16.8h, v0.8h[1]
        mls             v22.8h,  v23.8h, v0.8h[4]
        mla             v18.8h,  v20.8h, v0.8h[0]
        mla             v19.8h,  v21.8h, v0.8h[5]
        mla             v3.8h,   v1.8h,  v0.8h[0]
        mla             v22.8h,  v2.8h,  v0.8h[5]
        sqadd           v19.8h,  v18.8h, v19.8h
        sqadd           v22.8h,  v3.8h,  v22.8h
        sqrshrun        \d0\().8b, v19.8h, #7
        sqrshrun        v22.8b,    v22.8h, #7
        trn1            \d0\().2d, \d0\().2d, v22.2d
        
.endm

.macro  vp8_epel8_v6    d0,  s0,  s1,  s2
        uxtl            v18.8h, \s1\().8b
        uxtl2           v19.8h, \s1\().16b
        uxtl2           v17.8h, \s0\().16b
        uxtl            v20.8h, \s2\().8b
        uxtl            v16.8h, \s0\().8b
        uxtl2           v21.8h, \s2\().16b
        mul             v18.8h, v18.8h, v0.8h[2]
        mul             v19.8h, v19.8h, v0.8h[3]
        mls             v18.8h, v17.8h, v0.8h[1]
        mls             v19.8h, v20.8h, v0.8h[4]
        mla             v18.8h, v16.8h, v0.8h[0]
        mla             v19.8h, v21.8h, v0.8h[5]
        sqadd           v19.8h, v18.8h, v19.8h
        sqrshrun        \d0\().8b, v19.8h, #7
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
        movrel          x17,  subpel_filters-16
        sxtw            x5,  w5
        add             x16,  x17,  x5, lsl #4 // x
        sub             sp,  sp,  #336+16
        ld1             {v0.8h},  [x16]
        add             x7,  sp,  #15
        sxtw            x4,  w4	
        add             x16, x4, #5   // h
        bic             x7,  x7,  #15
1:
        ld1             {v1.16b, v2.16b}, [x2], x3
        vp8_epel16_h6   v1, v1, v2
        st1             {v1.16b}, [x7], #16
        subs            x16, x16, #1
        bne             1b


        // second pass (vertical):
        sxtw            x6,  w6 
        add             x6,  x17,  x6, lsl #4  // y
        add             x7,  sp,  #15
        ld1             {v0.8h},     [x6]
        bic             x7,  x7,  #15
2:
        ld1             {v1.16b, v2.16b},  [x7], #32
        ld1             {v3.16b, v4.16b},  [x7], #32
        ld1             {v22.16b, v23.16b},[x7]
        sub             x7,  x7,  #48

        trn1            v5.2d, v1.2d, v2.2d
        trn2            v2.2d, v1.2d, v2.2d

        trn1            v1.2d, v3.2d, v4.2d
        trn2            v4.2d, v3.2d, v4.2d

        trn1            v3.2d, v22.2d, v23.2d
        trn2            v23.2d, v22.2d, v23.2d
        
        vp8_epel8_v6    v5, v5, v1, v3
        vp8_epel8_v6    v2, v2, v4, v23
        trn1            v2.2d, v5.2d, v2.2d

        st1             {v2.16b}, [x0], x1
        subs            x4, x4, #1
        bne             2b

        add             sp,  sp,  #336+16
        ret
endfunc
        
