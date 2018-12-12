/*
 * VP8 NEON optimisations
 *
 * Copyright (c) 2010 Rob Clark <rob@ti.com>
 * Copyright (c) 2011 Mans Rullgard <mans@mansr.com>
 * Copyright (c) 2018 Magnus Röös <mla2.roos@gmail.com>
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

/*
function ff_vp8_luma_dc_wht_neon, export=1
        ld1             {v0.8h-v1.8h},  [x1]
        movi            v15.8h, #0

        rev64           v1.d2, v1.d2
        add             v2.8h, v0.8h, v1.8h
        st1             {v15.16b},    [x1], #32
        sub             v3.8h, v0.8h, v1.8h
        st1             {v15.16b},    [x1],
        

        vmov.i16        q8, #3

        vtrn.32         d0,  d2
        vtrn.32         d1,  d3
        vtrn.16         d0,  d1
        vtrn.16         d2,  d3

        vadd.i16        d0,  d0,  d16

        vadd.i16        d4,  d0,  d3
        vadd.i16        d6,  d1,  d2
        vsub.i16        d7,  d1,  d2
        vsub.i16        d5,  d0,  d3
        vadd.i16        q0,  q2,  q3
        vsub.i16        q1,  q2,  q3

        vshr.s16        q0,  q0,  #3
        vshr.s16        q1,  q1,  #3

        mov             w3,  #32
        st1         {v0.8h[0]},  [x0], w3
        st1         {v0.8h[4]},  [x0], w3
        st1         {v1.8h[0]},  [x0], w3
        st1         {v1.8h[4]},  [x0], w3
        st1         {v0.8h[1]},  [x0], w3
        st1         {v0.8h[5]},  [x0], w3
        st1         {v1.8h[1]},  [x0], w3
        st1         {v1.8h[5]},  [x0], w3
        st1         {v0.8h[2]},  [x0], w3
        st1         {v0.8h[6]},  [x0], w3
        st1         {v1.8h[2]},  [x0], w3
        st1         {v1.8h[6]},  [x0], w3
        st1         {v0.8h[3]},  [x0], w3
        st1         {v0.8h[7]},  [x0], w3
        st1         {v1.8h[3]},  [x0], w3
        st1         {v1.8h[7]},  [x0],

        ret
endfunc
*/
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

// Register layout:
//   P3..Q3 -> v0..v7
//   flim_E -> v22
//   flim_I -> v23
//   hev_thresh -> x5
//
.macro  vp8_loop_filter, inner=0, simple=0, hev_thresh
    .if \simple
        uabd            v17.16b, v3.16b,  v4.16b      // abs(P0-Q0)
        uabd            v23.16b, v2.16b,  v5.16b      // abs(P1-Q1)
        uqadd           v17.16b, v17.16b, v17.16b     // abs(P0-Q0) * 2
        ushr            v18.16b, v23.16b, #1          // abs(P1-Q1) / 2
        uqadd           v19.16b, v17.16b,  v18.16b    // (abs(P0-Q0)*2) + (abs(P1-Q1)/2)
        movi            v21.16b, #0x80
        cmhs            v16.16b, v22.16b, v19.16b    // (abs(P0-Q0)*2) + (abs(P1-Q1)/2) <= flim
    .else
        // calculate hev and normal_limit:
        uabd            v20.16b, v2.16b,  v3.16b      // abs(P1-P0)
        uabd            v21.16b, v5.16b,  v4.16b      // abs(Q1-Q0)
        uabd            v18.16b, v0.16b,  v1.16b      // abs(P3-P2)
        uabd            v19.16b, v1.16b,  v2.16b      // abs(P2-P1)
        cmhs            v16.16b, v23.16b, v20.16b     // abs(P1-P0) <= flim_I
        cmhs            v17.16b, v23.16b, v21.16b     // abs(Q1-Q0) <= flim_I
        cmhs            v18.16b, v23.16b, v18.16b     // abs(P3-P2) <= flim_I
        cmhs            v19.16b, v23.16b, v19.16b     // abs(P2-P1) <= flim_I
        and             v16.16b, v17.16b, v16.16b
        uabd            v17.16b, v7.16b,  v6.16b      // abs(Q3-Q2)
        and             v16.16b, v16.16b, v19.16b
        uabd            v19.16b, v6.16b,  v5.16b      // abs(Q2-Q1)
        and             v16.16b, v16.16b, v18.16b
        cmhs            v18.16b, v23.16b, v17.16b     // abs(Q3-Q2) <= flim_I
        cmhs            v19.16b, v23.16b, v19.16b     // abs(Q2-Q1) <= flim_I
        uabd            v17.16b, v3.16b,  v4.16b      // abs(P0-Q0)
        uabd            v23.16b, v2.16b,  v5.16b      // abs(P1-Q1)
        and             v16.16b, v16.16b, v18.16b
        uqadd           v17.16b, v17.16b, v17.16b     // abs(P0-Q0) * 2
        and             v16.16b, v16.16b, v19.16b
        ushr            v18.16b, v23.16b, #1          // abs(P1-Q1) / 2
        dup             v23.16b, \hev_thresh          // hev_thresh
        uqadd           v19.16b, v17.16b, v18.16b     // (abs(P0-Q0)*2) + (abs(P1-Q1)/2)
        cmhi            v20.16b, v20.16b, v23.16b     // abs(P1-P0) > hev_thresh
        cmhs            v19.16b, v22.16b, v19.16b     // (abs(P0-Q0)*2) + (abs(P1-Q1)/2) <= flim_E
        cmhi            v22.16b, v21.16b, v23.16b     // abs(Q1-Q0) > hev_thresh
        and             v16.16b, v16.16b, v19.16b
        movi            v21.16b, #0x80
        orr             v17.16b,  v20.16b, v22.16b
    .endif

        // at this point:
        //   v16: normal_limit
        //   v17: hev

        // convert to signed value:
        eor            v3.16b, v3.16b, v21.16b           // PS0 = P0 ^ 0x80
        eor            v4.16b, v4.16b, v21.16b           // QS0 = Q0 ^ 0x80

        movi           v20.8h, #3
        ssubl          v18.8h, v4.8b,  v3.8b             // QS0 - PS0
        ssubl2         v19.8h, v4.16b, v3.16b            //   (widened to 16bit)
        eor            v2.16b, v2.16b, v21.16b           // PS1 = P1 ^ 0x80
        eor            v5.16b, v5.16b, v21.16b           // QS1 = Q1 ^ 0x80
        mul            v18.8h, v18.8h, v20.8h            // w = 3 * (QS0 - PS0)
        mul            v19.8h, v19.8h, v20.8h

        sqsub          v20.16b, v2.16b, v5.16b           // clamp(PS1-QS1)
        movi           v22.16b, #4
        movi           v23.16b, #3
    .if \inner
        and            v20.16b, v20.16b, v17.16b         // if(hev) w += clamp(PS1-QS1)
    .endif
        saddw          v18.8h,  v18.8h, v20.8b           // w += clamp(PS1-QS1)
        saddw2         v19.8h,  v19.8h, v20.16b
        sqxtn          v18.8b,  v18.8h                   // narrow result back into v18
        sqxtn2         v18.16b, v19.8h
    .if !\inner && !\simple
        eor            v1.16b,  v1.16b,  v21.16b         // PS2 = P2 ^ 0x80
        eor            v6.16b,  v6.16b,  v21.16b         // QS2 = Q2 ^ 0x80
    .endif
        and            v18.16b, v18.16b, v16.16b         // w &= normal_limit

        // registers used at this point..
        //   v0 -> P3  (don't corrupt)
        //   v1-v6 -> PS2-QS2
        //   v7 -> Q3  (don't corrupt)
        //   v17 -> hev
        //   v18 -> w
        //   v21 -> #0x80
        //   v22 -> #4
        //   v23 -> #3
        //   v16, v19, v29 -> unused
        //
        // filter_common:   is4tap==1
        //   c1 = clamp(w + 4) >> 3;
        //   c2 = clamp(w + 3) >> 3;
        //   Q0 = s2u(QS0 - c1);
        //   P0 = s2u(PS0 + c2);

    .if \simple
        sqadd          v19.16b, v18.16b, v22.16b           // c1 = clamp((w&hev)+4)
        sqadd          v20.16b, v18.16b, v23.16b           // c2 = clamp((w&hev)+3)
        sshr           v19.16b, v19.16b, #3                // c1 >>= 3
        sshr           v20.16b, v20.16b, #3                // c2 >>= 3
        sqsub          v4.16b,  v4.16b,  v19.16b           // QS0 = clamp(QS0-c1)
        sqadd          v3.16b,  v3.16b,  v20.16b           // PS0 = clamp(PS0+c2)
        eor            v4.16b,  v4.16b,  v21.16b           // Q0 = QS0 ^ 0x80
        eor            v3.16b,  v3.16b,  v21.16b           // P0 = PS0 ^ 0x80
        eor            v5.16b,  v5.16b,  v21.16b           // Q1 = QS1 ^ 0x80
        eor            v2.16b,  v2.16b,  v21.16b           // P1 = PS1 ^ 0x80
    .elseif \inner
        // the !is4tap case of filter_common, only used for inner blocks
        //   c3 = ((c1&~hev) + 1) >> 1;
        //   Q1 = s2u(QS1 - c3);
        //   P1 = s2u(PS1 + c3);
        sqadd          v19.16b, v18.16b, v22.16b           // c1 = clamp((w&hev)+4)
        sqadd          v20.16b, v18.16b, v23.16b           // c2 = clamp((w&hev)+3)
        sshr           v19.16b, v19.16b, #3                // c1 >>= 3
        sshr           v20.16b, v20.16b, #3                // c2 >>= 3
        sqsub          v4.16b,  v4.16b,  v19.16b           // QS0 = clamp(QS0-c1)
        sqadd          v3.16b,  v3.16b,  v20.16b           // PS0 = clamp(PS0+c2)
        bic            v19.16b, v19.16b, v17.16b           // c1 & ~hev
        eor            v4.16b,  v4.16b,  v21.16b           // Q0 = QS0 ^ 0x80
        srshr          v19.16b, v19.16b, #1                // c3 >>= 1
        eor            v3.16b,  v3.16b,  v21.16b           // P0 = PS0 ^ 0x80
        sqsub          v5.16b,  v5.16b,  v19.16b           // QS1 = clamp(QS1-c3)
        sqadd          v2.16b,  v2.16b,  v19.16b           // PS1 = clamp(PS1+c3)
        eor            v5.16b,  v5.16b,  v21.16b           // Q1 = QS1 ^ 0x80
        eor            v2.16b,  v2.16b,  v21.16b           // P1 = PS1 ^ 0x80
    .else
        and            v20.16b, v18.16b, v17.16b           // w & hev
        sqadd          v19.16b, v20.16b, v22.16b           // c1 = clamp((w&hev)+4)
        sqadd          v20.16b, v20.16b, v23.16b           // c2 = clamp((w&hev)+3)
        sshr           v19.16b, v19.16b, #3                // c1 >>= 3
        sshr           v20.16b, v20.16b, #3                // c2 >>= 3
        bic            v18.16b, v18.16b, v17.16b           // w &= ~hev
        sqsub          v4.16b,  v4.16b,  v19.16b           // QS0 = clamp(QS0-c1)
        sqadd          v3.16b,  v3.16b,  v20.16b           // PS0 = clamp(PS0+c2)

        // filter_mbedge:
        //   a = clamp((27*w + 63) >> 7);
        //   Q0 = s2u(QS0 - a);
        //   P0 = s2u(PS0 + a);
        //   a = clamp((18*w + 63) >> 7);
        //   Q1 = s2u(QS1 - a);
        //   P1 = s2u(PS1 + a);
        //   a = clamp((9*w + 63) >> 7);
        //   Q2 = s2u(QS2 - a);
        //   P2 = s2u(PS2 + a);
        movi           v17.8h,  #63
        sshll          v22.8h, v18.8b, #3
        sshll2         v23.8h, v18.16b, #3
        saddw          v22.8h, v22.8h, v18.8b
        saddw2         v23.8h, v23.8h, v18.16b
        add            v16.8h, v17.8h, v22.8h
        add            v17.8h, v17.8h, v23.8h           //  9*w + 63
        add            v19.8h, v16.8h, v22.8h
        add            v20.8h, v17.8h, v23.8h           // 18*w + 63
        add            v22.8h, v19.8h, v22.8h
        add            v23.8h, v20.8h, v23.8h           // 27*w + 63
        sqshrn         v16.8b, v16.8h,  #7
        sqshrn2        v16.16b, v17.8h, #7              // clamp(( 9*w + 63)>>7)
        sqshrn         v19.8b,  v19.8h, #7
        sqshrn2        v19.16b, v20.8h, #7              // clamp((18*w + 63)>>7)
        sqshrn         v22.8b,  v22.8h, #7
        sqshrn2        v22.16b, v23.8h, #7              // clamp((27*w + 63)>>7)
        sqadd          v1.16b,  v1.16b,  v16.16b        // PS2 = clamp(PS2+a)
        sqsub          v6.16b,  v6.16b,  v16.16b        // QS2 = clamp(QS2-a)
        sqadd          v2.16b,  v2.16b,  v19.16b        // PS1 = clamp(PS1+a)
        sqsub          v5.16b,  v5.16b,  v19.16b        // QS1 = clamp(QS1-a)
        sqadd          v3.16b,  v3.16b,  v22.16b        // PS0 = clamp(PS0+a)
        sqsub          v4.16b,  v4.16b,  v22.16b        // QS0 = clamp(QS0-a)
        eor            v3.16b,  v3.16b,  v21.16b        // P0 = PS0 ^ 0x80
        eor            v4.16b,  v4.16b,  v21.16b        // Q0 = QS0 ^ 0x80
        eor            v2.16b,  v2.16b,  v21.16b        // P1 = PS1 ^ 0x80
        eor            v5.16b,  v5.16b,  v21.16b        // Q1 = QS1 ^ 0x80
        eor            v1.16b,  v1.16b,  v21.16b        // P2 = PS2 ^ 0x80
        eor            v6.16b,  v6.16b,  v21.16b        // Q2 = QS2 ^ 0x80
    .endif
.endm

.macro  vp8_v_loop_filter16 name, inner=0, simple=0
function ff_vp8_v_loop_filter16\name\()_neon, export=1
        sub             x0,  x0,  x1,  lsl #1+!\simple

        // Load pixels:
    .if !\simple
        ld1             {v0.16b},     [x0], x1 // P3
        ld1             {v1.16b},     [x0], x1 // P2
    .endif
        ld1             {v2.16b},     [x0], x1 // P1
        ld1             {v3.16b},     [x0], x1 // P0
        ld1             {v4.16b},     [x0], x1 // Q0
        ld1             {v5.16b},     [x0], x1 // Q1
    .if !\simple
        ld1             {v6.16b},     [x0], x1 // Q2
        ld1             {v7.16b},     [x0]     // Q3
        dup             v23.16b, w3                 // flim_I
    .endif
        dup             v22.16b, w2                 // flim_E

        vp8_loop_filter inner=\inner, simple=\simple, hev_thresh=w4

        // back up to P2:  dst -= stride * 6
        sub             x0,  x0,  x1,  lsl #2
    .if !\simple
        sub             x0,  x0,  x1,  lsl #1

        // Store pixels:
        st1             {v1.16b},     [x0], x1 // P2
    .endif
        st1          {v2.16b},     [x0], x1 // P1
        st1          {v3.16b},     [x0], x1 // P0
        st1          {v4.16b},     [x0], x1 // Q0
        st1          {v5.16b},     [x0], x1 // Q1
    .if !\simple
        st1          {v6.16b},     [x0]     // Q2
    .endif

        ret
endfunc
.endm

vp8_v_loop_filter16
vp8_v_loop_filter16 _inner,  inner=1
vp8_v_loop_filter16 _simple, simple=1

.macro  vp8_v_loop_filter8uv name, inner=0
function ff_vp8_v_loop_filter8uv\name\()_neon, export=1
        sub             x0,  x0,  x2,  lsl #2
        sub             x1,  x1,  x2,  lsl #2
        // Load pixels:
        ld1          {v0.d}[0],     [x0], x2  // P3
        ld1          {v0.d}[1],     [x1], x2  // P3
        ld1          {v1.d}[0],     [x0], x2  // P2
        ld1          {v1.d}[1],     [x1], x2  // P2
        ld1          {v2.d}[0],     [x0], x2  // P1
        ld1          {v2.d}[1],     [x1], x2  // P1
        ld1          {v3.d}[0],     [x0], x2  // P0
        ld1          {v3.d}[1],     [x1], x2  // P0
        ld1          {v4.d}[0],     [x0], x2  // Q0
        ld1          {v4.d}[1],     [x1], x2  // Q0
        ld1          {v5.d}[0],     [x0], x2  // Q1
        ld1          {v5.d}[1],     [x1], x2  // Q1
        ld1          {v6.d}[0],     [x0], x2  // Q2
        ld1          {v6.d}[1],     [x1], x2  // Q2
        ld1          {v7.d}[0],     [x0]      // Q3
        ld1          {v7.d}[1],     [x1]      // Q3
        
        dup          v22.16b, w3                 // flim_E
        dup          v23.16b, w4                 // flim_I

        vp8_loop_filter inner=\inner, hev_thresh=w5

        // back up to P2:  u,v -= stride * 6
        sub             x0,  x0,  x2,  lsl #2
        sub             x1,  x1,  x2,  lsl #2
        sub             x0,  x0,  x2,  lsl #1
        sub             x1,  x1,  x2,  lsl #1

        // Store pixels:

        st1          {v1.d}[0],     [x0], x2  // P2
        st1          {v1.d}[1],     [x1], x2  // P2
        st1          {v2.d}[0],     [x0], x2  // P1
        st1          {v2.d}[1],     [x1], x2  // P1
        st1          {v3.d}[0],     [x0], x2  // P0
        st1          {v3.d}[1],     [x1], x2  // P0
        st1          {v4.d}[0],     [x0], x2  // Q0
        st1          {v4.d}[1],     [x1], x2  // Q0
        st1          {v5.d}[0],     [x0], x2  // Q1
        st1          {v5.d}[1],     [x1], x2  // Q1
        st1          {v6.d}[0],     [x0]      // Q2
        st1          {v6.d}[1],     [x1]      // Q2

        ret
endfunc
.endm

vp8_v_loop_filter8uv
vp8_v_loop_filter8uv _inner, inner=1

        
.macro  vp8_h_loop_filter16 name, inner=0, simple=0
function ff_vp8_h_loop_filter16\name\()_neon, export=1

        sub             x0,  x0,  #4
        // Load pixels:
        ld1             {v0.d}[0], [x0], x1
        ld1             {v1.d}[0], [x0], x1
        ld1             {v2.d}[0], [x0], x1
        ld1             {v3.d}[0], [x0], x1
        ld1             {v4.d}[0], [x0], x1
        ld1             {v5.d}[0], [x0], x1
        ld1             {v6.d}[0], [x0], x1
        ld1             {v7.d}[0], [x0], x1
        ld1             {v0.d}[1], [x0], x1
        ld1             {v1.d}[1], [x0], x1
        ld1             {v2.d}[1], [x0], x1
        ld1             {v3.d}[1], [x0], x1
        ld1             {v4.d}[1], [x0], x1
        ld1             {v5.d}[1], [x0], x1
        ld1             {v6.d}[1], [x0], x1
        ld1             {v7.d}[1], [x0], x1

        transpose_8x16b   v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7, v30, v31

        dup          v22.16b, w2                 // flim_E
    .if !\simple
        dup          v23.16b, w3                 // flim_I
    .endif

        vp8_loop_filter inner=\inner, simple=\simple, hev_thresh=w4

        sub             x0,  x0,  x1, lsl #4    // backup 16 rows

        transpose_8x16b   v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7, v30, v31

        // Store pixels:
        st1             {v0.d}[0], [x0], x1
        st1             {v1.d}[0], [x0], x1
        st1             {v2.d}[0], [x0], x1
        st1             {v3.d}[0], [x0], x1
        st1             {v4.d}[0], [x0], x1
        st1             {v5.d}[0], [x0], x1
        st1             {v6.d}[0], [x0], x1
        st1             {v7.d}[0], [x0], x1
        st1             {v0.d}[1], [x0], x1
        st1             {v1.d}[1], [x0], x1
        st1             {v2.d}[1], [x0], x1
        st1             {v3.d}[1], [x0], x1
        st1             {v4.d}[1], [x0], x1
        st1             {v5.d}[1], [x0], x1
        st1             {v6.d}[1], [x0], x1
        st1             {v7.d}[1], [x0]

        ret
endfunc
.endm

vp8_h_loop_filter16
vp8_h_loop_filter16 _inner,  inner=1
vp8_h_loop_filter16 _simple, simple=1

.macro  vp8_h_loop_filter8uv name, inner=0
function ff_vp8_h_loop_filter8uv\name\()_neon, export=1
        sub             x0,  x0,  #4
        sub             x1,  x1,  #4

        // Load pixels:
        ld1          {v0.d}[0],     [x0], x2 // load u 
        ld1          {v0.d}[1],     [x1], x2 // load v 
        ld1          {v1.d}[0],     [x0], x2  
        ld1          {v1.d}[1],     [x1], x2  
        ld1          {v2.d}[0],     [x0], x2  
        ld1          {v2.d}[1],     [x1], x2  
        ld1          {v3.d}[0],     [x0], x2  
        ld1          {v3.d}[1],     [x1], x2  
        ld1          {v4.d}[0],     [x0], x2  
        ld1          {v4.d}[1],     [x1], x2  
        ld1          {v5.d}[0],     [x0], x2  
        ld1          {v5.d}[1],     [x1], x2  
        ld1          {v6.d}[0],     [x0], x2  
        ld1          {v6.d}[1],     [x1], x2  
        ld1          {v7.d}[0],     [x0], x2      
        ld1          {v7.d}[1],     [x1], x2      

        transpose_8x16b   v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7, v30, v31

        dup             v22.16b, w3                 // flim_E
        dup             v23.16b, w4                 // flim_I

        vp8_loop_filter inner=\inner, hev_thresh=w5

        sub             x0,  x0,  x2, lsl #3    // backup u 8 rows
        sub             x1,  x1,  x2, lsl #3    // backup v 8 rows

        transpose_8x16b   v0,  v1,  v2,  v3,  v4,  v5,  v6,  v7, v30, v31

        // Store pixels:
        st1          {v0.d}[0],     [x0], x2 // load u 
        st1          {v0.d}[1],     [x1], x2 // load v 
        st1          {v1.d}[0],     [x0], x2
        st1          {v1.d}[1],     [x1], x2
        st1          {v2.d}[0],     [x0], x2
        st1          {v2.d}[1],     [x1], x2
        st1          {v3.d}[0],     [x0], x2
        st1          {v3.d}[1],     [x1], x2
        st1          {v4.d}[0],     [x0], x2
        st1          {v4.d}[1],     [x1], x2
        st1          {v5.d}[0],     [x0], x2
        st1          {v5.d}[1],     [x1], x2
        st1          {v6.d}[0],     [x0], x2
        st1          {v6.d}[1],     [x1], x2
        st1          {v7.d}[0],     [x0]
        st1          {v7.d}[1],     [x1]

        ret

endfunc
.endm

vp8_h_loop_filter8uv
vp8_h_loop_filter8uv _inner, inner=1


function ff_put_vp8_pixels16_neon, export=1
1:
        subs            w4, w4, #4
        ld1             {v0.16b},     [x2], x3
        ld1             {v1.16b},     [x2], x3
        ld1             {v2.16b},     [x2], x3
        ld1             {v3.16b},     [x2], x3
        st1             {v0.16b},     [x0], x1
        st1             {v1.16b},     [x0], x1
        st1             {v2.16b},     [x0], x1
        st1             {v3.16b},     [x0], x1
        bgt             1b
        ret
endfunc

function ff_put_vp8_pixels8_neon, export=1
1:
        subs            w4, w4, #4
        ld1             {v0.8b},   [x2], x3
        ld1             {v0.d}[1], [x2], x3
        ld1             {v1.8b},   [x2], x3
        ld1             {v1.d}[1], [x2], x3
        st1             {v0.8b},   [x0], x1
        st1             {v0.d}[1], [x0], x1
        st1             {v1.8b},   [x0], x1
        st1             {v1.d}[1], [x0], x1
        bgt             1b
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

.macro  vp8_epel8_v6    d0,  s0,  s1,  s2, s3, s4, s5
        uxtl            \s2\().8h, \s2\().8b
        uxtl            \s3\().8h, \s3\().8b
        uxtl            \s1\().8h, \s1\().8b
        uxtl            \s4\().8h, \s4\().8b
        uxtl            \s0\().8h, \s0\().8b
        uxtl            \s5\().8h, \s5\().8b
        mul             \s2\().8h, \s2\().8h, v0.8h[2]
        mul             \s3\().8h, \s3\().8h, v0.8h[3]
        mls             \s2\().8h, \s1\().8h, v0.8h[1]
        mls             \s3\().8h, \s4\().8h, v0.8h[4]
        mla             \s2\().8h, \s0\().8h, v0.8h[0]
        mla             \s3\().8h, \s5\().8h, v0.8h[5]
        sqadd           \s3\().8h, \s2\().8h, \s3\().8h
        sqrshrun        \d0\().8b, \s3\().8h, #7
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
        ld1             {v1.8b - v4.8b},    [x7], #32
        ld1             {v16.8b - v19.8b},[x7], #32
        ld1             {v20.8b - v23.8b},[x7]
        sub             x7,  x7,  #48

        vp8_epel8_v6    v5, v1, v3, v16, v18, v20, v22
        vp8_epel8_v6    v2, v2, v4, v17, v19, v21, v23
        trn1            v2.2d, v5.2d, v2.2d

        st1             {v2.16b}, [x0], x1
        subs            x4, x4, #1
        bne             2b

        add             sp,  sp,  #336+16
        ret
endfunc
        
