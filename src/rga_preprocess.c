/*
 * RGA Hardware Accelerated Preprocessing Implementation
 * Uses Rockchip RGA2/RGA3 for resize + BGR->RGB + padding
 * Uses C API (im*_t functions) for C file compatibility
 */

#include "rga_preprocess.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <rga/im2d_type.h>
#include <rga/im2d_single.h>
#include <rga/im2d_buffer.h>
#include <rga/rga.h>

static bool rga_initialized = false;

int rga_preprocess_init(void) {
    rga_initialized = true;
    printf("RGA hardware preprocessing initialized\n");
    return 0;
}

void rga_preprocess_deinit(void) {
    rga_initialized = false;
}

bool rga_preprocess_available(void) {
    return rga_initialized;
}

int rga_letterbox(const uint8_t* src, int src_w, int src_h,
                  uint8_t* dst_buf, int dst_w, int dst_h,
                  float* scale, float* pad_left, float* pad_top) {
    if (!rga_initialized) {
        return -1;
    }

    // Calculate scale and padding (same as OpenCV version)
    float scale_x = (float)dst_w / src_w;
    float scale_y = (float)dst_h / src_h;
    float s = fminf(scale_x, scale_y);
    
    int new_w = (int)(src_w * s);
    int new_h = (int)(src_h * s);
    
    float x_offset = (dst_w - new_w) / 2.0f;
    float y_offset = (dst_h - new_h) / 2.0f;
    
    if (scale) *scale = s;
    if (pad_left) *pad_left = x_offset;
    if (pad_top) *pad_top = y_offset;

    // Fill background with gray (114, 114, 114)
    memset(dst_buf, 114, dst_w * dst_h * 3);

    // Create RGA buffers using C API
    // Source: BGR888 (OpenCV default)
    rga_buffer_t src_buf = wrapbuffer_virtualaddr_t((void*)src, src_w, src_h, src_w, src_h, RK_FORMAT_BGR_888);
    
    // Create a temporary buffer for the resized BGR image
    uint8_t* temp_bgr_buf = (uint8_t*)malloc(new_w * new_h * 3);
    if (!temp_bgr_buf) {
        return -1;
    }
    
    // Step 1: Resize BGR -> BGR (RGA handles resize best in same format)
    rga_buffer_t temp_bgr_rga = wrapbuffer_virtualaddr_t(temp_bgr_buf, new_w, new_h, new_w, new_h, RK_FORMAT_BGR_888);
    IM_STATUS status = imresize_t(src_buf, temp_bgr_rga, 0, 0, IM_INTERP_LINEAR, 1);
    if (status != IM_STATUS_SUCCESS) {
        fprintf(stderr, "RGA resize failed: %d\n", status);
        free(temp_bgr_buf);
        return -1;
    }
    
    // Step 2: Color convert BGR -> RGB and copy to destination with padding
    // RGA color convert from temp BGR buffer to destination sub-region
    int x_off = (int)x_offset;
    int y_off = (int)y_offset;
    
    // For now, do color conversion manually to ensure correctness
    // BGR -> RGB: swap R and B channels
    for (int y = 0; y < new_h; y++) {
        uint8_t* dst_row = dst_buf + ((y_off + y) * dst_w + x_off) * 3;
        uint8_t* src_row = temp_bgr_buf + y * new_w * 3;
        for (int x = 0; x < new_w; x++) {
            // BGR -> RGB: swap B and R
            dst_row[x * 3 + 0] = src_row[x * 3 + 2]; // R
            dst_row[x * 3 + 1] = src_row[x * 3 + 1]; // G
            dst_row[x * 3 + 2] = src_row[x * 3 + 0]; // B
        }
    }
    
    free(temp_bgr_buf);
    return 0;
}
