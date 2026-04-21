/*
 * RGA Hardware Accelerated Preprocessing for RK3588
 * Replaces OpenCV CPU-based letterbox with RGA2/RGA3 hardware
 */

#ifndef RGA_PREPROCESS_H
#define RGA_PREPROCESS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize RGA hardware
int rga_preprocess_init(void);

// Cleanup RGA
void rga_preprocess_deinit(void);

// Check if RGA is available
bool rga_preprocess_available(void);

// RGA accelerated letterbox: resize + color convert + padding
// src: source image buffer (BGR, continuous)
// src_w, src_h: source dimensions
// dst_buf: pre-allocated destination buffer (RGB, continuous)
// dst_w, dst_h: target dimensions (e.g., 640x640)
// scale, pad_left, pad_top: output parameters for coordinate mapping
// Returns 0 on success
int rga_letterbox(const uint8_t* src, int src_w, int src_h,
                  uint8_t* dst_buf, int dst_w, int dst_h,
                  float* scale, float* pad_left, float* pad_top);

#ifdef __cplusplus
}
#endif

#endif // RGA_PREPROCESS_H
