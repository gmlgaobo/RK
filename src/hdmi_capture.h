#ifndef HDMI_CAPTURE_H
#define HDMI_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_BUFFERS 4

typedef struct {
    int fd;             // Device file descriptor
    int width;          // Frame width
    int height;         // Frame height
    int pixelformat;    // V4L2 pixel format fourcc
    int nbufs;          // Number of mapped buffers
    void* buffers[MAX_BUFFERS];  // Mapped buffer pointers
    int lengths[MAX_BUFFERS];    // Buffer lengths in bytes
    int current_buf;    // Current dequeued buffer index
    bool is_mplane;     // Is multi-planar format
} hdmi_capture_t;

#ifdef __cplusplus
extern "C" {
#endif

// Open HDMI capture device
// device: /dev/videoX path
// width/height: requested frame size
// Returns 0 on success, -1 on error
int hdmi_open(hdmi_capture_t* cap, const char* device, int width, int height);

// Start streaming
// Returns 0 on success, -1 on error
int hdmi_start(hdmi_capture_t* cap);

// Stop streaming
void hdmi_stop(hdmi_capture_t* cap);

// Close device and free resources
void hdmi_close(hdmi_capture_t* cap);

// Get next frame from the queue
// Sets out_data to pointer to BGR data
// Returns 0 on success, -1 if no frame available or error
int hdmi_get_frame(hdmi_capture_t* cap, uint8_t** out_data, int* out_width, int* out_height);

// Release frame back to driver for re-use
void hdmi_release_frame(hdmi_capture_t* cap);

#ifdef __cplusplus
}
#endif

#endif
