#ifndef HDMI_CAPTURE_H
#define HDMI_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_BUFFERS 4

typedef struct {
    int fd;
    int width;
    int height;
    int pixelformat;
    int nbufs;
    void* buffers[MAX_BUFFERS];
    int lengths[MAX_BUFFERS];
    int current_buf;
    bool is_mplane;
} hdmi_capture_t;

// Open HDMI capture device
int hdmi_open(hdmi_capture_t* cap, const char* device, int width, int height);

// Start streaming
int hdmi_start(hdmi_capture_t* cap);

// Stop streaming
void hdmi_stop(hdmi_capture_t* cap);

// Close device
void hdmi_close(hdmi_capture_t* cap);

// Get next frame, returns pointer to BGR data
int hdmi_get_frame(hdmi_capture_t* cap, uint8_t** out_data, int* out_width, int* out_height);

// Release frame buffer
void hdmi_release_frame(hdmi_capture_t* cap);

#endif
