/*
 * RK3588 HDMI-IN Capture
 * Adapts to rk_hdmirx driver with V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
 * Supports 1920x1080 BGR24 format
 */

#include "hdmi_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

// Helper: ioctl wrapper, retry on EINTR
static int xioctl(int fd, int req, void* arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    // Retry interrupted system call, standard V4L2 pattern
    return r;
}

int hdmi_open(hdmi_capture_t* cap, const char* device, int width, int height) {
    // Zero-initialize the capture struct
    memset(cap, 0, sizeof(*cap));
    cap->fd = -1;
    cap->width = width;
    cap->height = height;
    cap->nbufs = 0;
    cap->is_mplane = true; // Rockchip HDMI driver uses multi-planar API

    // Open device file: O_RDWR for V4L2, O_NONBLOCK for non-blocking mode
    cap->fd = open(device, O_RDWR | O_NONBLOCK, 0);
    if (cap->fd == -1) {
        perror("open");
        return -1;
    }

    // Query device capabilities
    struct v4l2_capability cap_info;
    if (xioctl(cap->fd, VIDIOC_QUERYCAP, &cap_info) < 0) {
        perror("VIDIOC_QUERYCAP");
        goto error;
    }

    // Check if device supports multi-planar capture
    if (!(cap_info.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE)) {
        fprintf(stderr, "Device does not support multi-planar capture\n");
        goto error;
    }

    printf("Driver: %s\nCard: %s\nBus: %s\n", cap_info.driver, cap_info.card, cap_info.bus_info);

    // Set pixel format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width;
    fmt.fmt.pix_mp.height = height;
    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_BGR24;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    // For packed BGR24, we only need one plane
    fmt.fmt.pix_mp.num_planes = 1;

    // Ask driver to set the format
    if (xioctl(cap->fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        goto error;
    }

    // Read back the actual format that driver accepted
    if (xioctl(cap->fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        goto error;
    }

    // Store actual parameters
    cap->width = fmt.fmt.pix_mp.width;
    cap->height = fmt.fmt.pix_mp.height;
    cap->pixelformat = fmt.fmt.pix_mp.pixelformat;

    // Print the actual format for debugging
    printf("Actual format: %dx%d\n", cap->width, cap->height);
    printf("Fourcc: %c%c%c%c (0x%08x)\n",
           (cap->pixelformat & 0xff),
           (cap->pixelformat >> 8) & 0xff,
           (cap->pixelformat >> 16) & 0xff,
           (cap->pixelformat >> 24) & 0xff,
           cap->pixelformat);
    printf("Number of planes: %d\n", fmt.fmt.pix_mp.num_planes);

    for (int i = 0; i < fmt.fmt.pix_mp.num_planes; i++) {
        printf("  Plane %d: size=%d, bytesperline=%d\n",
               i, fmt.fmt.pix_mp.plane_fmt[i].sizeimage,
               fmt.fmt.pix_mp.plane_fmt[i].bytesperline);
    }

    // Request buffers from the driver
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = MAX_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP; // Use memory-mapped buffers

    if (xioctl(cap->fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        goto error;
    }

    if (req.count < 2) {
        fprintf(stderr, "Insufficient buffer memory from driver\n");
        goto error;
    }

    // Map each buffer into our process address space
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[MAX_BUFFERS];
        memset(&buf, 0, sizeof(buf));
        memset(planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = fmt.fmt.pix_mp.num_planes; // length = number of planes

        // Query buffer information
        if (xioctl(cap->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            goto error;
        }

        // We only use the first plane for packed BGR24
        cap->lengths[i] = planes[0].length;
        cap->buffers[i] = mmap(NULL, planes[0].length,
                              PROT_READ | PROT_WRITE,
                              MAP_SHARED, cap->fd, planes[0].m.mem_offset);
        // Map the buffer from kernel space to user space
        if (cap->buffers[i] == MAP_FAILED) {
            perror("mmap");
            goto error;
        }

        // Queue the buffer so driver can fill it with frames
        if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            goto error;
        }

        cap->nbufs++;
    }

    printf("Opened %s: %dx%d, %d buffers mapped\n", device, cap->width, cap->height, cap->nbufs);
    return 0;

error:
    hdmi_close(cap);
    return -1;
}

int hdmi_start(hdmi_capture_t* cap) {
    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (xioctl(cap->fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        return -1;
    }
    return 0;
}

void hdmi_stop(hdmi_capture_t* cap) {
    // Stop streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    xioctl(cap->fd, VIDIOC_STREAMOFF, &type);
}

void hdmi_close(hdmi_capture_t* cap) {
    // Cleanup: unmap buffers and close device
    if (cap->fd != -1) {
        for (int i = 0; i < cap->nbufs; i++) {
            if (cap->buffers[i] && cap->buffers[i] != MAP_FAILED) {
                munmap(cap->buffers[i], cap->lengths[i]);
            }
        }
        close(cap->fd);
        cap->fd = -1;
    }
}

int hdmi_get_frame(hdmi_capture_t* cap, uint8_t** out_data, int* out_width, int* out_height) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MAX_BUFFERS];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.m.planes = planes;
    // length = number of planes, this is required for mplane API
    buf.length = 1;

    // Dequeue a filled buffer from the driver
    if (xioctl(cap->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return -1; // No frame available yet, try again later
        }
        perror("VIDIOC_DQBUF");
        return -1;
    }

    // Store current buffer index
    cap->current_buf = buf.index;
    // Return pointer to the mapped data
    *out_data = (uint8_t*)cap->buffers[buf.index];
    *out_width = cap->width;
    *out_height = cap->height;
    return 0;
}

void hdmi_release_frame(hdmi_capture_t* cap) {
    struct v4l2_buffer buf;
    struct v4l2_plane planes[MAX_BUFFERS];
    memset(&buf, 0, sizeof(buf));
    memset(planes, 0, sizeof(planes));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = cap->current_buf;
    buf.m.planes = planes;
    buf.length = 1;

    // Re-queue the buffer back to driver after we're done with it
    if (xioctl(cap->fd, VIDIOC_QBUF, &buf) < 0) {
        perror("VIDIOC_QBUF");
    }
}
