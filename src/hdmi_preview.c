/*
 * HDMI Preview with OpenCV
 * Displays live capture from HDMI-IN in a window
 */

#include "hdmi_capture.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>

static double get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv->tv_sec * 1000.0 + tv->tv_usec / 1000.0;
}

int main(int argc, char** argv) {
    const char* device = "/dev/video40";
    int width = 1920;
    int height = 1080;

    if (argc >= 2) {
        device = argv[1];
    }

    printf("RK3588 HDMI Capture Preview\n");
    printf("Device: %s\nResolution: %dx%d\n", device, width, height);

    hdmi_capture_t cap;
    if (hdmi_open(&cap, device, width, height) != 0) {
        fprintf(stderr, "Failed to open %s\n", device);
        return 1;
    }

    if (hdmi_start(&cap) != 0) {
        fprintf(stderr, "Failed to start streaming\n");
        hdmi_close(&cap);
        return 1;
    }

    printf("Streaming started, opening preview window...\n");
    printf("Press 'q' or ESC to quit\n");

    cv::namedWindow("HDMI Preview", cv::WINDOW_NORMAL);
    cv::resizeWindow("HDMI Preview", 960, 540);

    int frame_count = 0;
    double start_time = get_time_ms();
    double fps = 0;

    while (true) {
        uint8_t* data;
        int w, h;

        if (hdmi_get_frame(&cap, &data, &w, &h) != 0) {
            // No frame yet, retry
            usleep(1000);
            continue;
        }

        // Wrap the buffer directly into OpenCV Mat (no copy)
        cv::Mat frame(h, w, CV_8UC3, data);

        // Calculate FPS
        frame_count++;
        double elapsed = get_time_ms() - start_time;
        if (elapsed >= 1000.0) {
            fps = frame_count * 1000.0 / elapsed;
            printf("FPS: %.1f\n", fps);
            frame_count = 0;
            start_time = get_time_ms();
        }

        // Draw FPS on frame
        char text[32];
        snprintf(text, sizeof(text), "FPS: %.1f", fps);
        cv::putText(frame, text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        cv::imshow("HDMI Preview", frame);

        hdmi_release_frame(&cap);

        // Check for exit
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) { // q or ESC
            break;
        }
    }

    hdmi_stop(&cap);
    hdmi_close(&cap);
    cv::destroyAllWindows();

    printf("Done\n");
    return 0;
}
