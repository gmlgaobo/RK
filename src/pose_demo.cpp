/*
 * HDMI + YOLOv8n-pose Pose Detection Demo
 * Capture from HDMI, run inference, draw keypoints on frame
 */

#include "hdmi_capture.h"
#include "yolo_inference.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <unistd.h>

static double get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

// Connect keypoints for skeleton drawing
const int skeleton[14][2] = {
    {0, 1}, {0, 2}, {1, 3}, {2, 4},
    {5, 6}, {5, 11}, {6, 12}, {11, 12},
    {11, 13}, {12, 14}, {0, 5}, {0, 6}, {5, 7}, {6, 8}
};

int main(int argc, char** argv) {
    const char* device = "/dev/video40";
    int width = 1920;
    int height = 1080;
    const char* model_path = "yolov8n-pose.rknn";

    if (argc >= 2) {
        device = argv[1];
    }
    if (argc >= 5) {
        width = atoi(argv[2]);
        height = atoi(argv[3]);
        model_path = argv[4];
    }

    printf("RK3588 YOLOv8n-pose Detection Demo\n");
    printf("Device: %s\nResolution: %dx%d\nModel: %s\n", device, width, height, model_path);

    // Open HDMI capture
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

    // Initialize YOLO inference
    YoloPoseInference inference;
    if (inference.init(model_path) != 0) {
        fprintf(stderr, "Failed to initialize YOLO model: %s\n", model_path);
        fprintf(stderr, "Make sure you have converted the model to .rknn and placed it here\n");
        hdmi_stop(&cap);
        hdmi_close(&cap);
        return 1;
    }

    printf("Streaming started, opening preview window...\n");
    printf("Press 'q' or ESC to quit\n");

    cv::namedWindow("YOLOv8n-pose Detection", cv::WINDOW_NORMAL);
    cv::resizeWindow("YOLOv8n-pose Detection", 960, 540);

    int frame_count = 0;
    double start_time = get_time_ms();
    double fps = 0;
    double total_inference_time = 0;
    int total_frames = 0;

    while (true) {
        uint8_t* data;
        int w, h;

        if (hdmi_get_frame(&cap, &data, &w, &h) != 0) {
            usleep(1000);
            continue;
        }

        // Wrap buffer to OpenCV Mat (zero copy)
        cv::Mat frame(h, w, CV_8UC3, data);

        // Run inference
        double infer_start = get_time_ms();
        auto detections = inference.detect(frame, 0.5f);
        double infer_end = get_time_ms();
        total_inference_time += (infer_end - infer_start);
        total_frames++;

        // Draw detections
        for (const auto& det : detections) {
            // Draw bounding box
            float x1 = det.x - det.w / 2.0f;
            float y1 = det.y - det.h / 2.0f;
            float x2 = det.x + det.w / 2.0f;
            float y2 = det.y + det.h / 2.0f;
            cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

            // Draw keypoints
            for (int k = 0; k < NUM_KP; k++) {
                if (det.kp_score[k] > 0.5f) {
                    cv::circle(frame, cv::Point(det.kp[k][0], det.kp[k][1]), 4, cv::Scalar(0, 0, 255), -1);
                }
            }

            // Draw skeleton
            for (int i = 0; i < 14; i++) {
                int kp1 = skeleton[i][0];
                int kp2 = skeleton[i][1];
                if (det.kp_score[kp1] > 0.5f && det.kp_score[kp2] > 0.5f) {
                    cv::line(frame,
                            cv::Point(det.kp[kp1][0], det.kp[kp1][1]),
                            cv::Point(det.kp[kp2][0], det.kp[kp2][1]),
                            cv::Scalar(255, 0, 0), 2);
                }
            }

            // Draw score
            char text[32];
            snprintf(text, sizeof(text), "%.2f", det.score);
            cv::putText(frame, text, cv::Point(x1 + 5, y1 + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // Calculate FPS
        frame_count++;
        double elapsed = get_time_ms() - start_time;
        if (elapsed >= 1000.0) {
            fps = frame_count * 1000.0 / elapsed;
            printf("FPS: %.1f, avg inference: %.1f ms\n", fps, total_inference_time / total_frames);
            frame_count = 0;
            start_time = get_time_ms();
        }

        // Draw FPS
        char fps_text[32];
        snprintf(fps_text, sizeof(fps_text), "FPS: %.1f", fps);
        cv::putText(frame, fps_text, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        cv::imshow("YOLOv8n-pose Detection", frame);

        hdmi_release_frame(&cap);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    if (total_frames > 0) {
        printf("Average inference time: %.1f ms\n", total_inference_time / total_frames);
        printf("Average FPS: %.1f\n", 1000.0 * total_frames / (get_time_ms() - start_time));
    }

    hdmi_stop(&cap);
    hdmi_close(&cap);
    cv::destroyAllWindows();

    printf("Done\n");
    return 0;
}
