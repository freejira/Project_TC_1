// cam.cpp
//
// 카메라 -> OpenCV -> ai_frame_shm (FrameWriter).
// AI 파이프라인의 프레임 생산자(producer). 얼굴/눈 검출은 하지 않는다.
// 그 일은 eye_seeker.py(파이썬/MediaPipe)가 ai_frame_shm을 읽어서 수행한다.
//
// 카메라 입력: libcamerasrc(GStreamer) -> OpenCV VideoCapture(CAP_GSTREAMER).
//   Raspberry Pi OS(Trixie 등)의 libcamera 스택에서 동작. picamera2/Python
//   버전 문제(libcamera .so가 시스템 python에만 바인딩)를 C++에서는 겪지 않음.
//
// 모니터링용 logs/latest.jpg 는 여기서 계속 저장한다 (기존 파이프라인 호환용).
//   SHM 프레임과는 별개 -- ai_GUI는 SHM 프레임을 쓰지만, 외부에서 파일로
//   라이브 화면을 보고 싶을 때를 위해 남겨둠. 필요 없으면 SAVE_LATEST=false.
//
// 실행 전: 같은 ai_frame_shm을 쓰는 다른 cam 인스턴스가 없어야 함.
//
// 빌드: g++ -std=c++17 cam.cpp -o cam `pkg-config --cflags --libs opencv4` -lrt

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "ai_shm.hpp"

namespace fs = std::filesystem;

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running = false; }

// latest.jpg 저장 여부 / 주기 (30fps 기준 5프레임마다 약 6Hz)
static constexpr bool SAVE_LATEST = true;
static constexpr int  MONITOR_SAVE_INTERVAL = 5;
static constexpr int  MONITOR_JPEG_QUALITY = 70;

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // ~/my/AI_c/logs 준비
    const char* home = std::getenv("HOME");
    fs::path logs_dir = fs::path(home ? home : ".") / "my" / "AI_c" / "logs";
    std::error_code ec;
    fs::create_directories(logs_dir, ec);
    std::string latest_path = (logs_dir / "latest.jpg").string();

    // GStreamer 파이프라인: libcamerasrc -> BGR -> appsink
    // 해상도/fps는 ai_shm.hpp의 kFrameWidth/Height와 반드시 일치해야 한다.
    std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw,width=" + std::to_string(ai_shm::kFrameWidth) +
        ",height=" + std::to_string(ai_shm::kFrameHeight) +
        ",framerate=30/1,format=RGBx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::fprintf(stderr,
            "[cam] 카메라 열기 실패. OpenCV가 GStreamer 지원으로 빌드됐는지,\n"
            "      gstreamer1.0-libcamera 가 설치됐는지 확인하세요.\n");
        return 1;
    }
    std::fprintf(stderr, "[cam] 카메라 열림 (%dx%d)\n",
                 ai_shm::kFrameWidth, ai_shm::kFrameHeight);

    ai_shm::FrameWriter writer;   // ai_frame_shm 생성 (이 프로세스가 소유)
    std::fprintf(stderr, "[cam] ai_frame_shm 생성 완료\n");

    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY,
                                          MONITOR_JPEG_QUALITY};
    cv::Mat frame;
    long frame_count = 0;
    auto fps_t0 = std::chrono::steady_clock::now();

    while (g_running) {
        cap >> frame;
        if (frame.empty()) continue;

        // 해상도가 다르면 SHM 레이아웃과 안 맞으므로 강제 리사이즈(안전장치)
        if (frame.cols != ai_shm::kFrameWidth ||
            frame.rows != ai_shm::kFrameHeight) {
            cv::resize(frame, frame,
                       cv::Size(ai_shm::kFrameWidth, ai_shm::kFrameHeight));
        }
        // 연속 메모리 보장 (memcpy로 SHM에 통째로 복사하므로 필수)
        if (!frame.isContinuous()) frame = frame.clone();

        writer.write(frame.data, frame.cols, frame.rows, frame.channels());

        ++frame_count;

        if (SAVE_LATEST && frame_count % MONITOR_SAVE_INTERVAL == 0) {
            cv::imwrite(latest_path, frame, jpeg_params);
        }

        if (frame_count % 30 == 0) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - fps_t0).count();
            double fps = dt > 0 ? 30.0 / dt : 0.0;
            fps_t0 = now;
            std::fprintf(stderr, "[cam] fps=%.1f frame=%ld\n", fps, frame_count);
        }
    }

    std::fprintf(stderr, "[cam] 종료\n");
    cap.release();
    return 0;
}
