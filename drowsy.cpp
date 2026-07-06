// drowsy.cpp
//
// ai_landmark_shm(eye_seeker가 쓴 눈 12점 좌표)을 읽어서:
//   1) EAR(Eye Aspect Ratio)을 좌표로 직접 계산하고
//   2) 눈 감김 지속시간(벽시계 시간 기준)을 고려해 다단계 졸음 상태를 판정한 뒤
//   3) 결과(ear + 단계 + 지속시간)를 ai_status_shm에 쓴다.
//
// 다단계 정의 (ai_shm::Stage):
//   NORMAL(0)  : EAR >= 임계값
//   WARNING(1) : EAR < 임계값이고 지속시간 < DROWSY_DURATION
//   DROWSY(2)  : EAR < 임계값이고 지속시간 >= DROWSY_DURATION -> Main STM32에 경고/감속
//   NO_FACE(3) : 좌표 invalid (얼굴 미검출) -> 판정 불가
//
// DROWSY 진입(rising edge)에서만 Main STM32로 경고/감속 명령을 1회 보낸다.
// 실제 전송(UART/CAN)은 send_decel_warning() placeholder에 연결할 것.
//
// 튜닝 파라미터(EAR_THRESHOLD, DROWSY_DURATION_S)는 실환경에서 재조정 대상.
//
// 빌드: g++ -std=c++17 drowsy.cpp -o drowsy -lrt

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <optional>
#include <thread>

#include "ai_shm.hpp"

using ai_shm::EyePoint;
using ai_shm::Stage;

// ---------------------------------------------------------------------------
// 튜닝 파라미터 (미확정 -- 실환경 테스트 후 확정)
// ---------------------------------------------------------------------------
static constexpr float  EAR_THRESHOLD   = 0.21f;  // 이 값 미만이면 눈 감김
static constexpr double DROWSY_DURATION_S = 3.0;  // 이만큼 지속되면 DROWSY
static constexpr double STALE_TIMEOUT_S = 1.0;    // 좌표 갱신 끊김 판단
static constexpr double POLL_INTERVAL_S = 0.02;   // 50 Hz

static std::atomic<bool> g_running{true};
static void handle_signal(int) { g_running = false; }

// ---------------------------------------------------------------------------
// EAR 계산: 6점 순서 [outer_corner, upper1, upper2, inner_corner, lower2, lower1]
// (eye_seeker가 RIGHT_EYE/LEFT_EYE 인덱스로 이 순서를 보장해서 씀)
// ---------------------------------------------------------------------------
static float ear_one_eye(const EyePoint p[6]) {
    float v1 = std::hypot(p[1].x - p[5].x, p[1].y - p[5].y);
    float v2 = std::hypot(p[2].x - p[4].x, p[2].y - p[4].y);
    float h  = std::hypot(p[0].x - p[3].x, p[0].y - p[3].y);
    if (h == 0.f) return 0.f;
    return (v1 + v2) / (2.f * h);
}

// ---------------------------------------------------------------------------
// Main STM32로 감속/경고 전송 (placeholder).
// 실제 연결: UART 시리얼(/dev/serial0 등)에 프레임 write, 또는 오케스트레이터
// 공유 상태에 flag 세팅해서 그쪽 UART/CAN 태스크가 내보내게 함.
// rising edge에서만 호출되므로 idempotent/빠르게 유지할 것.
// ---------------------------------------------------------------------------
static void send_decel_warning() {
    std::fprintf(stderr, "[drowsy] >>> DROWSY: Main STM32에 감속/경고 전송\n");
    // TODO: UART/CAN 실제 전송 연결
}

static void send_clear() {
    std::fprintf(stderr, "[drowsy] <<< 회복: 경고 해제 전송\n");
    // TODO: UART/CAN 실제 전송 연결
}

static const char* stage_name(Stage s) {
    switch (s) {
        case Stage::NORMAL:  return "NORMAL";
        case Stage::WARNING: return "WARNING";
        case Stage::DROWSY:  return "DROWSY";
        case Stage::NO_FACE: return "NO_FACE";
    }
    return "?";
}

int main() {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // ai_landmark_shm(eye_seeker가 생성)에 attach
    std::optional<ai_shm::LandmarkReader> reader;
    try {
        reader.emplace();
    } catch (const std::exception& e) {
        std::fprintf(stderr,
            "[drowsy] ai_landmark_shm attach 실패: %s\n"
            "         eye_seeker.py 를 먼저 실행하세요.\n", e.what());
        return 1;
    }

    ai_shm::StatusWriter status;   // ai_status_shm 생성 (이 프로세스가 소유)
    std::fprintf(stderr, "[drowsy] ai_landmark_shm attach, ai_status_shm 생성 완료\n");

    bool eyes_closed = false;         // 현재 눈 감김 상태
    double closed_start = 0.0;        // 눈 감김 시작 시각
    bool in_drowsy = false;           // DROWSY 발동 상태(rising/falling edge용)
    long tick = 0;

    while (g_running) {
        auto lm = reader->read();
        double now = ai_shm::now_seconds();

        Stage stage;
        float ear = 0.f;
        float closed_duration = 0.f;

        bool valid = lm.has_value() && lm->valid &&
                     (now - lm->timestamp) <= STALE_TIMEOUT_S;

        if (!valid) {
            // 얼굴 미검출 / 데이터 끊김 -> NO_FACE, 눈감김 상태 리셋
            stage = Stage::NO_FACE;
            eyes_closed = false;
        } else {
            ear = (ear_one_eye(lm->points) + ear_one_eye(lm->points + 6)) / 2.f;

            if (ear < EAR_THRESHOLD) {
                if (!eyes_closed) {          // 방금 감기 시작
                    eyes_closed = true;
                    closed_start = now;
                }
                closed_duration = static_cast<float>(now - closed_start);
                stage = (closed_duration >= DROWSY_DURATION_S)
                            ? Stage::DROWSY : Stage::WARNING;
            } else {
                eyes_closed = false;
                stage = Stage::NORMAL;
            }
        }

        // DROWSY rising/falling edge 처리 (에피소드당 1회 전송)
        bool now_drowsy = (stage == Stage::DROWSY);
        if (now_drowsy && !in_drowsy) {
            send_decel_warning();
        } else if (!now_drowsy && in_drowsy) {
            send_clear();
        }
        in_drowsy = now_drowsy;

        status.write(ear, stage, closed_duration);

        if (++tick % 50 == 0) {   // 약 1초마다 디버그 출력
            std::fprintf(stderr, "[drowsy] ear=%.3f stage=%s closed=%.1fs\n",
                         ear, stage_name(stage), closed_duration);
        }

        std::this_thread::sleep_for(
            std::chrono::duration<double>(POLL_INTERVAL_S));
    }

    std::fprintf(stderr, "[drowsy] 종료\n");
    return 0;
}
