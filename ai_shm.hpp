// ai_shm.hpp
//
// AI 파이프라인(cam / landmark_worker / drowsy / ai_GUI)이 공유하는 POSIX 공유
// 메모리 3개 블록의 레이아웃과 create/attach/read/write 동작을 한곳에 정의한다.
//
// 프로세스별 역할:
//   - cam (C++)             : 카메라 -> OpenCV -> ai_frame_shm 에 프레임 write
//   - landmark_worker (Py)  : ai_frame_shm 읽기 -> MediaPipe -> 눈 12점 좌표만
//                             ai_landmark_shm 에 write (EAR/졸음 판정 안 함)
//   - drowsy (C++)          : ai_landmark_shm 읽기 -> EAR 계산 + 지속시간 ->
//                             다단계 졸음 판정 -> ai_status_shm 에 write
//   - ai_GUI (C++)          : 세 블록 모두 읽어 프레임 위에 좌표/EAR/단계 시각화
//
// 동기화:
//   - landmark / status 블록: seqlock (writer가 seq를 홀수->짝수로 토글).
//     페이로드가 작아 재시도 비용이 낮음. common.py/drowsy_shm.hpp와 동일 방식.
//   - frame 블록: 페이로드가 커서(~900KB) seqlock 재시도가 비싸므로,
//     더블버퍼 + active_index(atomic) 방식. writer는 비활성 버퍼에 다 쓴 뒤
//     active_index를 원자적으로 교체한다. reader는 항상 최신 active 버퍼를 읽음.
//
// 소유권 규칙 (drowsy README와 동일):
//   - 각 블록은 writer 프로세스만 create/unlink 한다.
//   - reader(consumer)는 attach 후 close만 하고 절대 unlink 하지 않는다.
//     * ai_frame_shm      -> cam 이 소유
//     * ai_landmark_shm   -> landmark_worker 가 소유
//     * ai_status_shm     -> drowsy 가 소유
//
// 중요: QSharedMemory 금지 (System V IPC라서 Python multiprocessing.shared_memory
// 및 여기서 쓰는 POSIX shm_open()과 호환 안 됨). <sys/mman.h> POSIX API 직접 사용.
//
// 빌드 시 링크: -lrt (구형 glibc). 최신 glibc는 불필요할 수 있음.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace ai_shm {

// =====================================================================
// 공통 상수 / 유틸
// =====================================================================

constexpr int kFrameWidth  = 640;
constexpr int kFrameHeight = 480;
constexpr int kFrameChannels = 3;                 // BGR
constexpr size_t kFrameBytes =
    static_cast<size_t>(kFrameWidth) * kFrameHeight * kFrameChannels;  // 921600

constexpr int kNumEyePoints = 12;                 // 오른쪽 눈 6 + 왼쪽 눈 6

// 졸음 다단계 (drowsy 가 판정해서 status 블록에 기록)
enum class Stage : uint8_t {
    NORMAL  = 0,   // EAR >= 임계값
    WARNING = 1,   // EAR < 임계값, 지속 < DROWSY 기준
    DROWSY  = 2,   // EAR < 임계값, 지속 >= DROWSY 기준
    NO_FACE = 3,   // 좌표 invalid (얼굴 미검출)
};

inline double now_seconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

// seq 필드 로드 (acquire 배리어). ARM은 메모리 모델이 약해 명시적 배리어 필요.
inline uint32_t load_seq_acquire(const void* base) {
    return reinterpret_cast<const std::atomic<uint32_t>*>(base)
        ->load(std::memory_order_acquire);
}

// =====================================================================
// 블록 정의
// =====================================================================

// ---- 블록 1: ai_frame_shm (원본 BGR 프레임, 더블버퍼) ----------------
// 레이아웃:
//   [ active_index : uint32 (atomic) ]
//   [ header0 : FrameHeader ][ pixels0 : kFrameBytes ]
//   [ header1 : FrameHeader ][ pixels1 : kFrameBytes ]
// writer는 (1 - active)에 header+pixels를 채운 뒤 active_index를 그쪽으로 교체.
constexpr const char* kFrameShmName = "/ai_frame_shm";

#pragma pack(push, 1)
struct FrameHeader {
    double   timestamp;   // 캡처 시각 (epoch seconds)
    uint32_t seq;         // 프레임 일련번호 (writer 단조 증가)
    uint16_t width;
    uint16_t height;
    uint8_t  channels;    // 3 (BGR)
};
#pragma pack(pop)
static_assert(sizeof(FrameHeader) == 17, "FrameHeader 크기 불일치");

constexpr size_t kFrameSlotSize = sizeof(FrameHeader) + kFrameBytes;
constexpr size_t kFrameShmSize  = sizeof(uint32_t) + 2 * kFrameSlotSize;

struct FrameView {
    double   timestamp;
    uint32_t seq;
    uint16_t width;
    uint16_t height;
    uint8_t  channels;
    const uint8_t* pixels;   // active 슬롯 픽셀 (읽기용, 복사 여부는 호출자 판단)
};

// ---- 블록 2: ai_landmark_shm (눈 12점 좌표, seqlock) -----------------
// 레이아웃: [ seq : uint32 ][ LandmarkHeader ][ points : 12 * (x,y) float ]
constexpr const char* kLandmarkShmName = "/ai_landmark_shm";

#pragma pack(push, 1)
struct LandmarkHeader {
    double   timestamp;
    uint8_t  valid;       // 얼굴/눈 검출 성공 여부 (1/0)
    uint16_t frame_w;     // 좌표를 계산한 프레임 해상도
    uint16_t frame_h;
};
#pragma pack(pop)
static_assert(sizeof(LandmarkHeader) == 13, "LandmarkHeader 크기 불일치");

struct EyePoint { float x; float y; };

constexpr size_t kLandmarkPointsBytes = sizeof(float) * kNumEyePoints * 2;  // 96
constexpr size_t kLandmarkShmSize =
    sizeof(uint32_t) + sizeof(LandmarkHeader) + kLandmarkPointsBytes;        // 113

struct Landmarks {
    double   timestamp;
    bool     valid;
    uint16_t frame_w;
    uint16_t frame_h;
    EyePoint points[kNumEyePoints];   // [0..5] 오른쪽 눈, [6..11] 왼쪽 눈
};

// ---- 블록 3: ai_status_shm (EAR + 졸음 단계, seqlock) ----------------
// 레이아웃: [ seq : uint32 ][ StatusPayload ]
constexpr const char* kStatusShmName = "/ai_status_shm";

#pragma pack(push, 1)
struct StatusPayload {
    double   timestamp;
    float    ear;              // drowsy 가 좌표로 계산한 EAR
    uint8_t  stage;            // Stage enum 값
    float    closed_duration;  // 눈 감김 지속시간(초). NORMAL/NO_FACE 면 0
};
#pragma pack(pop)
static_assert(sizeof(StatusPayload) == 17, "StatusPayload 크기 불일치");

constexpr size_t kStatusShmSize = sizeof(uint32_t) + sizeof(StatusPayload);  // 21

struct Status {
    double timestamp;
    float  ear;
    Stage  stage;
    float  closed_duration;
};

// =====================================================================
// 내부 공용: POSIX shm RAII 래퍼 (create 또는 attach)
// =====================================================================
class ShmRegion {
public:
    // create=true  : writer. O_CREAT로 만들고 ftruncate + zero-init.
    // create=false : reader. 기존 블록에 attach (없으면 예외).
    ShmRegion(const char* name, size_t size, bool create, bool zero_init = true)
        : name_(name), size_(size), owner_(create) {
        int flags = create ? (O_CREAT | O_RDWR) : O_RDONLY;
        int prot  = create ? (PROT_READ | PROT_WRITE) : PROT_READ;

        fd_ = shm_open(name, flags, 0666);
        if (fd_ < 0) {
            throw std::runtime_error(
                std::string("shm_open 실패: ") + name +
                (create ? "" : " (writer 프로세스가 먼저 실행 중인지 확인)"));
        }
        if (create) {
            if (ftruncate(fd_, static_cast<off_t>(size)) != 0) {
                ::close(fd_);
                throw std::runtime_error(std::string("ftruncate 실패: ") + name);
            }
        }
        base_ = mmap(nullptr, size, prot, MAP_SHARED, fd_, 0);
        if (base_ == MAP_FAILED) {
            ::close(fd_);
            throw std::runtime_error(std::string("mmap 실패: ") + name);
        }
        if (create && zero_init) {
            std::memset(base_, 0, size);
        }
    }

    ~ShmRegion() {
        if (base_ != nullptr && base_ != MAP_FAILED) munmap(base_, size_);
        if (fd_ >= 0) ::close(fd_);
        if (owner_) shm_unlink(name_.c_str());   // 소유자(writer)만 unlink
    }

    ShmRegion(const ShmRegion&) = delete;
    ShmRegion& operator=(const ShmRegion&) = delete;

    void*       data()       { return base_; }
    const void* data() const { return base_; }

private:
    std::string name_;
    size_t size_;
    bool   owner_;
    int    fd_ = -1;
    void*  base_ = nullptr;
};

// =====================================================================
// 블록 1: FRAME  (writer = cam / reader = landmark_worker(파이썬), ai_GUI)
// =====================================================================
class FrameWriter {
public:
    FrameWriter() : region_(kFrameShmName, kFrameShmSize, /*create=*/true) {}

    // pixels: kFrameBytes 크기의 BGR 연속 버퍼 (cv::Mat::data, 연속 메모리 가정)
    void write(const uint8_t* pixels, int width, int height, int channels) {
        auto* base = static_cast<uint8_t*>(region_.data());
        auto* active = reinterpret_cast<std::atomic<uint32_t>*>(base);

        uint32_t cur = active->load(std::memory_order_relaxed);
        uint32_t next = 1 - cur;                       // 비활성 슬롯에 쓴다
        uint8_t* slot = base + sizeof(uint32_t) + next * kFrameSlotSize;

        FrameHeader hdr;
        hdr.timestamp = now_seconds();
        hdr.seq       = ++frame_seq_;
        hdr.width     = static_cast<uint16_t>(width);
        hdr.height    = static_cast<uint16_t>(height);
        hdr.channels  = static_cast<uint8_t>(channels);

        std::memcpy(slot, &hdr, sizeof(hdr));
        std::memcpy(slot + sizeof(hdr), pixels, kFrameBytes);

        // 다 쓴 뒤 active를 교체 (release: 위 memcpy들이 먼저 보이도록)
        active->store(next, std::memory_order_release);
    }

private:
    ShmRegion region_;
    uint32_t  frame_seq_ = 0;
};

class FrameReader {
public:
    FrameReader() : region_(kFrameShmName, kFrameShmSize, /*create=*/false) {}

    // 반환된 FrameView.pixels 는 SHM 내부를 직접 가리킴. 오래 붙잡지 말고
    // 필요하면 즉시 복사(cv::Mat clone 등)할 것. 극히 드물게 writer가 같은
    // 슬롯을 다시 덮어쓸 수 있으므로, 지연에 민감하면 read 직후 복사 권장.
    std::optional<FrameView> read() const {
        const auto* base = static_cast<const uint8_t*>(region_.data());
        const auto* active =
            reinterpret_cast<const std::atomic<uint32_t>*>(base);

        uint32_t idx = active->load(std::memory_order_acquire);
        if (idx > 1) return std::nullopt;              // 아직 아무도 안 씀

        const uint8_t* slot = base + sizeof(uint32_t) + idx * kFrameSlotSize;
        FrameHeader hdr;
        std::memcpy(&hdr, slot, sizeof(hdr));
        if (hdr.seq == 0) return std::nullopt;          // 유효 프레임 없음

        FrameView v;
        v.timestamp = hdr.timestamp;
        v.seq       = hdr.seq;
        v.width     = hdr.width;
        v.height    = hdr.height;
        v.channels  = hdr.channels;
        v.pixels    = slot + sizeof(FrameHeader);
        return v;
    }

private:
    ShmRegion region_;
};

// =====================================================================
// 블록 2: LANDMARK  (writer = landmark_worker(파이썬) / reader = drowsy, ai_GUI)
//   * C++ writer는 테스트/폴백용. 실제 writer는 파이썬(ai_shm.py)임.
// =====================================================================
class LandmarkWriter {
public:
    LandmarkWriter() : region_(kLandmarkShmName, kLandmarkShmSize, /*create=*/true) {}

    void write_invalid(int frame_w, int frame_h) {
        write_impl(false, frame_w, frame_h, nullptr);
    }
    // points: EyePoint[kNumEyePoints] (오른쪽 6 + 왼쪽 6)
    void write_valid(int frame_w, int frame_h, const EyePoint* points) {
        write_impl(true, frame_w, frame_h, points);
    }

private:
    void write_impl(bool valid, int frame_w, int frame_h, const EyePoint* points) {
        auto* base = static_cast<uint8_t*>(region_.data());
        auto* seq_ptr = reinterpret_cast<std::atomic<uint32_t>*>(base);
        uint32_t seq = seq_ptr->load(std::memory_order_relaxed);

        seq_ptr->store(seq + 1, std::memory_order_release);   // 홀수: 쓰기 중

        LandmarkHeader hdr;
        hdr.timestamp = now_seconds();
        hdr.valid   = valid ? 1 : 0;
        hdr.frame_w = static_cast<uint16_t>(frame_w);
        hdr.frame_h = static_cast<uint16_t>(frame_h);
        std::memcpy(base + sizeof(uint32_t), &hdr, sizeof(hdr));

        float flat[kNumEyePoints * 2] = {0};
        if (valid && points) {
            for (int i = 0; i < kNumEyePoints; ++i) {
                flat[i * 2]     = points[i].x;
                flat[i * 2 + 1] = points[i].y;
            }
        }
        std::memcpy(base + sizeof(uint32_t) + sizeof(LandmarkHeader),
                    flat, sizeof(flat));

        seq_ptr->store(seq + 2, std::memory_order_release);   // 짝수: 안정
    }

    ShmRegion region_;
};

class LandmarkReader {
public:
    LandmarkReader() : region_(kLandmarkShmName, kLandmarkShmSize, /*create=*/false) {}

    std::optional<Landmarks> read(int max_retries = 8) const {
        const auto* base = static_cast<const uint8_t*>(region_.data());
        constexpr size_t hdr_off = sizeof(uint32_t);
        constexpr size_t pts_off = hdr_off + sizeof(LandmarkHeader);

        for (int i = 0; i < max_retries; ++i) {
            uint32_t s1 = load_seq_acquire(base);
            if (s1 & 1) continue;

            LandmarkHeader hdr;
            std::memcpy(&hdr, base + hdr_off, sizeof(hdr));
            float flat[kNumEyePoints * 2];
            std::memcpy(flat, base + pts_off, sizeof(flat));

            uint32_t s2 = load_seq_acquire(base);
            if (s1 == s2) {
                Landmarks out;
                out.timestamp = hdr.timestamp;
                out.valid   = hdr.valid != 0;
                out.frame_w = hdr.frame_w;
                out.frame_h = hdr.frame_h;
                for (int p = 0; p < kNumEyePoints; ++p) {
                    out.points[p].x = flat[p * 2];
                    out.points[p].y = flat[p * 2 + 1];
                }
                return out;
            }
        }
        return std::nullopt;
    }

private:
    ShmRegion region_;
};

// =====================================================================
// 블록 3: STATUS  (writer = drowsy / reader = ai_GUI, 오케스트레이터)
// =====================================================================
class StatusWriter {
public:
    StatusWriter() : region_(kStatusShmName, kStatusShmSize, /*create=*/true) {}

    void write(float ear, Stage stage, float closed_duration) {
        auto* base = static_cast<uint8_t*>(region_.data());
        auto* seq_ptr = reinterpret_cast<std::atomic<uint32_t>*>(base);
        uint32_t seq = seq_ptr->load(std::memory_order_relaxed);

        seq_ptr->store(seq + 1, std::memory_order_release);   // 홀수

        StatusPayload p;
        p.timestamp = now_seconds();
        p.ear = ear;
        p.stage = static_cast<uint8_t>(stage);
        p.closed_duration = closed_duration;
        std::memcpy(base + sizeof(uint32_t), &p, sizeof(p));

        seq_ptr->store(seq + 2, std::memory_order_release);   // 짝수
    }

private:
    ShmRegion region_;
};

class StatusReader {
public:
    StatusReader() : region_(kStatusShmName, kStatusShmSize, /*create=*/false) {}

    std::optional<Status> read(int max_retries = 8) const {
        const auto* base = static_cast<const uint8_t*>(region_.data());
        constexpr size_t off = sizeof(uint32_t);

        for (int i = 0; i < max_retries; ++i) {
            uint32_t s1 = load_seq_acquire(base);
            if (s1 & 1) continue;

            StatusPayload p;
            std::memcpy(&p, base + off, sizeof(p));

            uint32_t s2 = load_seq_acquire(base);
            if (s1 == s2) {
                Status out;
                out.timestamp = p.timestamp;
                out.ear = p.ear;
                out.stage = static_cast<Stage>(p.stage);
                out.closed_duration = p.closed_duration;
                return out;
            }
        }
        return std::nullopt;
    }

private:
    ShmRegion region_;
};

}  // namespace ai_shm
