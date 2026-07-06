"""ai_shm.py: ai_shm.hpp와 바이트 레벨로 1:1 맞춘 파이썬 공유 메모리 계층.

역할 분담(파이썬 쪽에서 실제로 쓰는 것):
  - FrameReader   : cam(C++)이 ai_frame_shm에 쓴 BGR 프레임을 읽음
  - LandmarkWriter: eye_seeker가 눈 12점 좌표를 ai_landmark_shm에 씀
(status 블록은 drowsy(C++)와 ai_GUI(C++)만 다루므로 파이썬에는 reader만 참고용으로 둠)

중요 - POSIX shm 직접 mmap:
  C++ 쪽은 shm_open("/ai_frame_shm", ...)을 쓴다. 리눅스에서 POSIX shm은
  /dev/shm/<name> 파일로 노출되므로, 파이썬에서 multiprocessing.shared_memory
  대신 /dev/shm/ai_frame_shm 파일을 직접 open+mmap 하면 C++와 100% 동일한
  메모리에 접근한다. (multiprocessing.shared_memory는 이름 앞 '/' 처리가
  버전마다 달라 hpp와 어긋날 위험이 있어 피한다.)

레이아웃은 ai_shm.hpp의 #pragma pack(push,1) 구조체와 정확히 일치해야 한다.
struct 포맷 문자열은 리틀엔디언+패딩없음('<')으로 고정한다 (aarch64 리틀엔디언).
"""

import mmap
import os
import struct
import time

# ---------------------------------------------------------------------------
# 공통 상수 (hpp와 동일해야 함)
# ---------------------------------------------------------------------------
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_CHANNELS = 3
FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT * FRAME_CHANNELS  # 921600

NUM_EYE_POINTS = 12  # 오른쪽 눈 6 + 왼쪽 눈 6

SHM_DIR = "/dev/shm"

# 블록 이름 (hpp의 k*ShmName에서 앞의 '/' 뗀 것 == /dev/shm 아래 파일명)
FRAME_SHM_NAME = "ai_frame_shm"
LANDMARK_SHM_NAME = "ai_landmark_shm"
STATUS_SHM_NAME = "ai_status_shm"

# ---- FRAME 블록 레이아웃 ----
# [ active_index : uint32 ][ slot0 ][ slot1 ]
# slot = FrameHeader + pixels(FRAME_BYTES)
# FrameHeader(hpp): double ts, uint32 seq, uint16 w, uint16 h, uint8 ch  (pack=1)
_FRAME_HEADER_FMT = "<dIHHB"
_FRAME_HEADER_SIZE = struct.calcsize(_FRAME_HEADER_FMT)  # 17
assert _FRAME_HEADER_SIZE == 17, _FRAME_HEADER_SIZE
_FRAME_SLOT_SIZE = _FRAME_HEADER_SIZE + FRAME_BYTES
_FRAME_SHM_SIZE = 4 + 2 * _FRAME_SLOT_SIZE
_ACTIVE_INDEX_OFF = 0

# ---- LANDMARK 블록 레이아웃 ----
# [ seq : uint32 ][ LandmarkHeader ][ points : 12*(x,y) float ]
# LandmarkHeader(hpp): double ts, uint8 valid, uint16 frame_w, uint16 frame_h (pack=1)
_LM_SEQ_FMT = "<I"
_LM_SEQ_SIZE = 4
_LM_HEADER_FMT = "<dBHH"
_LM_HEADER_SIZE = struct.calcsize(_LM_HEADER_FMT)  # 13
assert _LM_HEADER_SIZE == 13, _LM_HEADER_SIZE
_LM_POINTS_FMT = "<%df" % (NUM_EYE_POINTS * 2)
_LM_POINTS_SIZE = struct.calcsize(_LM_POINTS_FMT)  # 96
_LM_HEADER_OFF = _LM_SEQ_SIZE
_LM_POINTS_OFF = _LM_HEADER_OFF + _LM_HEADER_SIZE
_LANDMARK_SHM_SIZE = _LM_SEQ_SIZE + _LM_HEADER_SIZE + _LM_POINTS_SIZE  # 113

# ---- STATUS 블록 레이아웃 (파이썬은 참고용 reader만) ----
# [ seq : uint32 ][ StatusPayload ]
# StatusPayload(hpp): double ts, float ear, uint8 stage, float closed_duration (pack=1)
_ST_SEQ_SIZE = 4
_ST_PAYLOAD_FMT = "<dfBf"
_ST_PAYLOAD_SIZE = struct.calcsize(_ST_PAYLOAD_FMT)  # 17
assert _ST_PAYLOAD_SIZE == 17, _ST_PAYLOAD_SIZE
_STATUS_SHM_SIZE = _ST_SEQ_SIZE + _ST_PAYLOAD_SIZE  # 21

STAGE_NAMES = {0: "NORMAL", 1: "WARNING", 2: "DROWSY", 3: "NO_FACE"}


# ---------------------------------------------------------------------------
# 내부 공용: /dev/shm/<name> 을 열고 mmap 하는 헬퍼
# ---------------------------------------------------------------------------
def _open_existing(name, size):
    """기존 블록(다른 프로세스가 create)에 attach. 없으면 FileNotFoundError."""
    path = os.path.join(SHM_DIR, name)
    fd = os.open(path, os.O_RDWR)  # 읽기+쓰기로 열되, reader는 읽기만 사용
    try:
        return mmap.mmap(fd, size, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE)
    finally:
        os.close(fd)  # mmap 후 fd는 닫아도 매핑 유지됨


def _create(name, size):
    """새 블록 생성(zero-init). 이 이름의 writer가 파이썬인 경우에만 사용."""
    path = os.path.join(SHM_DIR, name)
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        os.ftruncate(fd, size)
        mm = mmap.mmap(fd, size, mmap.MAP_SHARED,
                       mmap.PROT_READ | mmap.PROT_WRITE)
    finally:
        os.close(fd)
    mm[:size] = b"\x00" * size
    return mm


def _load_seq(mm, off=0):
    return struct.unpack_from("<I", mm, off)[0]


# ===========================================================================
# FRAME 블록 reader  (cam(C++)이 writer)
# ===========================================================================
class FrameReader:
    """cam(C++)이 ai_frame_shm에 쓴 최신 BGR 프레임을 읽는다.

    반환은 numpy 배열이 아니라 (bytes, header dict)로, 호출자가 원하는 방식으로
    변환하게 둔다. eye_seeker.py에서 numpy.frombuffer로 감싸 cv2에 넘긴다.
    """

    def __init__(self):
        self._mm = _open_existing(FRAME_SHM_NAME, _FRAME_SHM_SIZE)

    def read(self):
        """최신 프레임을 (pixels_bytes, meta) 로 반환. 유효 프레임 없으면 None.

        meta = {timestamp, seq, width, height, channels}
        """
        idx = struct.unpack_from("<I", self._mm, _ACTIVE_INDEX_OFF)[0]
        if idx > 1:
            return None  # 아직 아무도 안 씀
        slot_off = 4 + idx * _FRAME_SLOT_SIZE
        ts, seq, w, h, ch = struct.unpack_from(_FRAME_HEADER_FMT, self._mm, slot_off)
        if seq == 0:
            return None
        px_off = slot_off + _FRAME_HEADER_SIZE
        pixels = self._mm[px_off:px_off + FRAME_BYTES]
        meta = {"timestamp": ts, "seq": seq,
                "width": w, "height": h, "channels": ch}
        return pixels, meta

    def close(self):
        self._mm.close()


# ===========================================================================
# LANDMARK 블록 writer  (eye_seeker가 writer -> 이 프로세스가 소유/생성)
# ===========================================================================
class LandmarkWriter:
    """눈 12점 좌표(오른쪽 6 + 왼쪽 6)를 ai_landmark_shm에 seqlock으로 쓴다.

    소유권: eye_seeker가 이 블록의 writer이므로 create + 종료 시 unlink 담당.
    """

    def __init__(self, create=True):
        if create:
            self._mm = _create(LANDMARK_SHM_NAME, _LANDMARK_SHM_SIZE)
            self._owner = True
        else:
            self._mm = _open_existing(LANDMARK_SHM_NAME, _LANDMARK_SHM_SIZE)
            self._owner = False

    def write_valid(self, points, frame_w, frame_h):
        """points: (x, y) 12개 리스트 (오른쪽 눈 6 다음 왼쪽 눈 6)."""
        self._write(True, points, frame_w, frame_h)

    def write_invalid(self, frame_w, frame_h):
        """얼굴/눈 미검출 프레임. 좌표는 0으로 채움."""
        self._write(False, None, frame_w, frame_h)

    def _write(self, valid, points, frame_w, frame_h):
        seq = _load_seq(self._mm, 0)
        # 1) 홀수: 쓰기 중
        struct.pack_into("<I", self._mm, 0, (seq + 1) & 0xFFFFFFFF)
        # 2) header
        struct.pack_into(_LM_HEADER_FMT, self._mm, _LM_HEADER_OFF,
                         time.time(), 1 if valid else 0,
                         int(frame_w) & 0xFFFF, int(frame_h) & 0xFFFF)
        # 3) points (flatten, invalid면 0)
        if valid and points is not None:
            flat = []
            for x, y in points:
                flat.append(float(x))
                flat.append(float(y))
        else:
            flat = [0.0] * (NUM_EYE_POINTS * 2)
        struct.pack_into(_LM_POINTS_FMT, self._mm, _LM_POINTS_OFF, *flat)
        # 4) 짝수: 안정
        struct.pack_into("<I", self._mm, 0, (seq + 2) & 0xFFFFFFFF)

    def close(self):
        self._mm.close()
        if self._owner:
            path = os.path.join(SHM_DIR, LANDMARK_SHM_NAME)
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass


# ===========================================================================
# STATUS 블록 reader  (참고용 -- 실제 writer는 drowsy(C++))
# ===========================================================================
class StatusReader:
    def __init__(self):
        self._mm = _open_existing(STATUS_SHM_NAME, _STATUS_SHM_SIZE)

    def read(self, max_retries=8):
        for _ in range(max_retries):
            s1 = _load_seq(self._mm, 0)
            if s1 & 1:
                continue
            ts, ear, stage, closed = struct.unpack_from(
                _ST_PAYLOAD_FMT, self._mm, _ST_SEQ_SIZE)
            s2 = _load_seq(self._mm, 0)
            if s1 == s2:
                return {"timestamp": ts, "ear": ear,
                        "stage": stage, "stage_name": STAGE_NAMES.get(stage, "?"),
                        "closed_duration": closed}
        return None

    def close(self):
        self._mm.close()
