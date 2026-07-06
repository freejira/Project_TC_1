"""AI_shm.py: AI_c 파이프라인(cam / check_eye / drowny / ai_GUI)이 공유하는
POSIX 공유메모리 레이어. ctypes.Structure로 mmap 버퍼를 직접 매핑해서,
struct.pack_into/unpack_from 없이 "진짜 구조체 필드 접근"(data.landmark.ear
같은 식)이 되도록 만들었다.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/AI_shm.py)

** face_detected는 이 블록에 없음 **
얼굴 검출 여부는 stage(STAGE_NO_FACE=0)로 이미 표현되므로 별도 필드로
중복 저장하지 않는다. init.c 쪽 SystemSharedData_t의 sleep_flag가
최상위 졸음 상태를 대표하고, 여기 status 섹션은 GUI 디버깅용 상세 값
(ear/stage/closed_duration_s/timestamp)만 담당한다.

** POSIX shm_open 방식 (System V IPC 아님) **
Qt6 GUI 쪽에서 QSharedMemory 대신 shm_open/mmap을 쓰기로 한 이유와 동일:
QSharedMemory는 System V IPC 기반이라 우리가 쓰는 POSIX 방식(shm_open,
결국 리눅스에서는 /dev/shm/<name> 파일을 여는 것과 동일)과 바이트 레벨로
호환되지 않는다. `shmid_ds`/`shmget`/`shmctl` 같은 System V API는 여기서
전혀 쓰지 않는다 -- 그건 커널이 관리하는 세그먼트 메타데이터(권한, attach
카운트 등)를 위한 구조체지, 우리 데이터(눈 좌표/EAR/졸음 단계)를 담는
자리가 아니다.

** 단일 블록 구조 **
랜드마크(check_eye가 씀)와 상태(drowny가 씀)를 별도 shm 두 개로 나누지
않고, `/dev/shm/AI_shm` 하나에 `AiSharedData` 구조체 하나로 관리한다.
그 안에 `landmark`, `status` 두 개의 하위 섹션이 있고, 각 섹션은 독립된
`seq` 필드를 갖는다 (writer가 서로 다른 프로세스이므로 섹션끼리 seqlock을
공유하면 안 됨).

** 소유권 모델 **
이 블록의 생성(create)과 해제(unlink)는 **AI_init만** 한다.
cam/check_eye/drowny/ai_GUI는 전부 attach만 하고, close()만 호출한다
(unlink 안 함).

** multiprocessing.shared_memory를 안 쓰는 이유 **
독립 프로세스(부모-자식 관계 아님)가 각자 attach만 해도, Python의
resource_tracker가 attach한 프로세스 기준으로 정리를 시도해서 실제
소유자가 아닌 프로세스가 죽을 때 shm이 사라지는 예측 불가능한 상황이
생길 수 있다. 그래서 raw `os.open` + `mmap`으로 `/dev/shm/<name>`을
직접 다룬다.

** 동시성: seqlock (섹션별 독립) **
독립 프로세스 간에는 `multiprocessing.Lock`을 상속할 수 없으므로, 커널 락
없는 seqlock 패턴을 쓴다:
  - writer: seq를 홀수로 올림(쓰기 시작) -> 필드 기록 -> seq를 다음
    짝수로 올림(쓰기 완료)
  - reader: seq를 읽어(짝수여야 안정 상태) 필드를 읽고 seq를 다시 읽어서
    바뀌었거나 홀수였으면 재시도.
  - 모든 구조체에 `_pack_ = 1`을 지정해서 정렬 패딩을 없앴다. 이렇게 하면
    C/C++ 쪽에서 동일한 필드 순서 + `#pragma pack(push,1)`로 정의한
    struct와 바이트 레벨로 그대로 호환된다.

** 섹션 목록 **
  - landmark : check_eye(writer) -> drowny/ai_GUI(reader)
      눈 랜드마크 12점(오른쪽 6 + 왼쪽 6) 픽셀 좌표 + 프레임 크기 + valid
  - status   : drowny(writer) -> ai_GUI(reader)
      EAR / 졸음 단계(stage) / 눈 감김 지속시간 / timestamp
"""

import ctypes
import mmap
import os

SHM_DIR = "/dev/shm"
AI_SHM_NAME = "AI_shm"

MAX_READ_RETRIES = 5


# ---------------------------------------------------------------------------
# 구조체 정의 -- 여기가 C 쪽 struct와 1:1로 맞춰야 하는 부분.
# _pack_ = 1 필수 (안 하면 컴파일러/아키텍처에 따라 패딩이 들어갈 수 있음).
# ---------------------------------------------------------------------------

class Point(ctypes.Structure):
    """눈 랜드마크 한 점의 픽셀 좌표."""
    _pack_ = 1
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
    ]


class LandmarkSection(ctypes.Structure):
    """check_eye(writer) -> drowny/ai_GUI(reader).
    seq: 홀수=쓰기 중, 짝수=안정 상태 (seqlock, 이 섹션 전용)."""
    _pack_ = 1
    _fields_ = [
        ("seq", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("frame_w", ctypes.c_uint16),
        ("frame_h", ctypes.c_uint16),
        ("right_eye", Point * 6),   # eye_seeker.py의 RIGHT_EYE 순서
        ("left_eye", Point * 6),    # eye_seeker.py의 LEFT_EYE 순서
    ]


class StatusSection(ctypes.Structure):
    """drowny(writer) -> ai_GUI(reader). seq: landmark와 독립된 별도 seqlock.
    얼굴 검출 여부는 stage(STAGE_NO_FACE=0)로 표현하므로 별도 필드 없음."""
    _pack_ = 1
    _fields_ = [
        ("seq", ctypes.c_uint32),
        ("timestamp", ctypes.c_double),
        ("ear", ctypes.c_float),
        ("stage", ctypes.c_uint8),
        ("closed_duration_s", ctypes.c_float),
    ]


class AiSharedData(ctypes.Structure):
    """AI_shm 블록 전체 레이아웃 -- 이 구조체 하나가 /dev/shm/AI_shm에
    그대로 매핑된다."""
    _pack_ = 1
    _fields_ = [
        ("landmark", LandmarkSection),
        ("status", StatusSection),
    ]


AI_SHM_SIZE = ctypes.sizeof(AiSharedData)


# ---------------------------------------------------------------------------
# 저수준 헬퍼 -- 생성/attach/해제 (AI_init 전용: create_ai_shm/unlink_ai_shm)
# ---------------------------------------------------------------------------

def _shm_path(name):
    return os.path.join(SHM_DIR, name)


def create_ai_shm():
    """AI_shm 블록을 생성하고 0으로 초기화한다 (AI_init 전용).
    이전 실행이 비정상 종료해서 파일이 남아있어도 재사용 + 0으로 덮어써서
    안전하게 재초기화한다."""
    path = _shm_path(AI_SHM_NAME)
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        os.ftruncate(fd, AI_SHM_SIZE)
        mm = mmap.mmap(fd, AI_SHM_SIZE)
        mm[:] = bytes(AI_SHM_SIZE)
        mm.flush()
    finally:
        os.close(fd)  # mmap이 매핑을 유지하므로 원본 fd는 닫아도 무방
    return mm


def open_ai_shm():
    """이미 생성되어 있는 AI_shm 블록에 attach (생성/재초기화 없음)."""
    path = _shm_path(AI_SHM_NAME)
    fd = os.open(path, os.O_RDWR)
    try:
        mm = mmap.mmap(fd, AI_SHM_SIZE)
    finally:
        os.close(fd)
    return mm


def unlink_ai_shm():
    """블록 이름 자체를 제거 (AI_init 전용, 종료 시 호출)."""
    try:
        os.unlink(_shm_path(AI_SHM_NAME))
    except FileNotFoundError:
        pass


def _attach_data():
    """mmap을 열고 그 위에 AiSharedData를 얹는다. mmap 객체를 함께 반환하는
    이유: mmap이 GC되면 매핑도 해제되므로, ctypes 구조체보다 오래 살아있게
    붙잡아둬야 함 (Writer/Reader가 self._mm으로 참조 유지)."""
    mm = open_ai_shm()
    data = AiSharedData.from_buffer(mm)
    return mm, data


# ---------------------------------------------------------------------------
# landmark 섹션: writer/reader (attach 전용 -- check_eye/drowny/GUI가 사용)
# ---------------------------------------------------------------------------

class LandmarkWriter:
    """landmark 섹션에 attach해서 쓰는 writer (check_eye가 사용).
    생성/해제는 AI_init 담당 -- close()는 mmap만 닫고 unlink하지 않는다."""

    def __init__(self):
        self._mm, self._data = _attach_data()

    def write_valid(self, points, w, h):
        """points: (x, y) 픽셀 좌표 12개, 순서는 오른쪽 눈 6 + 왼쪽 눈 6
        (eye_seeker.py의 RIGHT_EYE + LEFT_EYE 인덱스와 동일 순서)."""
        if len(points) != 12:
            raise ValueError(f"expected 12 points, got {len(points)}")
        d = self._data.landmark
        d.seq += 1  # 홀수 -> 쓰기 시작
        for i in range(6):
            d.right_eye[i].x, d.right_eye[i].y = points[i]
        for i in range(6):
            d.left_eye[i].x, d.left_eye[i].y = points[6 + i]
        d.frame_w = w
        d.frame_h = h
        d.valid = 1
        d.seq += 1  # 짝수 -> 쓰기 완료

    def write_invalid(self, w, h):
        """이번 프레임에 얼굴이 검출되지 않았을 때 호출. 좌표는 0으로 채움."""
        d = self._data.landmark
        d.seq += 1
        for i in range(6):
            d.right_eye[i].x = 0.0
            d.right_eye[i].y = 0.0
            d.left_eye[i].x = 0.0
            d.left_eye[i].y = 0.0
        d.frame_w = w
        d.frame_h = h
        d.valid = 0
        d.seq += 1

    def close(self):
        self._mm.close()


class LandmarkReader:
    """landmark 섹션에 attach해서 읽는 reader (drowny/ai_GUI가 사용)."""

    def __init__(self):
        self._mm, self._data = _attach_data()

    def read(self, max_retries=MAX_READ_RETRIES):
        """일관된 스냅샷을 dict로 반환. writer가 계속 쓰는 중이라 안정적인
        읽기를 못 얻으면 None (호출 측에서 다음 tick에 재시도할 것)."""
        d = self._data.landmark
        for _ in range(max_retries):
            seq1 = d.seq
            if seq1 % 2 == 1:
                continue  # writer가 쓰는 중
            valid = d.valid
            frame_w = d.frame_w
            frame_h = d.frame_h
            right_eye = [(p.x, p.y) for p in d.right_eye]
            left_eye = [(p.x, p.y) for p in d.left_eye]
            seq2 = d.seq
            if seq1 != seq2:
                continue  # 읽는 도중 write 발생 -> torn read, 재시도
            return {
                "valid": bool(valid),
                "frame_w": frame_w,
                "frame_h": frame_h,
                "right_eye": right_eye,
                "left_eye": left_eye,
            }
        return None

    def close(self):
        self._mm.close()


# ---------------------------------------------------------------------------
# status 섹션: writer/reader (attach 전용 -- drowny가 쓰고, ai_GUI가 읽음)
# ---------------------------------------------------------------------------

class StatusWriter:
    """status 섹션에 attach해서 쓰는 writer (drowny가 사용)."""

    def __init__(self):
        self._mm, self._data = _attach_data()

    def write(self, ear, stage, closed_duration_s, timestamp):
        d = self._data.status
        d.seq += 1
        d.timestamp = timestamp
        d.ear = ear
        d.stage = stage
        d.closed_duration_s = closed_duration_s
        d.seq += 1

    def close(self):
        self._mm.close()


class StatusReader:
    """status 섹션에 attach해서 읽는 reader (ai_GUI가 사용)."""

    def __init__(self):
        self._mm, self._data = _attach_data()

    def read(self, max_retries=MAX_READ_RETRIES):
        d = self._data.status
        for _ in range(max_retries):
            seq1 = d.seq
            if seq1 % 2 == 1:
                continue
            timestamp = d.timestamp
            ear = d.ear
            stage = d.stage
            closed_duration_s = d.closed_duration_s
            seq2 = d.seq
            if seq1 != seq2:
                continue
            return {
                "timestamp": timestamp,
                "ear": ear,
                "stage": stage,
                "closed_duration_s": closed_duration_s,
            }
        return None

    def close(self):
        self._mm.close()
