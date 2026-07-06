"""drowny.py: AI_shm의 landmark 섹션에서 눈 랜드마크 좌표를 읽어 EAR을
계산하고, 벽시계 시간 기반으로 졸음을 판정해 AI_shm의 status 섹션에
기록한다. 판정이 바뀌는 시점(rising/falling edge)에 Main STM32로
경고/감속 요청 및 해제 신호를 보낸다.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/drowny.py)
실행: <코드 폴더>/drowsy_env_312/bin/python drowny.py

책임 범위:
  - 하는 일: 랜드마크 읽기 -> EAR 계산 -> 눈 감김 지속시간 판정 ->
    상태 write -> (rising/falling edge에서) STM32에 경고/해제 전송

AI_shm 사용 범위: landmark 섹션을 읽고(AI_shm.LandmarkReader), status
섹션에 씀(AI_shm.StatusWriter). 생성/해제는 AI_init 담당 -- close()는
mmap만 닫고 unlink하지 않는다.
"""

import ctypes
import mmap
import os
import signal
import sys
import time

import AI_shm

# ---------------------------------------------------------------------------
# init.c의 SystemSharedData_t 미러
# ---------------------------------------------------------------------------
SYS_SHM_NAME = "/sys_shared_memory"
MUTEX_SIZE = 40   # sizeof(pthread_mutex_t) 확인 후 맞출 것


class PthreadMutexRaw(ctypes.Structure):
    _fields_ = [("_opaque", ctypes.c_uint8 * MUTEX_SIZE)]


class SystemSharedData(ctypes.Structure):
    _fields_ = [
        ("mutex", PthreadMutexRaw),
        ("system_state", ctypes.c_int),
        ("module_type", ctypes.c_int),
        ("latest_fault", ctypes.c_int),
        ("module_id", ctypes.c_uint32),
        ("dock_detected", ctypes.c_uint8),
        ("auth_result", ctypes.c_uint8),
        ("power_granted", ctypes.c_uint8),
        ("module_function_enabled", ctypes.c_uint8),
        ("target_speed_rpm", ctypes.c_float),
        ("current_speed_rpm", ctypes.c_float),
        ("motor_pwm_duty", ctypes.c_uint16),
        ("requested_power_w", ctypes.c_float),
        ("granted_power_w", ctypes.c_float),
        ("reported_power_w", ctypes.c_float),
        ("power_violation_count", ctypes.c_uint8),
        ("pressure_value", ctypes.c_float),
        ("target_temp_c", ctypes.c_float),
        ("current_temp_c", ctypes.c_float),
        ("peltier_pwm", ctypes.c_uint8),
        ("fan_pwm", ctypes.c_uint8),
        ("warning_flag", ctypes.c_uint8),
        ("sleep_flag", ctypes.c_uint8),
    ]


libpthread = ctypes.CDLL("libpthread.so.0", use_errno=True)


class SysShmClient:
    def __init__(self):
        path = "/dev/shm" + SYS_SHM_NAME
        self._fd = os.open(path, os.O_RDWR)
        size = ctypes.sizeof(SystemSharedData)
        self._mm = mmap.mmap(self._fd, size)
        os.close(self._fd)
        self._data = SystemSharedData.from_buffer(self._mm)

    def _lock(self):
        libpthread.pthread_mutex_lock(ctypes.byref(self._data.mutex))

    def _unlock(self):
        libpthread.pthread_mutex_unlock(ctypes.byref(self._data.mutex))

    def set_flags(self, warning: bool, stage: int):
        # sleep_flag에는 drowny stage(STAGE_*, 0~3)를 그대로 기록한다.
        # 주의: 소비 측(C/STM32)은 sleep_flag를 0/1 불리언이 아니라
        # 0~3 단계값으로 읽어야 한다 (졸음 판정은 sleep_flag == STAGE_DROWSY(3)).
        self._lock()
        try:
            self._data.warning_flag = 1 if warning else 0
            self._data.sleep_flag = int(stage) & 0xFF
        finally:
            self._unlock()

    def close(self):
        self._mm.close()


# ---------------------------------------------------------------------------
# 튜닝 파라미터
# ---------------------------------------------------------------------------
EAR_THRESHOLD = 0.21
EYES_CLOSED_DURATION_S = 3.0
POLL_INTERVAL_S = 0.05

STAGE_NO_FACE = 0
STAGE_NORMAL = 1
STAGE_WARNING = 2
STAGE_DROWSY = 3

_running = True


def _handle_sigterm(signum, frame):
    global _running
    _running = False


def eye_aspect_ratio(points):
    p = points
    vert1 = ((p[1][0] - p[5][0]) ** 2 + (p[1][1] - p[5][1]) ** 2) ** 0.5
    vert2 = ((p[2][0] - p[4][0]) ** 2 + (p[2][1] - p[4][1]) ** 2) ** 0.5
    horiz = ((p[0][0] - p[3][0]) ** 2 + (p[0][1] - p[3][1]) ** 2) ** 0.5
    if horiz == 0:
        return 0.0
    return (vert1 + vert2) / (2.0 * horiz)


def send_decel_warning():
    print("[drowny] >>> DECEL/WARNING request", file=sys.stderr)


def send_clear():
    print("[drowny] >>> CLEAR (recovered)", file=sys.stderr)


def main():
    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    # 1. AI_shm(landmark/status) attach -- AI_init이 생성한 블록
    lm_reader = None
    for _ in range(20):
        try:
            lm_reader = AI_shm.LandmarkReader()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if lm_reader is None:
        print("[drowny] AI_shm attach failed. Check that AI_init is "
              "running.", file=sys.stderr)
        sys.exit(1)

    status_writer = None
    for _ in range(20):
        try:
            status_writer = AI_shm.StatusWriter()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if status_writer is None:
        print("[drowny] AI_shm attach failed.", file=sys.stderr)
        lm_reader.close()
        sys.exit(1)

    # 2. init.c의 /sys_shared_memory attach -- init_task가 먼저 떠서
    #    shm_open(O_CREAT)로 만들어둔 뒤에만 성공함.
    sys_shm = None
    for _ in range(20):
        try:
            sys_shm = SysShmClient()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if sys_shm is None:
        print("[drowny] /sys_shared_memory attach failed. Check that "
              "init_task is running.", file=sys.stderr)
        lm_reader.close()
        status_writer.close()
        sys.exit(1)

    print("[drowny] AI_shm + /sys_shared_memory attach done",
          file=sys.stderr)

    closed_since = None
    was_drowsy = False

    try:
        while _running:
            lm = lm_reader.read()
            now = time.time()

            if lm is None or not lm["valid"]:
                closed_since = None
                ear = 0.0
                face_detected = False
                stage = STAGE_NO_FACE
                closed_duration = 0.0
            else:
                right = eye_aspect_ratio(lm["right_eye"])
                left = eye_aspect_ratio(lm["left_eye"])
                ear = (right + left) / 2.0
                face_detected = True

                if ear < EAR_THRESHOLD:
                    if closed_since is None:
                        closed_since = now
                    closed_duration = now - closed_since
                else:
                    closed_since = None
                    closed_duration = 0.0

                if closed_duration >= EYES_CLOSED_DURATION_S:
                    stage = STAGE_DROWSY
                elif closed_duration > 0.0:
                    stage = STAGE_WARNING
                else:
                    stage = STAGE_NORMAL

            is_drowsy = stage == STAGE_DROWSY
            is_warning = stage == STAGE_WARNING

            status_writer.write(
                ear=ear,
                stage=stage,
                face_detected=face_detected,
                closed_duration_s=closed_duration,
                timestamp=now,
            )

            # 시스템 공유 메모리(init.c)에는 stage를 sleep_flag로 그대로 반영
            sys_shm.set_flags(warning=is_warning, stage=stage)

            if is_drowsy and not was_drowsy:
                send_decel_warning()
            elif was_drowsy and not is_drowsy:
                send_clear()
            was_drowsy = is_drowsy

            time.sleep(POLL_INTERVAL_S)
    finally:
        print("[drowny] shutting down", file=sys.stderr)
        lm_reader.close()
        status_writer.close()
        sys_shm.close()

if __name__ == "__main__":
    main()
