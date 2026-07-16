"""drowny.py: AI_shm의 landmark 섹션에서 눈 랜드마크 좌표를 읽어 EAR을
계산하고, 벽시계 시간 기반으로 AI_state(0~5)를 판정해 AI_shm의 status
섹션과 init.c의 sys_shared_memory(sleep_flag 필드)에 기록한다. 판정이
바뀌는 시점(rising/falling edge)에 Main STM32로 경고/감속 요청 및 해제
신호를 보낸다.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/drowny.py)
실행: <코드 폴더>/drowsy_env_312/bin/python drowny.py

책임 범위:
  - 하는 일: 랜드마크 읽기 -> EAR 계산 -> 눈 감김 지속시간 판정 ->
    AI_state 결정 -> 상태 write -> (rising/falling edge에서) STM32에
    경고/해제 전송

AI_shm 사용 범위: landmark 섹션을 읽고(AI_shm.LandmarkReader), status
섹션에 씀(AI_shm.StatusWriter). 생성/해제는 AI_init 담당 -- close()는
mmap만 닫고 unlink하지 않는다.

AI_state (0~5) 정의:
  0 NO_FACE     : 얼굴 파악 안됨
  1 FACE_OK     : 얼굴(눈) 감지, 정상
  2 EYE_CLOSING : 눈 감김 (0~1초) -- 카운터 시작
  3 DROWSY_EST  : 졸음 추정 (1~3초) -- 부저 작동
  4 SLEEP_EST   : 취침 추정 (3초 이상) -- LED 점등 + 감속 시작
  5 STOPPED     : 정차 이후. AI_state==4(SLEEP_EST)인 동안 공유 메모리의
                  current_speed_rpm을 읽어 0까지 떨어진 것을 확인하면
                  drowny.py가 자체적으로 5로 승격시킨다. 감속 자체는
                  Common STM 담당, drowny.py는 결과(속도==0)만 읽어서
                  반영한다.

주의: init.c/STM32 쪽 struct 필드명은 여전히 sleep_flag이지만, 이제
0/1 불리언이 아니라 위 0~5 AI_state 값을 그대로 담는다. STM32 측 트리거
조건(부저: AI_state==3, LED+감속: AI_state==4)은 별도 작업으로 반영 예정.

저속 예외 (LOW_SPEED_THRESHOLD_RPM, 20):
  - current_speed_rpm(RPM) 값이 20 미만인 저속 구간에서는
    EYE_CLOSING(2)/DROWSY_EST(3) 판정을 적용하지 않는다 (아래 3단계
    임계값 근거의 Seeing Machines Guardian, 현대 등 상용 기준의
    저속 컷오프 20km/h 상당을 RPM 기준으로 채택). km/h 환산은 하지
    않고 RPM 값을 그대로 비교한다.
  - SLEEP_EST(4)는 이 저속 기준에서 제외한다 -- 이미 감속(decel)이
    진행 중인 상태이므로, 감속으로 RPM이 20 밑으로 내려가는 순간
    "저속이니 졸음 아님"으로 오판해서 4단계가 풀려버리면 안 되기 때문.

상태 전이 제한:
  - 2(EYE_CLOSING)/3(DROWSY_EST)/4(SLEEP_EST) 상태에서는 1(FACE_OK, 완전
    회복)로 내려가거나, 현재보다 높은 단계로 올라가는 것만 허용한다.
    즉 3->2, 4->3처럼 한 틱짜리 EAR 노이즈로 인한 임의 하향(flicker)은
    막는다.
  - 단 NO_FACE(0)는 카메라가 얼굴/눈 자체를 잃어버린 별개의 신호
    상실 이벤트이므로 이 규칙과 무관하게 항상 즉시 반영된다.

FIXED (sleep_flag_reader.c로 실측 확인된 버그): SystemSharedData ctypes
mirror에서 warning_flag를 c_uint8(1바이트)로 잘못 선언했었다 --
init.c 실제 타입은 uint32_t(4바이트)라서, 그 뒤에 오는 sleep_flag의
Python 오프셋이 실제 C 오프셋보다 앞쪽(컴파일러 패딩 영역)으로
계산되고 있었다. 즉 이 파일이 sleep_flag/warning_flag를 쓴다고
"믿고" 있었지만 실제로는 아무도 읽지 않는 패딩 바이트에 쓰고 있었고,
진짜 sleep_flag 필드는 항상 0(초기값)으로 남아있었다 -- STM32가
드로우시니스 이벤트에 전혀 반응하지 않았던 근본 원인. target_speed_rpm
등도 init.c에서는 uint32_t인데 c_float로 잘못 선언돼 있어서 크기는
맞았지만(둘 다 4바이트) 값 해석 자체가 틀렸었다 -- 전부 실제 타입에
맞춰 수정함.
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
MUTEX_SIZE = 48   # sizeof(pthread_mutex_t) 확인 후 맞출 것


class PthreadMutexRaw(ctypes.Structure):
    _fields_ = [("_opaque", ctypes.c_uint8 * MUTEX_SIZE)]


class SystemSharedData(ctypes.Structure):
    # 주의: 아래 타입은 init.c의 SystemSharedData_t와 반드시 1:1로
    # 동일해야 한다. 예전 버전은 target_speed_rpm 등을 c_float로,
    # warning_flag를 c_uint8(1바이트)로 잘못 선언했었다 -- init.c에서는
    # 전부 uint32_t다. 특히 warning_flag가 실제로는 4바이트인데 여기서
    # 1바이트로 계산되면, 그 뒤에 오는 sleep_flag의 오프셋이 실제 위치
    # (구조체 패딩 때문에 몇 바이트 뒤)보다 앞으로 밀려서, 여기서 쓰는
    # sleep_flag/warning_flag가 진짜 필드가 아니라 컴파일러 패딩 바이트에
    # 쓰이는 결과가 됐었다 (sleep_flag_reader.c로 실측 확인된 버그).
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
        ("target_speed_rpm", ctypes.c_uint32),
        ("current_speed_rpm", ctypes.c_uint32),
        ("motor_pwm_duty", ctypes.c_uint16),
        ("requested_power_w", ctypes.c_uint32),
        ("granted_power_w", ctypes.c_uint32),
        ("reported_power_w", ctypes.c_uint32),
        ("power_violation_count", ctypes.c_uint8),
        ("pressure_value", ctypes.c_uint32),
        ("target_temp_c", ctypes.c_uint32),
        ("current_temp_c", ctypes.c_uint32),
        ("peltier_pwm", ctypes.c_uint8),
        ("fan_pwm", ctypes.c_uint8),
        ("warning_flag", ctypes.c_uint32),   # was c_uint8 -- real bug
        ("sleep_flag", ctypes.c_uint8),   # now holds AI_state (0~5)
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

    def read_current_speed_rpm(self) -> int:
        # Common STM(F429ZI)이 CAN으로 보고하는 실측 속도(정수 RPM,
        # init.c에서 uint32_t). AI_state==SLEEP_EST 상태에서 정차 여부
        # (STOPPED 승격) 판단에 쓴다.
        self._lock()
        try:
            return self._data.current_speed_rpm
        finally:
            self._unlock()

    def set_flags(self, warning: bool, ai_state: int):
        # sleep_flag 필드에는 AI_state(0~5)를 그대로 기록한다.
        # 주의: 소비 측(C/STM32)은 sleep_flag를 0/1 불리언이 아니라
        # 0~5 단계값으로 읽어야 한다.
        self._lock()
        try:
            self._data.warning_flag = 1 if warning else 0
            self._data.sleep_flag = int(ai_state) & 0xFF
        finally:
            self._unlock()

    def close(self):
        self._mm.close()


# ---------------------------------------------------------------------------
# 튜닝 파라미터
# ---------------------------------------------------------------------------
EAR_THRESHOLD = 0.21
DROWSY_EST_THRESHOLD_S = 1.0    # 이 시간 이상 감으면 EYE_CLOSING -> DROWSY_EST
SLEEP_EST_THRESHOLD_S = 3.5     # 이 시간 이상 감으면 DROWSY_EST -> SLEEP_EST
STOPPED_SPEED_EPSILON_RPM = 0.5  # 이 이하면 current_speed_rpm==0으로 간주
POLL_INTERVAL_S = 0.05

# 저속 컷오프 -- 상용 졸음 감지 제품/현대 등의 저속 기준(20km/h 상당)을
# 채택. 이 미만에서는 EYE_CLOSING/DROWSY_EST 판정을 적용하지 않는다.
# (SLEEP_EST(4)는 감속 진행 중이므로 이 기준에서 제외됨 -- 메인 루프 참고)
# current_speed_rpm(RPM) 값을 그대로 비교한다 -- km/h 환산 없음.
LOW_SPEED_THRESHOLD_RPM = 20


AI_STATE_NO_FACE = 0
AI_STATE_FACE_OK = 1
AI_STATE_EYE_CLOSING = 2
AI_STATE_DROWSY_EST = 3
AI_STATE_SLEEP_EST = 4
AI_STATE_STOPPED = 5   # SLEEP_EST + current_speed_rpm==0일 때 승격

# 이 상태들에 있는 동안은 1(FACE_OK)로 회복하거나 상위 단계로 올라가는
# 것만 허용 (임의 하향/flicker 방지). NO_FACE(0)는 예외 -- 항상 즉시 반영.
_MONOTONIC_STATES = (AI_STATE_EYE_CLOSING, AI_STATE_DROWSY_EST,
                      AI_STATE_SLEEP_EST)

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
    print("[drowny] >>> DECEL/WARNING request (SLEEP_EST entered)",
          file=sys.stderr)


def send_clear():
    print("[drowny] >>> CLEAR (recovered from SLEEP_EST)", file=sys.stderr)


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
    was_elevated = False
    current_stage = AI_STATE_NO_FACE   # 직전 틱의 확정 AI_state (전이 제한용)

    try:
        while _running:
            lm = lm_reader.read()
            now = time.time()

            # 매 틱 속도(RPM)를 읽는다 -- 저속 게이팅(EYE_CLOSING/DROWSY_EST
            # 미적용 여부)과 기존 STOPPED 승격 판단 양쪽에 필요.
            current_speed_rpm = sys_shm.read_current_speed_rpm()
            # SLEEP_EST(4)/STOPPED(5)는 저속 예외 대상 -- 감속 진행 중
            # 속도가 떨어지는 것 자체로 "저속이니 졸음 아님"이 되어버리는
            # 것을 막는다.
            low_speed = (abs(current_speed_rpm) < LOW_SPEED_THRESHOLD_RPM
                         and current_stage not in (AI_STATE_SLEEP_EST,
                                                    AI_STATE_STOPPED))

            if lm is None or not lm["valid"]:
                closed_since = None
                ear = 0.0
                face_detected = False
                raw_state = AI_STATE_NO_FACE
                closed_duration = 0.0

            else:
                right = eye_aspect_ratio(lm["right_eye"])
                left = eye_aspect_ratio(lm["left_eye"])
                ear = (right + left) / 2.0
                face_detected = True

                eyes_closed = (right < EAR_THRESHOLD) and (left < EAR_THRESHOLD)

                if eyes_closed:
                    if closed_since is None:
                        closed_since = now
                    closed_duration = now - closed_since
                else:
                    closed_since = None
                    closed_duration = 0.0

                if low_speed:
                    # 저속: 눈 감김 지속시간과 무관하게 졸음 판정 자체를
                    # 적용하지 않는다 (타이머는 계속 흐르게 두되, 이 틱의
                    # 판정만 FACE_OK로 억제).
                    raw_state = AI_STATE_FACE_OK
                elif closed_duration >= SLEEP_EST_THRESHOLD_S:
                    raw_state = AI_STATE_SLEEP_EST
                elif closed_duration >= DROWSY_EST_THRESHOLD_S:
                    raw_state = AI_STATE_DROWSY_EST
                elif closed_duration > 0.0:
                    raw_state = AI_STATE_EYE_CLOSING
                else:
                    raw_state = AI_STATE_FACE_OK

            # ---- 상태 전이 제한 ------------------------------------
            # 2/3/4단계에 있는 동안은 1단계(FACE_OK, 완전 회복)로 내려
            # 가거나 현재보다 높은 단계로 올라가는 것만 허용한다.
            # NO_FACE(0)는 얼굴 자체 상실이므로 이 규칙과 무관하게
            # 항상 즉시 반영한다.
            if not face_detected:
                ai_state = AI_STATE_NO_FACE
            elif current_stage in _MONOTONIC_STATES:
                if raw_state == AI_STATE_FACE_OK or raw_state > current_stage:
                    ai_state = raw_state
                else:
                    ai_state = current_stage   # 임의 하향 무시 -- 유지
            else:
                ai_state = raw_state

            # SLEEP_EST 상태에서 실제로 속도가 0까지 떨어졌으면 STOPPED로 승격.
            # (감속 자체는 STM32 쪽 작업 -- 여기서는 결과만 읽어서 반영)
            if ai_state == AI_STATE_SLEEP_EST:
                if abs(current_speed_rpm) <= STOPPED_SPEED_EPSILON_RPM:
                    ai_state = AI_STATE_STOPPED

            current_stage = ai_state

            # SLEEP_EST/STOPPED 둘 다 "고조 상태"로 취급 -- STOPPED로 승격돼도
            # decel/warning 해제(send_clear)가 잘못 발생하지 않도록 함께 본다.
            is_elevated = ai_state in (AI_STATE_SLEEP_EST, AI_STATE_STOPPED)
            is_buzzer = ai_state == AI_STATE_DROWSY_EST   # 부저 트리거 비트

            status_writer.write(
                ear=ear,
                stage=ai_state,
                face_detected=face_detected,
                closed_duration_s=closed_duration,
                timestamp=now,
            )

            # 시스템 공유 메모리(init.c)의 sleep_flag 필드에는 AI_state를
            # 그대로 반영. warning_flag는 부저(DROWSY_EST) 트리거 비트.
            sys_shm.set_flags(warning=is_buzzer, ai_state=ai_state)

            if is_elevated and not was_elevated:
                send_decel_warning()
            elif was_elevated and not is_elevated:
                send_clear()
            was_elevated = is_elevated

            time.sleep(POLL_INTERVAL_S)
    finally:
        print("[drowny] shutting down", file=sys.stderr)
        lm_reader.close()
        status_writer.close()
        sys_shm.close()


if __name__ == "__main__":
    main()
