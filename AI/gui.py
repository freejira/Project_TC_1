"""gui.py: latest.jpg 위에 EAR/상태 오버레이와 눈 랜드마크 점을 그려서
디버깅용 라이브 화면으로 보여준다.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/gui.py)
실행: <코드 폴더>/drowsy_env_312/bin/python gui.py

책임 범위:
  - 하는 일: latest.jpg 로드 -> AI_shm(landmark/status) 읽기 -> 오버레이
    합성 -> 화면 표시 (또는 --no-window 시 텍스트 출력)

AI_shm 사용 범위: landmark 섹션을 읽고(AI_shm.LandmarkReader), status
섹션을 읽음(AI_shm.StatusReader). 둘 다 attach만 하고 close()는 mmap만
닫는다 (unlink는 AI_init 몫).

NOTE: 디스플레이(X11/Wayland)가 있는 환경에서만 창 모드가 동작함. SSH로
헤드리스 접속 중이면 cv2.imshow가 실패하므로, 그 경우 --no-window 옵션으로
텍스트 출력만 하는 방식으로 폴백함.
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import ctypes
import mmap
import os

import AI_shm

SYS_SHM_PATH = "/dev/shm/sys_shared_memory"


class _PthreadMutexRaw(ctypes.Structure):
    _fields_ = [("_opaque", ctypes.c_uint8 * 48)]


class _SystemSharedData(ctypes.Structure):
    """init.c의 SystemSharedData_t와 1:1 대응 -- sleep_flag의 실제
    바이트 오프셋을 계산하기 위한 용도로만 쓴다 (읽기/쓰기 자체는 안 함).

    주의: drowny.py의 SystemSharedData와 반드시 같은 필드 순서/타입을
    유지해야 한다. 예전에는 이런 struct 없이 "파일의 마지막 바이트 =
    sleep_flag"라고 가정했는데, 구조체 끝에 컴파일러가 붙이는 정렬
    패딩 때문에 실제로는 sleep_flag가 아니라 패딩 바이트를 읽고 있었다
    (sleep_flag_reader.c로 실측 확인된 버그, drowny.py 쪽 쓰기 버그와
    같은 종류). 이제는 진짜 오프셋을 계산해서 그 위치만 읽는다."""
    _fields_ = [
        ("mutex", _PthreadMutexRaw),
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
        ("warning_flag", ctypes.c_uint32),
        ("sleep_flag", ctypes.c_uint8),
    ]


_SLEEP_FLAG_OFFSET = _SystemSharedData.sleep_flag.offset


class SysSleepFlag:
    """init.c의 /sys_shared_memory에서 sleep_flag 1바이트만 read.

    _SystemSharedData ctypes 정의로 계산한 실제 오프셋(_SLEEP_FLAG_OFFSET)
    위치만 읽는다. uint8 단일 바이트 read는 원자적이므로 뮤텍스는 필요
    없다. 생성/해제는 init.c 담당 -- 여기서는 close만 하고 unlink하지
    않는다."""

    def __init__(self):
        fd = os.open(SYS_SHM_PATH, os.O_RDONLY)
        size = os.fstat(fd).st_size
        if _SLEEP_FLAG_OFFSET >= size:
            os.close(fd)
            raise ValueError(
                f"sleep_flag offset({_SLEEP_FLAG_OFFSET}) >= shm size({size}); "
                f"init.c의 SystemSharedData_t와 여기 _SystemSharedData 정의가 "
                f"어긋났을 수 있음")
        self._mm = mmap.mmap(fd, size, prot=mmap.PROT_READ)
        os.close(fd)

    def read(self):
        return self._mm[_SLEEP_FLAG_OFFSET]

    def close(self):
        self._mm.close()

DISPLAY_INTERVAL_S = 0.15      # latest.jpg 재로드 주기
STATE_PRINT_INTERVAL_S = 0.5   # 상태값 콘솔 출력 주기 (창 갱신과는 별개)

AI_DIR = Path(__file__).resolve().parent
LOGS_DIR = AI_DIR / "logs"
LATEST_IMAGE_PATH = LOGS_DIR / "latest.jpg"
WINDOW_NAME = "Drowsiness Monitor (debug)"

EYE_POINT_RADIUS = 2
EYE_POINT_COLOR = (0, 255, 255)     # yellow dots on each landmark
EYE_LINE_COLOR = (255, 255, 0)      # cyan contour connecting the 6 points

# drowny.py와 합의된 AI_state 코드 (0~5)
AI_STATE_NO_FACE = 0
AI_STATE_FACE_OK = 1
AI_STATE_EYE_CLOSING = 2
AI_STATE_DROWSY_EST = 3
AI_STATE_SLEEP_EST = 4
AI_STATE_STOPPED = 5

STAGE_LABELS = {
    AI_STATE_NO_FACE: ("NO FACE", (0, 165, 255)),
    AI_STATE_FACE_OK: ("FACE OK", (0, 200, 0)),
    AI_STATE_EYE_CLOSING: ("EYES CLOSING", (0, 255, 255)),
    AI_STATE_DROWSY_EST: ("DROWSY (BUZZER)", (0, 140, 255)),
    AI_STATE_SLEEP_EST: ("SLEEP (LED+DECEL)", (0, 0, 255)),
    AI_STATE_STOPPED: ("STOPPED", (255, 0, 0)),
}

# sleep_flag(=init.c로 반영된 AI_state) 콘솔 출력용 이름
SLEEP_FLAG_NAME = {
    AI_STATE_NO_FACE: "NO_FACE",
    AI_STATE_FACE_OK: "FACE_OK",
    AI_STATE_EYE_CLOSING: "EYE_CLOSING",
    AI_STATE_DROWSY_EST: "DROWSY_EST",
    AI_STATE_SLEEP_EST: "SLEEP_EST",
    AI_STATE_STOPPED: "STOPPED",
}


def sleep_flag_name(v):
    return SLEEP_FLAG_NAME.get(v, "?(%s)" % v)


def draw_landmarks(frame, lm_state, img_w, img_h):
    """눈 랜드마크 12개 포인트를 이미지 위에 점 + 윤곽선으로 그림.

    lm_state: LandmarkReader.read()의 결과. frame_w/frame_h는 캡처 당시
    해상도이므로, latest.jpg가 다른 해상도로 로드된 경우를 대비해 스케일
    보정을 한다 (보통은 동일해서 scale=1.0).
    """
    if lm_state is None or not lm_state["valid"]:
        return frame

    out = frame
    scale_x = img_w / lm_state["frame_w"] if lm_state["frame_w"] else 1.0
    scale_y = img_h / lm_state["frame_h"] if lm_state["frame_h"] else 1.0

    for eye_pts in (lm_state["right_eye"], lm_state["left_eye"]):
        scaled = [(int(x * scale_x), int(y * scale_y)) for x, y in eye_pts]
        for pt in scaled:
            cv2.circle(out, pt, EYE_POINT_RADIUS, EYE_POINT_COLOR, -1)
        # 6개 점을 순서대로 이어서 눈 윤곽선처럼 표시 (폐곡선)
        for i in range(len(scaled)):
            cv2.line(out, scaled[i], scaled[(i + 1) % len(scaled)],
                      EYE_LINE_COLOR, 1)
    return out


def draw_overlay(frame, status):
    """AI_shm status 섹션 값을 이미지 위에 텍스트로 그려서 반환."""
    out = frame

    if status is None:
        cv2.putText(out, "NO DATA", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return out

    ear = status["ear"]
    stage = status["stage"]
    closed_duration = status["closed_duration_s"]

    # 1초 이상 값이 갱신 안 됐으면 drowny 프로세스 문제로 간주
    stale = (time.time() - status["timestamp"]) > 1.0

    if stale:
        label, color = "NO SIGNAL", (0, 0, 255)
    else:
        label, color = STAGE_LABELS.get(stage, ("UNKNOWN", (255, 255, 255)))

    cv2.putText(out, f"EAR: {ear:.3f}  {label}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(out, f"closed: {closed_duration:.2f}s", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(out, time.strftime("%Y-%m-%d %H:%M:%S"), (10, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


def run_with_window(lm_reader, status_reader, sys_flag):
    """cv2 창에 latest.jpg + 상태 오버레이 + 눈 랜드마크를 합성해서 표시."""
    print(f"[gui] displaying {LATEST_IMAGE_PATH} in window "
          f"'{WINDOW_NAME}' (press 'q' to quit)", file=sys.stderr)

    last_mtime = None
    last_state_print = 0.0

    while True:
        if LATEST_IMAGE_PATH.exists():
            mtime = LATEST_IMAGE_PATH.stat().st_mtime
            if mtime != last_mtime:
                img = cv2.imread(str(LATEST_IMAGE_PATH))
                if img is not None:
                    h, w = img.shape[:2]
                    lm_state = lm_reader.read()
                    status = status_reader.read()

                    composited = img.copy()
                    composited = draw_landmarks(composited, lm_state, w, h)
                    composited = draw_overlay(composited, status)
                    cv2.imshow(WINDOW_NAME, composited)
                last_mtime = mtime

        # 상태값은 별도 주기로 콘솔에도 출력
        now = time.time()
        if now - last_state_print >= STATE_PRINT_INTERVAL_S:
            status = status_reader.read()
            sf = sys_flag.read() if sys_flag else None
            if status is not None:
                print(
                    f"[gui] ear={status['ear']:.3f} "
                    f"stage={status['stage']} "
                    f"sleep_flag={sleep_flag_name(sf) if sf is not None else 'n/a'} "
                    f"closed={status['closed_duration_s']:.2f}s"
                )
            last_state_print = now

        # waitKey가 실제 창 갱신을 처리함; 'q'로 종료
        key = cv2.waitKey(int(DISPLAY_INTERVAL_S * 1000)) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


def run_text_only(lm_reader, status_reader, sys_flag):
    """디스플레이 없는 환경(SSH 헤드리스)용 폴백: 텍스트만 출력."""
    print("[gui] no-window mode: printing text status only", file=sys.stderr)
    try:
        while True:
            status = status_reader.read()
            lm_state = lm_reader.read()
            sf = sys_flag.read() if sys_flag else None

            if LATEST_IMAGE_PATH.exists():
                img_age = time.time() - LATEST_IMAGE_PATH.stat().st_mtime
                img_status = f"latest.jpg age={img_age:.2f}s"
            else:
                img_status = "latest.jpg NOT FOUND"

            lm_status = "landmarks=valid" if (lm_state and lm_state["valid"]) \
                else "landmarks=none"

            if status is None:
                print(f"[gui] read failed (torn read, retry) | "
                      f"{img_status} | {lm_status}")
            else:
                print(
                    f"[gui] ear={status['ear']:.3f} "
                    f"stage={status['stage']} "
                    f"sleep_flag={sleep_flag_name(sf) if sf is not None else 'n/a'} "
                    f"closed={status['closed_duration_s']:.2f}s "
                    f"| {img_status} | {lm_status}"
                )
            time.sleep(STATE_PRINT_INTERVAL_S)
    except KeyboardInterrupt:
        pass


def _attach_with_retry(factory, label, retries=20, interval_s=0.1):
    """AI_init이 이 프로세스보다 늦게 뜬 경우를 대비해 짧게 재시도한다."""
    for _ in range(retries):
        try:
            return factory()
        except FileNotFoundError:
            time.sleep(interval_s)
    print(f"[gui] {label} attach failed. Check that AI_init is running.",
          file=sys.stderr)
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-window", action="store_true",
                         help="text-only output for headless environments")
    args = parser.parse_args()

    LOGS_DIR.mkdir(parents=True, exist_ok=True)

    lm_reader = _attach_with_retry(AI_shm.LandmarkReader, "AI_shm(landmark)")
    status_reader = _attach_with_retry(AI_shm.StatusReader, "AI_shm(status)")

    print("[gui] AI_shm(landmark/status) attach done", file=sys.stderr)

    # init.c 시스템 공유 메모리의 sleep_flag 리더. 없어도(=init_task 미실행)
    # 카메라 오버레이는 계속 동작하고 sleep_flag만 'n/a'로 표시한다.
    try:
        sys_flag = SysSleepFlag()
        print("[gui] attached to /sys_shared_memory (sleep_flag)",
              file=sys.stderr)
    except FileNotFoundError:
        sys_flag = None
        print("[gui] /sys_shared_memory not found; sleep_flag shows 'n/a'. "
              "Is init_task running?", file=sys.stderr)

    try:
        if args.no_window:
            run_text_only(lm_reader, status_reader, sys_flag)
        else:
            try:
                run_with_window(lm_reader, status_reader, sys_flag)
            except cv2.error as e:
                print(f"[gui] cv2 window failed ({e}); "
                      f"falling back to text-only mode", file=sys.stderr)
                run_text_only(lm_reader, status_reader, sys_flag)
    finally:
        lm_reader.close()      # attach만 했으므로 unlink는 AI_init 몫
        status_reader.close()  # attach만 했으므로 unlink는 AI_init 몫
        if sys_flag:
            sys_flag.close()   # attach만 했으므로 unlink는 init.c 몫


if __name__ == "__main__":
    main()
