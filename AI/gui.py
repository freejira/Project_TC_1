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

import AI_shm

DISPLAY_INTERVAL_S = 0.15      # latest.jpg 재로드 주기
STATE_PRINT_INTERVAL_S = 0.5   # 상태값 콘솔 출력 주기 (창 갱신과는 별개)

AI_DIR = Path(__file__).resolve().parent
LOGS_DIR = AI_DIR / "logs"
LATEST_IMAGE_PATH = LOGS_DIR / "latest.jpg"
WINDOW_NAME = "Drowsiness Monitor (debug)"

EYE_POINT_RADIUS = 2
EYE_POINT_COLOR = (0, 255, 255)     # yellow dots on each landmark
EYE_LINE_COLOR = (255, 255, 0)      # cyan contour connecting the 6 points

# drowny.py와 합의된 stage 코드
STAGE_NO_FACE = 0
STAGE_NORMAL = 1
STAGE_WARNING = 2
STAGE_DROWSY = 3

STAGE_LABELS = {
    STAGE_NO_FACE: ("NO FACE", (0, 165, 255)),
    STAGE_NORMAL: ("FACE OK", (0, 200, 0)),
    STAGE_WARNING: ("EYES CLOSING", (0, 165, 255)),
    STAGE_DROWSY: ("DROWSY", (0, 0, 255)),
}


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


def run_with_window(lm_reader, status_reader):
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
            if status is not None:
                print(
                    f"[gui] ear={status['ear']:.3f} "
                    f"stage={status['stage']} "
                    f"face_detected={status['face_detected']} "
                    f"closed={status['closed_duration_s']:.2f}s"
                )
            last_state_print = now

        # waitKey가 실제 창 갱신을 처리함; 'q'로 종료
        key = cv2.waitKey(int(DISPLAY_INTERVAL_S * 1000)) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()


def run_text_only(lm_reader, status_reader):
    """디스플레이 없는 환경(SSH 헤드리스)용 폴백: 텍스트만 출력."""
    print("[gui] no-window mode: printing text status only", file=sys.stderr)
    try:
        while True:
            status = status_reader.read()
            lm_state = lm_reader.read()

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
                    f"face_detected={status['face_detected']} "
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

    try:
        if args.no_window:
            run_text_only(lm_reader, status_reader)
        else:
            try:
                run_with_window(lm_reader, status_reader)
            except cv2.error as e:
                print(f"[gui] cv2 window failed ({e}); "
                      f"falling back to text-only mode", file=sys.stderr)
                run_text_only(lm_reader, status_reader)
    finally:
        lm_reader.close()      # attach만 했으므로 unlink는 AI_init 몫
        status_reader.close()  # attach만 했으므로 unlink는 AI_init 몫


if __name__ == "__main__":
    main()
