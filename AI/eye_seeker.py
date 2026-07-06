"""eye_seeker.py: logs/latest.jpg 파일을 읽어 MediaPipe FaceMesh로 눈 랜드마크
12점(오른쪽 6 + 왼쪽 6)만 추출해 AI_shm에 쓴다.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/eye_seeker.py)
실행: <코드 폴더>/drowsy_env_312/bin/activate 안에서 (mediapipe 0.10.18)

책임 범위:
  - 하는 일: latest.jpg 읽기 -> FaceMesh -> 눈 12점 좌표 write

AI_shm 사용 범위: landmark 섹션에 attach해서 씀 (AI_shm.LandmarkWriter).
생성/해제는 AI_init 담당 -- close()는 mmap만 닫고 unlink하지 않는다.

주의 (파일 기반 폴링):
  - SHM 프레임 공유 대비 최신성이 떨어짐 (최대 MONITOR_SAVE_INTERVAL
    프레임 지연 + mtime 해상도). 지연이 문제되면 cam.py의
    MONITOR_SAVE_INTERVAL을 줄이는 것부터 검토.
  - cam.py가 파일을 쓰는 도중(imwrite 중간)에 읽으면 cv2.imread가 None을
    반환할 수 있음 -- 그 경우 다음 폴링에서 재시도.
  - cam.py가 죽어도 이 파일은 latest.jpg를 계속 재사용하며 죽지 않는다.
    STALE_WARN_S 이상 파일이 안 바뀌면 로그만 남기고 계속 동작한다.
"""

import signal
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp

import AI_shm

READY_MARKER = "AI_READY"

# MediaPipe FaceMesh 6점 EAR 랜드마크 인덱스.
# 눈당 순서: [바깥쪽 눈꼬리, 위쪽1, 위쪽2, 안쪽 눈꼬리, 아래쪽2, 아래쪽1]
# -> drowny의 EAR 계산이 이 순서를 그대로 가정하므로 절대 바꾸지 말 것.
RIGHT_EYE = [33, 160, 158, 133, 153, 144]
LEFT_EYE = [362, 385, 387, 263, 373, 380]

AI_DIR = Path(__file__).resolve().parent
LATEST_IMAGE_PATH = AI_DIR / "logs" / "latest.jpg"

POLL_INTERVAL_S = 0.03   # latest.jpg mtime 확인 주기 (cam.py 갱신 주기보다 촘촘하게)
STALE_WARN_S = 1.0       # 이 시간 이상 파일이 안 바뀌면 경고 로그만 (처리는 계속)

_running = True


def _handle_sigterm(signum, frame):
    global _running
    _running = False


def get_eye_points(landmarks, eye_idx, w, h):
    """정규화된 FaceMesh 랜드마크를 픽셀 (x, y) 튜플로 변환 (한쪽 눈)."""
    return [(landmarks[i].x * w, landmarks[i].y * h) for i in eye_idx]


def main():
    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    # AI_shm(landmark 섹션)은 AI_init이 이미 생성해뒀음 -- attach만 시도.
    # 정상 흐름에서는 바로 성공하지만, 시작 타이밍이 어긋난 경우를 대비해
    # 짧게 재시도한다.
    landmark_writer = None
    for _ in range(20):  # 최대 약 2초 대기
        try:
            landmark_writer = AI_shm.LandmarkWriter()
            break
        except FileNotFoundError:
            time.sleep(0.1)
    if landmark_writer is None:
        print("[eye_seeker] AI_shm attach failed. Check that AI_init is "
              "running.", file=sys.stderr)
        sys.exit(1)
    print("[eye_seeker] AI_shm(landmark) attach done", file=sys.stderr)

    face_mesh = mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    # shm attach + FaceMesh 초기화까지 끝나야 실질적으로 "동작 준비 완료".
    # AI_init이 이 신호를 보고서야 drowny를 Popen한다.
    print(READY_MARKER, flush=True)  # stdout -- AI_init이 감지
    print("[eye_seeker] signaled ready", file=sys.stderr)

    last_mtime = None
    last_warn = 0.0

    try:
        while _running:
            if not LATEST_IMAGE_PATH.exists():
                # cam.py가 아직 첫 프레임을 안 썼을 수 있음 -> 대기만, 죽지 않음
                time.sleep(POLL_INTERVAL_S)
                continue

            mtime = LATEST_IMAGE_PATH.stat().st_mtime
            now = time.time()

            if mtime == last_mtime:
                # 새 프레임 아님 -> 재처리 스킵 (CPU 절약)
                if now - mtime > STALE_WARN_S and now - last_warn > STALE_WARN_S:
                    print(f"[eye_seeker] latest.jpg stale ({now - mtime:.2f}s) "
                          f"-- check cam.py status", file=sys.stderr)
                    last_warn = now
                time.sleep(POLL_INTERVAL_S)
                continue

            last_mtime = mtime

            frame = cv2.imread(str(LATEST_IMAGE_PATH))
            if frame is None:
                # cam.py가 쓰는 도중에 읽어서 깨진 파일 -- 다음 tick에 재시도
                time.sleep(POLL_INTERVAL_S)
                continue

            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = face_mesh.process(rgb)

            if results.multi_face_landmarks:
                lm = results.multi_face_landmarks[0].landmark
                right_pts = get_eye_points(lm, RIGHT_EYE, w, h)
                left_pts = get_eye_points(lm, LEFT_EYE, w, h)
                # 순서: 오른쪽 눈 6점 다음 왼쪽 눈 6점 (drowny와 합의된 규약)
                landmark_writer.write_valid(right_pts + left_pts, w, h)
            else:
                landmark_writer.write_invalid(w, h)
    finally:
        print("[eye_seeker] shutting down", file=sys.stderr)
        face_mesh.close()
        landmark_writer.close()   # attach만 했으므로 unlink는 AI_init 몫


if __name__ == "__main__":
    main()
