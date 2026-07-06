"""eye_seeker.py: ai_frame_shm의 프레임을 읽어 MediaPipe FaceMesh로 눈 랜드마크
12점(오른쪽 6 + 왼쪽 6)만 추출해 ai_landmark_shm에 쓴다.

위치: ~/my/AI_c/eye_seeker.py
실행: ~/my/AI_c/drowsy_env_312/bin/activate 안에서 (mediapipe 0.10.18)
      (venv는 기존 ~/my/AI/drowsy_env_312 를 공유해도 됨)

책임 범위 (의도적으로 얇게):
  - 하는 일 : 프레임 읽기 -> FaceMesh -> 눈 12점 좌표 write
  - 안 하는 일 : EAR 계산, 졸음 판정, 이미지 저장, 오버레이 그리기
                (EAR/졸음은 drowsy(C++)가, 시각화는 ai_GUI(C++)가 담당)

이렇게 얇게 두면 MediaPipe를 나중에 dlib/YOLO 등으로 바꿔도 영향 범위가
이 파일 하나로 국한된다.

이전 cam.py와의 차이:
  - 카메라 캡처(rpicam-vid subprocess) 제거 -> 대신 ai_frame_shm에서 프레임 수신
  - EAR 계산/졸음 판정 제거 -> drowsy로 이관
  - latest.jpg 저장 제거 -> cam(C++)이 담당
"""

import signal
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

import ai_shm

# MediaPipe FaceMesh 6점 EAR 랜드마크 인덱스.
# 눈당 순서: [바깥쪽 눈꼬리, 위쪽1, 위쪽2, 안쪽 눈꼬리, 아래쪽2, 아래쪽1]
# -> drowsy(C++)의 EAR 계산이 이 순서를 그대로 가정하므로 절대 바꾸지 말 것.
RIGHT_EYE = [33, 160, 158, 133, 153, 144]
LEFT_EYE = [362, 385, 387, 263, 373, 380]

POLL_INTERVAL_S = 0.005   # 새 프레임 대기 폴링 간격
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

    # ai_frame_shm(cam이 생성)에 attach. cam이 먼저 떠 있어야 함.
    try:
        frame_reader = ai_shm.FrameReader()
    except FileNotFoundError:
        print("[eye_seeker] ai_frame_shm 없음. cam(C++)을 먼저 실행하세요.",
              file=sys.stderr)
        sys.exit(1)

    # ai_landmark_shm 생성 (이 프로세스가 writer/소유자)
    landmark_writer = ai_shm.LandmarkWriter(create=True)
    print("[eye_seeker] ai_frame_shm attach, ai_landmark_shm 생성 완료",
          file=sys.stderr)

    face_mesh = mp.solutions.face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    last_seq = 0
    processed = 0
    fps_t0 = time.time()

    try:
        while _running:
            got = frame_reader.read()
            if got is None:
                time.sleep(POLL_INTERVAL_S)
                continue

            pixels, meta = got
            # 같은 프레임 두 번 처리 방지 (cam의 프레임 seq로 판별)
            if meta["seq"] == last_seq:
                time.sleep(POLL_INTERVAL_S)
                continue
            last_seq = meta["seq"]

            w, h, ch = meta["width"], meta["height"], meta["channels"]
            # SHM 바이트 -> numpy BGR 이미지 (복사본; SHM 버퍼를 오래 안 붙잡음)
            frame = np.frombuffer(pixels, dtype=np.uint8).reshape(h, w, ch)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            results = face_mesh.process(rgb)

            if results.multi_face_landmarks:
                lm = results.multi_face_landmarks[0].landmark
                right_pts = get_eye_points(lm, RIGHT_EYE, w, h)
                left_pts = get_eye_points(lm, LEFT_EYE, w, h)
                # 순서: 오른쪽 눈 6점 다음 왼쪽 눈 6점 (hpp/drowsy와 동일 규약)
                landmark_writer.write_valid(right_pts + left_pts, w, h)
            else:
                landmark_writer.write_invalid(w, h)

            processed += 1
            if processed % 30 == 0:
                now = time.time()
                fps = 30.0 / (now - fps_t0) if now > fps_t0 else 0.0
                fps_t0 = now
                face = bool(results.multi_face_landmarks)
                print(f"[eye_seeker] fps={fps:4.1f} frame_seq={last_seq} "
                      f"face={face}", file=sys.stderr)
    finally:
        print("[eye_seeker] 종료", file=sys.stderr)
        face_mesh.close()
        frame_reader.close()
        landmark_writer.close()   # writer이므로 unlink까지 수행


if __name__ == "__main__":
    main()
