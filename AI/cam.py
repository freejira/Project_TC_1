"""cam.py: Camera capture (producer).

Location: 코드 전체가 모여있는 폴더 (예: ~/my/AI/cam.py)
Run inside: <코드 폴더>/drowsy_env_312/bin/activate

Captures frames from the Raspberry Pi camera using `rpicam-vid`
(MJPEG codec) and periodically updates `logs/latest.jpg`.

NOTE(patched): rpicam-vid가 이미 개별 JPEG 프레임을 만들어 주므로,
저장 안 할 프레임까지 cv2.imdecode/imwrite로 압축 해제+재인코딩하지
않는다. 최초 1회(준비 확인)만 디코딩하고, 이후로는 raw JPEG 바이트를
그대로 파일에 쓴다 -- eye_seeker.py(MediaPipe)와 CPU를 나눠 쓰는
RPi4에서 이 디코딩 낭비가 파이프 백프레셔 -> fps 급락/급등(버스트)의
원인이었다.

This process performs no AI inference.

** readiness 신호 **
rpicam-vid(카메라 하드웨어) 초기화가 느릴 수 있어서, AI_init이 고정
딜레이 대신 이 프로세스의 실제 준비 상태를 기다린다. 첫 프레임을 성공적으로
디코딩한 시점(= 카메라가 실제로 스트리밍 중임을 확인한 시점)에 stdout으로
정확히 한 줄 "AI_READY"를 찍는다. 다른 모든 로그는 지금처럼 stderr로 가므로
stdout은 이 마커 전용 채널이다 -- 여기 print() 호출을 추가하지 말 것.
"""

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np


# Camera / capture settings
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAMERATE = 30

RPICAM_CMD = [
    "rpicam-vid",
    "-t", "0",                       # run indefinitely
    "--codec", "mjpeg",
    "--width", str(FRAME_WIDTH),
    "--height", str(FRAME_HEIGHT),
    "--framerate", str(FRAMERATE),
    "--nopreview",
    "-o", "-",                       # write stream to stdout
]

SOI = b"\xff\xd8"  # JPEG start of image
EOI = b"\xff\xd9"  # JPEG end of image

# ---------------------------------------------------------------------------
# Monitoring output settings
# ---------------------------------------------------------------------------
# latest.jpg: 매 프레임 덮어써서 저장 -> GUI/외부에서 라이브 화면처럼 사용 가능
# 매 프레임 디스크에 쓰면 SD카드 마모/부하가 있으므로 N프레임마다 한 번만 기록
MONITOR_SAVE_INTERVAL = 5   # 5프레임마다 latest.jpg 갱신 (30fps 기준 약 6Hz)
# NOTE: 더 이상 cv2.imwrite로 재인코딩하지 않고 rpicam-vid가 만든 JPEG
# 바이트를 그대로 쓰므로, 화질을 낮춰서 용량을 줄이고 싶으면 여기 대신
# RPICAM_CMD에 "--quality", "70" 같은 옵션을 추가할 것.

READY_MARKER = "AI_READY"

_running = True


def _handle_sigterm(signum, frame):
    global _running
    _running = False


def mjpeg_frames(proc):
    """Yield raw JPEG byte chunks from an rpicam-vid MJPEG stdout stream.

    NOTE: 예전에는 여기서 매 프레임마다 cv2.imdecode를 호출해 BGR로
    디코딩한 뒤 yield했다. 하지만 latest.jpg에는 MONITOR_SAVE_INTERVAL마다
    한 번(5프레임 중 1번)만 쓰는데, 디코딩은 30fps 전체에 대해 실행되고
    있었다 -- 이미 완성된 JPEG 바이트를 매번 압축 해제하는 순수 낭비.
    eye_seeker.py(MediaPipe FaceMesh)와 CPU를 나눠 써야 하는 RPi4에서
    이 낭비가 순간적으로 rpicam-vid의 stdout 파이프를 못 비우게 만들고,
    파이프가 꽉 차면 rpicam-vid 쪽 쓰기가 블로킹되어 프레임 생성 자체가
    멈췄다가(fps 급락) CPU 여유가 생기는 순간 밀린 데이터를 몰아서
    처리(fps 급등)하는 패턴으로 나타난다.
    지금은 raw JPEG 바이트만 yield하고, 디코딩은 실제로 필요한 지점
    (준비 확인 1회, 저장 시점)에서만 한다."""
    buf = b""
    while _running:
        chunk = proc.stdout.read(4096)
        if not chunk:
            break
        buf += chunk
        while True:
            start = buf.find(SOI)
            if start == -1:
                break
            end = buf.find(EOI, start + 2)
            if end == -1:
                break
            jpg = buf[start:end + 2]
            buf = buf[end + 2:]
            yield jpg


def main():
    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    # Ensure <코드 폴더>/logs exists
    ai_dir = Path(__file__).resolve().parent
    logs_dir = ai_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    latest_path = str(logs_dir / "latest.jpg")
    # 같은 디렉토리 안에서만 os.rename이 원자적이므로, 임시 파일도 logs_dir
    # 안에 둔다. 확장자는 반드시 .jpg로 끝나야 함 -- cv2.imwrite는 파일
    # 확장자로 인코더를 결정하므로, 미인식 확장자를 주면 인코딩에 실패하고
    # (버전에 따라 예외 발생) 그 뒤 os.replace가 존재하지 않는 파일을
    # 찾다가 FileNotFoundError로 프로세스를 죽인다.
    tmp_path = str(logs_dir / "latest_tmp.jpg")

    # cam.py는 이 프로세스 자체(rpicam-vid의 부모)가 잘 떴다는 것만으로는
    # "준비됨"이 아니다 -- 실제 프레임이 디코딩돼야 카메라 하드웨어가 정말
    # 스트리밍 중인지 확인되므로, ready 신호는 mjpeg_frames 루프의 첫
    # 프레임에서 보낸다 (아래 ready_sent 플래그).
    proc = subprocess.Popen(RPICAM_CMD, stdout=subprocess.PIPE, bufsize=10 ** 8)
    print("[cam] rpicam-vid started", file=sys.stderr)

    frame_count = 0
    fps_t0 = time.time()
    ready_sent = False

    try:
        for jpg in mjpeg_frames(proc):
            if not _running:
                break

            if not ready_sent:
                # 최초 1번만 실제로 디코딩해서 "진짜 유효한 JPEG"인지 확인.
                # 이후로는 저장 안 할 프레임을 디코딩하지 않는다 (아래 참고).
                test_frame = cv2.imdecode(
                    np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if test_frame is None:
                    continue  # 아직 완전한 프레임이 아님, 다음 것 대기
                print(READY_MARKER, flush=True)  # stdout -- AI_init이 감지
                print("[cam] first frame decoded, signaled ready",
                      file=sys.stderr)
                ready_sent = True

            frame_count += 1

            # 모니터링용 latest.jpg 갱신 (N프레임마다, 덮어쓰기)
            # rpicam-vid가 이미 JPEG로 인코딩해서 주므로, 디코딩 후
            # 재인코딩(cv2.imwrite)하지 않고 raw 바이트를 그대로 쓴다 --
            # 화질/용량은 필요하면 RPICAM_CMD의 --quality 옵션으로 조절.
            # latest.jpg에 직접 쓰지 않고 임시 파일에 쓴 뒤 os.replace로
            # 바꿔치기한다 (같은 파일시스템 내 rename은 원자적) -- 이렇게
            # 해야 eye_seeker.py/gui.py의 cv2.imread가 쓰다 만 파일을 읽어
            # "Premature end of JPEG file" 경고와 함께 None을 받는 경우가
            # 없어진다.
            if frame_count % MONITOR_SAVE_INTERVAL == 0:
                try:
                    with open(tmp_path, "wb") as f:
                        f.write(jpg)
                    os.replace(tmp_path, latest_path)
                except OSError as e:
                    print(f"[cam] latest.jpg write failed ({e}), "
                          f"skipping this update", file=sys.stderr)

            # Periodic debug output to stderr.
            if frame_count % 30 == 0:
                now = time.time()
                fps = 30.0 / (now - fps_t0) if now > fps_t0 else 0.0
                fps_t0 = now
                print(f"[cam] fps={fps:4.1f}", file=sys.stderr)

    finally:
        print("[cam] shutting down", file=sys.stderr)
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            proc.kill()


if __name__ == "__main__":
    main()
