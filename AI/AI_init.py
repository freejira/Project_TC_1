"""AI_init.py: AI_shm(랜드마크+상태 통합 블록)을 먼저 생성한 뒤,
cam / check_eye(eye_seeker) / drowny 세 프로세스를 readiness 신호 기반으로
순차 기동하고, 생존 감시 및 재시작, 종료 시 shm 해제까지 담당하는
supervisor.

위치: 코드 전체가 모여있는 폴더 (예: ~/my/AI/AI_init.py)
실행: <코드 폴더>/drowsy_env_312/bin/python AI_init.py
      (venv는 기존 ~/my/AI/drowsy_env_312 를 그대로 공유해도 무방)

역할 범위:
  - AI_shm(landmark 섹션 + status 섹션)의 생성/해제를 전담한다. cam /
    check_eye / drowny는 attach만 하고 unlink하지 않는다.
  - 자식 하나가 죽어도 다른 자식이 attach 중인 shm은 사라지지 않으므로,
    재시작은 죽은 프로세스 자기 자신만 하면 된다.
  - subprocess.Popen(fork+exec)으로 자식을 띄운다 -- OpenCV/mediapipe를
    이미 초기화한 상태에서 순수 fork만 하면 자식 쪽에서 deadlock/crash
    위험이 있기 때문.

readiness 신호 기반 순차 기동:
  - cam.py / eye_seeker.py는 자신이 정상 동작을 시작한 시점에 stdout으로
    정확히 한 줄 "AI_READY"를 출력한다 (다른 로그는 전부 stderr로 감).
  - AI_init은 자식의 stdout을 파이프로 받아 이 마커를 감지한 뒤에야
    다음 프로세스를 Popen한다. 타임아웃 내에 신호가 없으면 경고만 남기고
    진행한다.
  - drowny는 체인의 마지막이라 readiness gating 대상이 아니다.

시작 순서:
  1) AI_init이 AI_shm을 생성 (0으로 초기화)
  2) cam       -> logs/latest.jpg 갱신 시작 -> AI_READY
  3) check_eye -> logs/latest.jpg를 mtime 폴링, AI_shm의 landmark 섹션에 attach해서 씀 -> AI_READY
  4) drowny    -> AI_shm의 landmark 섹션을 읽고, status 섹션에 attach해서 씀
"""

import logging
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

import AI_shm

# ---------------------------------------------------------------------------
# 경로 / 환경 설정
# ---------------------------------------------------------------------------
AI_DIR = Path(__file__).resolve().parent
VENV_PYTHON = AI_DIR / "drowsy_env_312" / "bin" / "python"

PYTHON = str(VENV_PYTHON) if VENV_PYTHON.exists() else sys.executable

# 시작 순서 (앞->뒤).
PROC_ORDER = ["cam", "check_eye", "drowny"]

PROC_SPEC = {
    "cam": {"cmd": [PYTHON, str(AI_DIR / "cam.py")]},
    "check_eye": {"cmd": [PYTHON, str(AI_DIR / "eye_seeker.py")]},
    "drowny": {"cmd": [PYTHON, str(AI_DIR / "drowny.py")]},
}

# readiness 신호를 기다릴 대상과 타임아웃. 여기 없는 프로세스(drowny)는
# gating 없이 Popen만 하고 바로 다음으로 넘어간다.
READY_MARKER = "AI_READY"
READY_TIMEOUT_S = {
    "cam": 30.0,        # rpicam-vid 하드웨어 초기화가 느릴 수 있음
    "check_eye": 15.0,  # mediapipe FaceMesh 초기화 + shm attach
}

MONITOR_INTERVAL_S = 0.5    # 생존 감시 폴링 주기
RESTART_BACKOFF_S = 2.0     # 재시작 전 대기 (연속 크래시 시 CPU 폭주 방지)
MAX_RESTARTS_PER_PROC = 10  # 이 횟수를 넘으면 해당 프로세스는 포기하고 알림
RESTART_WINDOW_S = 60.0     # 이 시간 내의 재시작만 카운트 (오래된 재시작은 잊음)
SHUTDOWN_GRACE_S = 3.0      # SIGTERM 후 SIGKILL까지 대기 시간

logging.basicConfig(
    level=logging.INFO,
    format="[AI_init] %(asctime)s %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
    stream=sys.stderr,
)
log = logging.getLogger("AI_init")

_running = True


def _handle_signal(signum, frame):
    global _running
    log.info(f"signal {signum} received, shutting down")
    _running = False


class Supervisor:
    """PROC_ORDER 순서로 자식 프로세스를 readiness 신호 기반으로 기동하고,
    죽으면 자기 자신만 재시작하며, 종료 시 시작 역순으로 정리한다."""

    def __init__(self):
        self.procs: dict[str, subprocess.Popen] = {}
        # 각 프로세스별 최근 재시작 타임스탬프 목록 (RESTART_WINDOW_S 기준 슬라이딩)
        self.restart_history: dict[str, list[float]] = {name: [] for name in PROC_ORDER}
        self.giving_up: set[str] = set()

    # -- 기동 -----------------------------------------------------------

    def _launch(self, name: str) -> subprocess.Popen:
        spec = PROC_SPEC[name]
        log.info(f"starting '{name}' ({' '.join(spec['cmd'])})")
        # stdout은 readiness 마커 전용 채널로 파이프 -- 일반 로그는 전부
        # 자식 쪽에서 stderr로 찍으므로 여기서 굳이 stderr까지 가로챌
        # 필요는 없음 (상속되어 그대로 터미널에 보임).
        # start_new_session=True: 부모가 보내는 시그널이 자식에게 의도치
        # 않게 이중 전달되지 않도록 별도 프로세스 그룹으로 둔다.
        proc = subprocess.Popen(
            spec["cmd"],
            cwd=str(AI_DIR),
            start_new_session=True,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.procs[name] = proc
        return proc

    def _wait_ready(self, name: str, proc: subprocess.Popen):
        timeout_s = READY_TIMEOUT_S.get(name)
        if timeout_s is None:
            return  # gating 대상 아님 (예: drowny)

        log.info(f"waiting for '{name}' readiness (timeout={timeout_s}s)")
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if proc.poll() is not None:
                log.warning(f"'{name}' exited before signaling ready "
                            f"(returncode={proc.returncode})")
                return
            rlist, _, _ = select.select([proc.stdout], [], [], 0.2)
            if rlist:
                line = proc.stdout.readline()
                if line and READY_MARKER in line:
                    log.info(f"'{name}' signaled ready")
                    return
        log.warning(f"'{name}' did not signal ready within {timeout_s}s "
                    f"-- proceeding anyway")

    def startup_chain(self):
        """PROC_ORDER 순서로, 앞 프로세스의 readiness 신호(또는 타임아웃)를
        기다린 뒤에만 다음 프로세스를 Popen한다."""
        for name in PROC_ORDER:
            if not _running:
                return
            proc = self._launch(name)
            self._wait_ready(name, proc)

    # -- 종료 -----------------------------------------------------------

    def stop_one(self, name: str, grace_s: float = SHUTDOWN_GRACE_S):
        proc = self.procs.get(name)
        if proc is None or proc.poll() is not None:
            return
        log.info(f"stopping '{name}' (pid={proc.pid})")
        try:
            proc.terminate()  # SIGTERM
            proc.wait(timeout=grace_s)
        except subprocess.TimeoutExpired:
            log.warning(f"'{name}' did not exit in {grace_s}s, killing")
            proc.kill()
            proc.wait(timeout=2)
        except Exception as e:
            log.error(f"error stopping '{name}': {e}")

    def stop_all(self):
        # 시작 역순으로 종료: drowny -> check_eye -> cam
        for name in reversed(PROC_ORDER):
            self.stop_one(name)

    # -- 재시작 정책 -------------------------------------------------------

    def _record_restart(self, name: str) -> bool:
        """재시작을 기록하고, 최근 윈도우 내 재시작 횟수가 한도를 넘었으면
        False를 반환(더 이상 재시작하지 말라는 뜻)."""
        now = time.time()
        hist = self.restart_history[name]
        hist.append(now)
        self.restart_history[name] = [t for t in hist if now - t <= RESTART_WINDOW_S]
        return len(self.restart_history[name]) <= MAX_RESTARTS_PER_PROC

    def handle_crash(self, name: str):
        """shm이 전부 AI_init 소유이므로, 크래시난 프로세스 자기 자신만
        재시작하면 된다 (cascade 불필요)."""
        proc = self.procs.get(name)
        rc = proc.returncode if proc else None
        log.warning(f"'{name}' exited (returncode={rc})")

        if name in self.giving_up:
            return

        if not self._record_restart(name):
            log.error(
                f"'{name}' crashed too many times in {RESTART_WINDOW_S}s "
                f"(limit={MAX_RESTARTS_PER_PROC}); giving up on it. "
                f"Part of the AI pipeline is now stopped -- investigate cause."
            )
            self.giving_up.add(name)
            return

        time.sleep(RESTART_BACKOFF_S)
        if not _running:
            return
        self._launch(name)

    # -- 메인 루프 -----------------------------------------------------------

    def monitor_loop(self):
        while _running:
            for name in PROC_ORDER:
                if name in self.giving_up:
                    continue
                proc = self.procs.get(name)
                if proc is not None and proc.poll() is not None:
                    self.handle_crash(name)
                    break  # 한 번에 하나씩만 처리 후 루프 재시작 (상태 일관성)
            time.sleep(MONITOR_INTERVAL_S)


def setup_shared_memory():
    """자식 프로세스들이 attach할 AI_shm을 미리 생성 + 0으로 초기화.
    AI_init이 이 블록의 유일한 생성/해제 주체다."""
    log.info("creating shared memory: AI_shm")
    AI_shm.create_ai_shm().close()


def teardown_shared_memory():
    """AI_init 종료 시 AI_shm 이름 자체를 제거. 자식들은 이미 정지된 뒤이므로
    안전하게 unlink 가능."""
    log.info("unlinking shared memory: AI_shm")
    AI_shm.unlink_ai_shm()


def main():
    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    if not VENV_PYTHON.exists():
        log.warning(
            f"venv python not found at {VENV_PYTHON}; falling back to "
            f"'{sys.executable}'. mediapipe/opencv version may not match."
        )

    setup_shared_memory()

    sup = Supervisor()
    try:
        log.info(f"starting AI pipeline: {PROC_ORDER}")
        sup.startup_chain()
        sup.monitor_loop()
    finally:
        log.info("shutting down AI pipeline")
        sup.stop_all()
        teardown_shared_memory()
        log.info("done")


if __name__ == "__main__":
    main()
