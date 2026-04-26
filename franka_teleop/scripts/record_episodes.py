"""
Xbox-controller teleop with episode recording for the Panda over rosbridge (UDP).

This is a standalone script that combines the
cartesian + gripper teleop from ``xbox_teleop.py`` with simple episode
recording: while an episode is active, every received ``franka_msgs/FrankaState``
and every published cartesian command is appended to JSONL files inside a
freshly-created episode directory. If a camera is available, the episode
directory also contains ``video.mkv`` (FFV1, lossless) and
``video_metadata.json`` with per-frame timestamps so each frame can be
correlated with the state / command streams. With a GUI display available, the
camera feed is also shown in a live OpenCV preview window.

It also supports a one-button "go home" that smoothly drives the equilibrium
pose toward a hardcoded cartesian position.

Buttons (Xbox standard layout, A=0, B=1, X=2, Y=3):
    A     -> toggle motion enable (starts DISABLED for safety)
    Y     -> start a new episode (creates a directory and begins recording)
    B     -> stop the current episode
    X     -> move equilibrium pose to the hardcoded HOME_POSITION
    LB    -> open gripper
    RB    -> grasp
    Start -> reset target to the current robot pose
    Back  -> quit

Usage:
    python3 record_episodes.py
    python3 record_episodes.py --output-dir ~/datasets/panda_teleop \\
        --device /dev/input/js1
    python3 record_episodes.py --no-preview
"""

import argparse
import json
import math
import os
import select
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from typing import Dict, List, Optional, TextIO, Tuple

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    cv2 = None  # type: ignore

# ---------------------------------------------------------------------------
# Defaults / constants
# ---------------------------------------------------------------------------

DEFAULT_SERVER = ("127.0.0.1", 9090)
DEFAULT_DEVICE = "/dev/input/js0"
DEFAULT_TOPIC = "/cartesian_impedance_example_controller/equilibrium_pose"
DEFAULT_STATE_TOPIC = "/franka_state_controller/franka_states"
DEFAULT_FRAME = "panda_link0"
DEFAULT_GRIPPER_NS = "/franka_gripper"
DEFAULT_OUTPUT_DIR = "./episodes"

# Video capture defaults
DEFAULT_CAMERA = "0"
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 30.0
DEFAULT_PREVIEW_WINDOW = "Franka camera preview"

# Gripper goal parameters
GRIPPER_OPEN_WIDTH = 0.08            # [m]
GRIPPER_GRASP_WIDTH = 0.0            # [m]
GRIPPER_GRASP_FORCE = 20.0           # [N]
GRIPPER_GRASP_EPSILON_INNER = 0.08   # [m]
GRIPPER_GRASP_EPSILON_OUTER = 0.08   # [m]
GRIPPER_SPEED = 0.1                  # [m/s]

# Cartesian limits and motion parameters
POSITION_LIMITS = ((-0.6, 0.6), (-0.6, 0.6), (0.05, 0.9))
LINEAR_SPEED = 0.15                  # [m/s] both for teleop and "go home"
PUBLISH_RATE_HZ = 100.0
AXIS_DEADZONE = 0.08

# Hardcoded "home" / reset pose. Pick something safely inside POSITION_LIMITS.
HOME_POSITION = (0.4, 0.0, 0.5)
HOME_REACH_TOL = 0.005               # [m] consider home reached within this distance

# Commanded end-effector yaw about base Z
YAW_DEG = 90.0
_yaw_half = math.radians(YAW_DEG) * 0.5
COMMAND_QUAT = (math.cos(_yaw_half), math.sin(_yaw_half), 0.0, 0.0)

# Joystick wire format
JS_EVENT_STRUCT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 4

BTN_A = 0
BTN_B = 1
BTN_X = 2
BTN_Y = 3
BTN_LB = 4
BTN_RB = 5
BTN_BACK = 6
BTN_START = 7


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def gui_display_available() -> bool:
    """Best-effort guard against opening OpenCV windows in headless Linux."""
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


def _ros_stamp_now() -> Dict:
    now = time.time()
    secs = int(now)
    nsecs = int((now - secs) * 1e9)
    return {"secs": secs, "nsecs": nsecs}


def step_toward(
    current: Tuple[float, float, float],
    target: Tuple[float, float, float],
    max_step: float,
) -> Tuple[Tuple[float, float, float], float]:
    """Move ``current`` toward ``target`` by at most ``max_step`` metres.

    Returns the new position and the remaining distance to the target.
    """
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    dz = target[2] - current[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    if dist <= max_step or dist == 0.0:
        return target, 0.0
    scale = max_step / dist
    return (
        current[0] + dx * scale,
        current[1] + dy * scale,
        current[2] + dz * scale,
    ), dist - max_step


# ---------------------------------------------------------------------------
# Rosbridge UDP client
# ---------------------------------------------------------------------------


class RosbridgeUDPClient:
    """Tiny rosbridge JSON-over-UDP client (send + receive)."""

    def __init__(self, server: Tuple[str, int]) -> None:
        self._server = server
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(5.0)
        self._send_lock = threading.Lock()

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def send(self, payload: Dict) -> None:
        data = json.dumps(payload).encode("utf-8")
        with self._send_lock:
            self._sock.sendto(data, self._server)

    def recv(self, timeout: Optional[float] = None) -> Dict:
        if timeout is not None:
            self._sock.settimeout(timeout)
        data, _ = self._sock.recvfrom(65535)
        return json.loads(data.decode("utf-8"))

    def advertise(self, topic: str, msg_type: str) -> None:
        self.send({"op": "advertise", "topic": topic, "type": msg_type})

    def unadvertise(self, topic: str) -> None:
        self.send({"op": "unadvertise", "topic": topic})

    def publish(self, topic: str, msg: Dict) -> None:
        self.send({"op": "publish", "topic": topic, "msg": msg})

    def subscribe(self, topic: str, msg_type: str, sub_id: str) -> None:
        self.send(
            {
                "op": "subscribe",
                "id": sub_id,
                "topic": topic,
                "type": msg_type,
                "queue_length": 1,
            }
        )

    def unsubscribe(self, topic: str, sub_id: str) -> None:
        self.send({"op": "unsubscribe", "id": sub_id, "topic": topic})


# ---------------------------------------------------------------------------
# Receiver thread: keeps the latest FrankaState and feeds the recorder
# ---------------------------------------------------------------------------


class StateReceiver(threading.Thread):
    """Continuously consumes UDP messages and tracks the latest FrankaState.

    When ``recording_file`` is set, each newly-received state is appended as a
    JSON line ``{"t_wall": ..., "t_mono": ..., "msg": <FrankaState dict>}``.
    """

    def __init__(self, client: RosbridgeUDPClient, state_topic: str) -> None:
        super().__init__(daemon=True)
        self._client = client
        self._state_topic = state_topic
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_state: Optional[Dict] = None
        self._latest_t_mono: Optional[float] = None
        self._state_count = 0

        self._record_lock = threading.Lock()
        self._record_file: Optional[TextIO] = None

    def stop(self) -> None:
        self._stop.set()

    def latest(self) -> Tuple[Optional[Dict], Optional[float]]:
        with self._lock:
            return self._latest_state, self._latest_t_mono

    def state_count(self) -> int:
        with self._lock:
            return self._state_count

    def set_recording_file(self, fh: Optional[TextIO]) -> None:
        with self._record_lock:
            self._record_file = fh

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._client.recv(timeout=0.5)
            except socket.timeout:
                continue
            except OSError:
                break

            if msg.get("op") != "publish":
                continue
            topic = msg.get("topic", "")
            if not (topic == self._state_topic or topic.endswith(self._state_topic.lstrip("/"))):
                continue

            state = msg.get("msg")
            if not isinstance(state, dict) or "O_T_EE" not in state:
                continue

            t_mono = time.monotonic()
            t_wall = time.time()

            with self._lock:
                self._latest_state = state
                self._latest_t_mono = t_mono
                self._state_count += 1

            with self._record_lock:
                fh = self._record_file
                if fh is not None:
                    try:
                        fh.write(
                            json.dumps(
                                {"t_wall": t_wall, "t_mono": t_mono, "msg": state},
                                separators=(",", ":"),
                            )
                            + "\n"
                        )
                    except (OSError, ValueError):
                        pass


# ---------------------------------------------------------------------------
# Joystick thread (same wire format as xbox_teleop.py)
# ---------------------------------------------------------------------------


class JoystickReader(threading.Thread):
    def __init__(self, device_path: str) -> None:
        super().__init__(daemon=True)
        self._path = device_path
        self._fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._axes: Dict[int, float] = {}
        self._buttons: Dict[int, bool] = {}
        self._edges: Dict[int, int] = {}

    def stop(self) -> None:
        self._stop.set()
        try:
            os.close(self._fd)
        except OSError:
            pass

    def axis(self, number: int) -> float:
        with self._lock:
            return self._axes.get(number, 0.0)

    def button(self, number: int) -> bool:
        with self._lock:
            return self._buttons.get(number, False)

    def pop_press(self, number: int) -> bool:
        with self._lock:
            count = self._edges.get(number, 0)
            if count > 0:
                self._edges[number] = count - 1
                return True
            return False

    def run(self) -> None:
        buf = b""
        event_size = JS_EVENT_STRUCT.size
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, 64 * event_size)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= event_size:
                _t, value, ev_type, number = JS_EVENT_STRUCT.unpack_from(buf)
                buf = buf[event_size:]
                ev_type &= ~JS_EVENT_INIT
                if ev_type == JS_EVENT_AXIS:
                    normalized = max(-1.0, min(1.0, value / 32767.0))
                    with self._lock:
                        self._axes[number] = normalized
                elif ev_type == JS_EVENT_BUTTON:
                    pressed = bool(value)
                    with self._lock:
                        was = self._buttons.get(number, False)
                        self._buttons[number] = pressed
                        if pressed and not was:
                            self._edges[number] = self._edges.get(number, 0) + 1


# ---------------------------------------------------------------------------
# ROS message builders (rosbridge JSON dicts)
# ---------------------------------------------------------------------------


def build_pose_msg(
    frame_id: str,
    position: Tuple[float, float, float],
    quat: Tuple[float, float, float, float],
) -> Dict:
    return {
        "header": {
            "seq": 0,
            "stamp": _ros_stamp_now(),
            "frame_id": frame_id,
        },
        "pose": {
            "position": {"x": position[0], "y": position[1], "z": position[2]},
            "orientation": {
                "x": quat[0],
                "y": quat[1],
                "z": quat[2],
                "w": quat[3],
            },
        },
    }


def build_move_goal(width: float, speed: float, goal_id: str) -> Dict:
    stamp = _ros_stamp_now()
    return {
        "header": {"seq": 0, "stamp": stamp, "frame_id": ""},
        "goal_id": {"stamp": stamp, "id": goal_id},
        "goal": {"width": float(width), "speed": float(speed)},
    }


def build_grasp_goal(
    width: float,
    speed: float,
    force: float,
    epsilon_inner: float,
    epsilon_outer: float,
    goal_id: str,
) -> Dict:
    stamp = _ros_stamp_now()
    return {
        "header": {"seq": 0, "stamp": stamp, "frame_id": ""},
        "goal_id": {"stamp": stamp, "id": goal_id},
        "goal": {
            "width": float(width),
            "epsilon": {
                "inner": float(epsilon_inner),
                "outer": float(epsilon_outer),
            },
            "speed": float(speed),
            "force": float(force),
        },
    }


# ---------------------------------------------------------------------------
# Video recorder (OpenCV + FFV1/MKV)
# ---------------------------------------------------------------------------


class VideoRecorder(threading.Thread):
    """Grabs frames from a camera and, while an episode is active, writes them
    losslessly to ``video.mkv`` (FFV1 in Matroska). Per-frame timestamps are
    accumulated in memory and flushed to ``video_metadata.json`` on stop.
    """

    def __init__(
        self,
        device: object,
        width: int,
        height: int,
        fps: float,
        preview: bool = False,
        preview_window: str = DEFAULT_PREVIEW_WINDOW,
    ) -> None:
        super().__init__(daemon=True)
        if cv2 is None:
            raise RuntimeError(
                "OpenCV (cv2) is not available; install opencv-python or run with --no-video"
            )

        self._requested_device = device
        self._requested_fps = float(fps)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._preview_enabled = bool(preview)
        self._preview_window = preview_window
        self._preview_created = False
        self._preview_warning_printed = False

        self._cap = cv2.VideoCapture(device)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open camera device {device!r}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self._cap.set(cv2.CAP_PROP_FPS, float(fps))

        # Grab a probe frame to discover the actual frame size the driver will
        # deliver; the VideoWriter needs an exact match or it silently fails.
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._cap.release()
            raise RuntimeError(f"camera device {device!r} opened but produced no frame")
        actual_h, actual_w = frame.shape[:2]
        self._width = int(actual_w)
        self._height = int(actual_h)
        reported_fps = float(self._cap.get(cv2.CAP_PROP_FPS) or 0.0)
        self._fps = reported_fps if reported_fps > 1.0 else float(fps)

        self._writer: Optional["cv2.VideoWriter"] = None
        self._ep_dir: Optional[str] = None
        self._frames: List[Dict] = []
        self._episode_start_t_wall: Optional[float] = None
        self._episode_start_t_mono: Optional[float] = None
        self._frames_dropped = 0
        self._total_frames_grabbed = 0
        self._latest_frame = frame.copy() if self._preview_enabled else None

    @property
    def width(self) -> int:
        return self._width

    @property
    def height(self) -> int:
        return self._height

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def preview_enabled(self) -> bool:
        return self._preview_enabled

    def stop(self) -> None:
        self._stop.set()

    def close_preview(self) -> None:
        """Close the preview window and stop retaining preview frames."""
        with self._lock:
            self._preview_enabled = False
            self._latest_frame = None

        if self._preview_created:
            try:
                cv2.destroyWindow(self._preview_window)
                cv2.waitKey(1)
            except Exception:  # pragma: no cover - depends on OpenCV GUI backend
                pass
        self._preview_created = False

    def poll_preview(self, recording: bool, enabled: bool, going_home: bool) -> None:
        """Render the latest frame and service OpenCV GUI events.

        OpenCV HighGUI calls are deliberately kept out of the capture thread;
        some backends expect window operations to happen on the main thread.
        """
        if not self._preview_enabled:
            return

        with self._lock:
            latest = None if self._latest_frame is None else self._latest_frame.copy()

        if latest is None:
            return

        try:
            if not self._preview_created:
                cv2.namedWindow(self._preview_window, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    self._preview_window,
                    min(self._width, 960),
                    min(self._height, 720),
                )
                self._preview_created = True

            label = "REC" if recording else "IDLE"
            mode = "HOME" if going_home else ("ON" if enabled else "OFF")
            overlay = latest
            cv2.rectangle(overlay, (0, 0), (overlay.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(
                overlay,
                f"{label}  motion:{mode}",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255) if recording else (220, 220, 220),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self._preview_window, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                print("\n[record] camera preview closed.")
                self.close_preview()
                return

            try:
                visible = cv2.getWindowProperty(self._preview_window, cv2.WND_PROP_VISIBLE)
            except Exception:  # pragma: no cover - backend-specific behavior
                visible = 1.0
            if visible < 1.0:
                self.close_preview()
        except Exception as exc:  # pragma: no cover - depends on OpenCV GUI backend
            if not self._preview_warning_printed:
                print(
                    f"\n[record] WARNING: camera preview disabled: {exc}",
                    file=sys.stderr,
                )
                self._preview_warning_printed = True
            self.close_preview()

    def start_episode(self, ep_dir: str) -> Optional[str]:
        """Begin writing frames into ``<ep_dir>/video.mkv``. Safe to call while
        the capture thread is running. Returns the video path on success."""
        assert cv2 is not None
        fourcc = cv2.VideoWriter_fourcc(*"FFV1")
        video_path = os.path.join(ep_dir, "video.mkv")
        writer = cv2.VideoWriter(
            video_path, fourcc, self._fps, (self._width, self._height), True
        )
        if not writer.isOpened():
            print(
                f"[record] WARNING: could not open VideoWriter with FFV1 at {video_path}; "
                "video will not be recorded for this episode.",
                file=sys.stderr,
            )
            return None
        with self._lock:
            self._writer = writer
            self._ep_dir = ep_dir
            self._frames = []
            self._episode_start_t_wall = time.time()
            self._episode_start_t_mono = time.monotonic()
        return video_path

    def stop_episode(self) -> None:
        """Close the VideoWriter and flush ``video_metadata.json``."""
        with self._lock:
            writer = self._writer
            ep_dir = self._ep_dir
            frames = self._frames
            start_wall = self._episode_start_t_wall
            start_mono = self._episode_start_t_mono
            self._writer = None
            self._ep_dir = None
            self._frames = []
            self._episode_start_t_wall = None
            self._episode_start_t_mono = None

        if writer is not None:
            try:
                writer.release()
            except Exception:  # pragma: no cover - defensive
                pass

        if ep_dir is None:
            return

        meta = {
            "video_file": "video.mkv",
            "codec": "FFV1",
            "container": "matroska",
            "pixel_format": "bgr24",
            "width": self._width,
            "height": self._height,
            "fps": self._fps,
            "requested_device": repr(self._requested_device),
            "requested_fps": self._requested_fps,
            "frame_count": len(frames),
            "start_wall_time": start_wall,
            "start_mono_time": start_mono,
            "frames": frames,
        }
        try:
            with open(os.path.join(ep_dir, "video_metadata.json"), "w") as fh:
                json.dump(meta, fh, indent=2)
        except OSError as exc:
            print(
                f"[record] WARNING: could not write video_metadata.json: {exc}",
                file=sys.stderr,
            )

    def run(self) -> None:
        assert cv2 is not None
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok or frame is None:
                # Camera hiccup: back off briefly and try again.
                time.sleep(0.005)
                continue

            t_wall = time.time()
            t_mono = time.monotonic()
            self._total_frames_grabbed += 1
            if frame.shape[1] != self._width or frame.shape[0] != self._height:
                frame = cv2.resize(frame, (self._width, self._height))

            with self._lock:
                if self._preview_enabled:
                    self._latest_frame = frame.copy()

                writer = self._writer
                if writer is None:
                    continue
                try:
                    writer.write(frame)
                except Exception:  # pragma: no cover - defensive
                    self._frames_dropped += 1
                    continue
                self._frames.append(
                    {
                        "index": len(self._frames),
                        "t_wall": t_wall,
                        "t_mono": t_mono,
                    }
                )

        try:
            self._cap.release()
        except Exception:  # pragma: no cover - defensive
            pass


# ---------------------------------------------------------------------------
# Episode recorder
# ---------------------------------------------------------------------------


class EpisodeRecorder:
    """Owns the per-episode directory and append-only JSONL files."""

    def __init__(
        self,
        output_dir: str,
        receiver: StateReceiver,
        args: argparse.Namespace,
        video: Optional[VideoRecorder] = None,
    ) -> None:
        self._output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self._receiver = receiver
        self._args = args
        self._video = video
        self._active = False
        self._dir: Optional[str] = None
        self._states_fh: Optional[TextIO] = None
        self._commands_fh: Optional[TextIO] = None
        self._events_fh: Optional[TextIO] = None
        self._meta: Dict = {}
        self._command_count = 0
        self._start_state_count = 0

    @property
    def active(self) -> bool:
        return self._active

    @property
    def directory(self) -> Optional[str]:
        return self._dir

    def start(self) -> Optional[str]:
        if self._active:
            print("[record] episode already in progress; ignoring start", file=sys.stderr)
            return None

        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ep_id = f"episode_{ts}"
        ep_dir = os.path.join(self._output_dir, ep_id)
        # Avoid clobbering if two episodes start within the same second.
        suffix = 1
        unique_dir = ep_dir
        while os.path.exists(unique_dir):
            unique_dir = f"{ep_dir}_{suffix}"
            suffix += 1
        os.makedirs(unique_dir)

        # Line-buffered text mode so partial recordings survive a crash.
        states_fh = open(os.path.join(unique_dir, "states.jsonl"), "w", buffering=1)
        commands_fh = open(os.path.join(unique_dir, "commands.jsonl"), "w", buffering=1)
        events_fh = open(os.path.join(unique_dir, "events.jsonl"), "w", buffering=1)

        self._dir = unique_dir
        self._states_fh = states_fh
        self._commands_fh = commands_fh
        self._events_fh = events_fh
        self._command_count = 0
        self._start_state_count = self._receiver.state_count()
        self._meta = {
            "episode_id": os.path.basename(unique_dir),
            "start_wall_time": time.time(),
            "start_iso": datetime.now().isoformat(timespec="seconds"),
            "host": self._args.host,
            "port": self._args.port,
            "command_topic": self._args.topic,
            "state_topic": self._args.state_topic,
            "frame_id": self._args.frame_id,
            "gripper_ns": self._args.gripper_ns,
            "publish_rate_hz": PUBLISH_RATE_HZ,
            "command_quat_xyzw": list(COMMAND_QUAT),
            "home_position": list(HOME_POSITION),
        }

        video_path: Optional[str] = None
        if self._video is not None:
            try:
                video_path = self._video.start_episode(unique_dir)
            except Exception as exc:  # pragma: no cover - defensive
                print(
                    f"[record] WARNING: video recording failed to start: {exc}",
                    file=sys.stderr,
                )
                video_path = None
        if video_path is not None:
            self._meta["video_file"] = os.path.basename(video_path)
            self._meta["video_metadata_file"] = "video_metadata.json"

        self._receiver.set_recording_file(states_fh)
        self._active = True
        self.log_event("episode_started", {"directory": unique_dir})
        print(f"[record] >>> episode started: {unique_dir}")
        return unique_dir

    def stop(self) -> Optional[str]:
        if not self._active:
            print("[record] no episode in progress; ignoring stop", file=sys.stderr)
            return None

        self.log_event("episode_stopped", {})

        # Detach the recorder file from the state thread first, so no further
        # appends race with close().
        self._receiver.set_recording_file(None)

        if self._video is not None:
            try:
                self._video.stop_episode()
            except Exception as exc:  # pragma: no cover - defensive
                print(
                    f"[record] WARNING: video recording failed to stop cleanly: {exc}",
                    file=sys.stderr,
                )

        states_count = self._receiver.state_count() - self._start_state_count
        self._meta.update(
            {
                "end_wall_time": time.time(),
                "end_iso": datetime.now().isoformat(timespec="seconds"),
                "duration_sec": time.time() - self._meta.get("start_wall_time", time.time()),
                "command_count": self._command_count,
                "state_count": states_count,
            }
        )
        meta_path = os.path.join(self._dir, "meta.json") if self._dir else None
        if meta_path:
            try:
                with open(meta_path, "w") as fh:
                    json.dump(self._meta, fh, indent=2)
            except OSError as exc:
                print(f"[record] WARNING: could not write meta.json: {exc}", file=sys.stderr)

        for fh in (self._states_fh, self._commands_fh, self._events_fh):
            if fh is not None:
                try:
                    fh.close()
                except OSError:
                    pass
        self._states_fh = None
        self._commands_fh = None
        self._events_fh = None

        ep_dir = self._dir
        self._dir = None
        self._active = False
        print(
            f"[record] <<< episode stopped: {ep_dir} "
            f"({self._command_count} cmds, {states_count} states, "
            f"{self._meta.get('duration_sec', 0.0):.2f} s)"
        )
        return ep_dir

    def log_command(
        self,
        position: Tuple[float, float, float],
        quat: Tuple[float, float, float, float],
        enabled: bool,
        going_home: bool,
    ) -> None:
        if not self._active or self._commands_fh is None:
            return
        try:
            self._commands_fh.write(
                json.dumps(
                    {
                        "t_wall": time.time(),
                        "t_mono": time.monotonic(),
                        "position": list(position),
                        "orientation_xyzw": list(quat),
                        "enabled": bool(enabled),
                        "going_home": bool(going_home),
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
            self._command_count += 1
        except (OSError, ValueError):
            pass

    def log_event(self, name: str, payload: Optional[Dict] = None) -> None:
        if not self._active or self._events_fh is None:
            return
        try:
            self._events_fh.write(
                json.dumps(
                    {
                        "t_wall": time.time(),
                        "t_mono": time.monotonic(),
                        "event": name,
                        "data": payload or {},
                    },
                    separators=(",", ":"),
                )
                + "\n"
            )
        except (OSError, ValueError):
            pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def position_from_state(state: Dict) -> Tuple[float, float, float]:
    o_t_ee = state["O_T_EE"]
    return (float(o_t_ee[12]), float(o_t_ee[13]), float(o_t_ee[14]))


def wait_for_first_state(receiver: StateReceiver, timeout: float = 5.0) -> Dict:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state, _ = receiver.latest()
        if state is not None:
            return state
        time.sleep(0.05)
    raise socket.timeout("timed out waiting for first FrankaState")


def run(args: argparse.Namespace) -> int:
    client = RosbridgeUDPClient((args.host, args.port))

    # Subscribe to franka_states first, then start the receiver thread that
    # consumes everything coming back over UDP.
    client.subscribe(args.state_topic, "franka_msgs/FrankaState", "record_episodes_state")
    receiver = StateReceiver(client, args.state_topic)
    receiver.start()

    print(f"[record] waiting for first state on {args.state_topic} ...")
    try:
        first_state = wait_for_first_state(receiver, timeout=5.0)
    except socket.timeout:
        print(
            "[record] ERROR: no franka_states received. Is rosbridge_udp running "
            f"on {args.host}:{args.port} and is the robot publishing?",
            file=sys.stderr,
        )
        receiver.stop()
        client.close()
        return 1

    position = position_from_state(first_state)
    initial_position = position
    print(
        f"[record] initial position: ({position[0]:+.3f}, {position[1]:+.3f}, "
        f"{position[2]:+.3f}) | quat=({COMMAND_QUAT[0]:.3f}, "
        f"{COMMAND_QUAT[1]:.3f}, {COMMAND_QUAT[2]:.3f}, {COMMAND_QUAT[3]:.3f})"
    )

    try:
        joystick = JoystickReader(args.device)
    except OSError as exc:
        print(f"[record] ERROR: cannot open joystick {args.device}: {exc}", file=sys.stderr)
        receiver.stop()
        client.close()
        return 1
    joystick.start()

    client.advertise(args.topic, "geometry_msgs/PoseStamped")
    gripper_ns = args.gripper_ns.rstrip("/")
    move_goal_topic = f"{gripper_ns}/move/goal"
    grasp_goal_topic = f"{gripper_ns}/grasp/goal"
    client.advertise(move_goal_topic, "franka_gripper/MoveActionGoal")
    client.advertise(grasp_goal_topic, "franka_gripper/GraspActionGoal")
    time.sleep(0.1)

    video: Optional[VideoRecorder] = None
    if not args.no_video:
        if cv2 is None:
            print(
                "[record] WARNING: OpenCV not available; continuing without video. "
                "Install opencv-python or pass --no-video to silence this.",
                file=sys.stderr,
            )
        else:
            preview_enabled = args.preview if args.preview is not None else True
            if preview_enabled and args.preview is None and not gui_display_available():
                preview_enabled = False
                print(
                    "[record] camera preview disabled: no GUI display detected. "
                    "Pass --preview to force it.",
                    file=sys.stderr,
                )
            device: object = args.camera
            if isinstance(device, str) and device.isdigit():
                device = int(device)
            try:
                video = VideoRecorder(
                    device=device,
                    width=args.camera_width,
                    height=args.camera_height,
                    fps=args.camera_fps,
                    preview=preview_enabled,
                )
                video.start()
                print(
                    f"[record] camera ready: device={args.camera} "
                    f"{video.width}x{video.height} @ {video.fps:.1f} fps (FFV1/mkv"
                    f"{', preview' if video.preview_enabled else ''})"
                )
                if video.preview_enabled:
                    print("[record] preview window: q/Esc closes preview")
            except Exception as exc:
                print(
                    f"[record] WARNING: could not initialise camera: {exc}. "
                    "Continuing without video.",
                    file=sys.stderr,
                )
                video = None

    recorder = EpisodeRecorder(args.output_dir, receiver, args, video=video)

    enabled = False
    going_home = False
    gripper_seq = 0
    dt = 1.0 / PUBLISH_RATE_HZ

    print(
        f"[record] publishing {args.topic} at {PUBLISH_RATE_HZ:.0f} Hz.\n"
        "         A: enable/disable | Y: start episode | B: stop episode |\n"
        "         X: go to home | LB: open | RB: grasp |\n"
        "         Start: reset target to current pose | Back: quit"
    )

    try:
        next_tick = time.monotonic()
        last_status = 0.0
        while True:
            if joystick.pop_press(BTN_BACK):
                print("\n[record] Back pressed, exiting.")
                break

            if joystick.pop_press(BTN_A):
                enabled = not enabled
                print(f"\n[record] motion {'ENABLED' if enabled else 'DISABLED'}")
                recorder.log_event("motion_enable", {"enabled": enabled})
                if not enabled:
                    going_home = False  # cancel any in-progress home motion

            if joystick.pop_press(BTN_Y):
                ep_dir = recorder.start()
                if ep_dir is not None:
                    # Snapshot the initial state so consumers can recover the
                    # episode start pose without searching states.jsonl.
                    state, _ = receiver.latest()
                    if state is not None:
                        try:
                            with open(os.path.join(ep_dir, "initial_state.json"), "w") as fh:
                                json.dump(state, fh)
                        except OSError:
                            pass

            if joystick.pop_press(BTN_B):
                recorder.stop()

            if joystick.pop_press(BTN_X):
                if not enabled:
                    print(
                        "\n[record] go-home ignored: motion is DISABLED. "
                        "Press A to enable first."
                    )
                else:
                    going_home = True
                    print(
                        f"\n[record] going home -> ({HOME_POSITION[0]:+.3f}, "
                        f"{HOME_POSITION[1]:+.3f}, {HOME_POSITION[2]:+.3f})"
                    )
                    recorder.log_event("go_home_requested", {"target": list(HOME_POSITION)})

            if joystick.pop_press(BTN_START):
                state, _ = receiver.latest()
                if state is not None:
                    position = position_from_state(state)
                    initial_position = position
                else:
                    position = initial_position
                going_home = False
                print(
                    f"\n[record] target reset to current pose "
                    f"({position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f})"
                )
                recorder.log_event("target_reset", {"position": list(position)})

            if joystick.pop_press(BTN_LB):
                gripper_seq += 1
                goal_id = f"record_open_{gripper_seq}"
                client.publish(
                    move_goal_topic,
                    build_move_goal(GRIPPER_OPEN_WIDTH, GRIPPER_SPEED, goal_id),
                )
                print(f"\n[record] gripper OPEN (width={GRIPPER_OPEN_WIDTH:.3f} m)")
                recorder.log_event(
                    "gripper_open",
                    {"width": GRIPPER_OPEN_WIDTH, "speed": GRIPPER_SPEED, "goal_id": goal_id},
                )

            if joystick.pop_press(BTN_RB):
                gripper_seq += 1
                goal_id = f"record_grasp_{gripper_seq}"
                client.publish(
                    grasp_goal_topic,
                    build_grasp_goal(
                        GRIPPER_GRASP_WIDTH,
                        GRIPPER_SPEED,
                        GRIPPER_GRASP_FORCE,
                        GRIPPER_GRASP_EPSILON_INNER,
                        GRIPPER_GRASP_EPSILON_OUTER,
                        goal_id,
                    ),
                )
                print(
                    f"\n[record] gripper GRASP (width={GRIPPER_GRASP_WIDTH:.3f} m, "
                    f"force={GRIPPER_GRASP_FORCE:.1f} N)"
                )
                recorder.log_event(
                    "gripper_grasp",
                    {
                        "width": GRIPPER_GRASP_WIDTH,
                        "speed": GRIPPER_SPEED,
                        "force": GRIPPER_GRASP_FORCE,
                        "epsilon_inner": GRIPPER_GRASP_EPSILON_INNER,
                        "epsilon_outer": GRIPPER_GRASP_EPSILON_OUTER,
                        "goal_id": goal_id,
                    },
                )

            if going_home and enabled:
                position, remaining = step_toward(
                    position, HOME_POSITION, LINEAR_SPEED * dt
                )
                if remaining <= HOME_REACH_TOL:
                    going_home = False
                    print("\n[record] home reached.")
                    recorder.log_event("go_home_done", {"position": list(position)})
            elif enabled:
                lx = apply_deadzone(joystick.axis(AXIS_LEFT_X), AXIS_DEADZONE)
                ly = apply_deadzone(joystick.axis(AXIS_LEFT_Y), AXIS_DEADZONE)
                ry = apply_deadzone(joystick.axis(AXIS_RIGHT_Y), AXIS_DEADZONE)
                vx = lx * LINEAR_SPEED
                vy = -ly * LINEAR_SPEED
                vz = -ry * LINEAR_SPEED
                position = (
                    clamp(position[0] + vx * dt, *POSITION_LIMITS[0]),
                    clamp(position[1] + vy * dt, *POSITION_LIMITS[1]),
                    clamp(position[2] + vz * dt, *POSITION_LIMITS[2]),
                )

            client.publish(args.topic, build_pose_msg(args.frame_id, position, COMMAND_QUAT))
            recorder.log_command(position, COMMAND_QUAT, enabled, going_home)
            if video is not None:
                video.poll_preview(recorder.active, enabled, going_home)

            now = time.monotonic()
            if now - last_status > 0.5:
                last_status = now
                rec_str = "REC" if recorder.active else "---"
                mode = "HOME" if going_home else ("ON" if enabled else "OFF")
                sys.stdout.write(
                    "\r[record] {:<4} {:<4} pos=({:+.3f},{:+.3f},{:+.3f}) "
                    "states={}            ".format(
                        rec_str,
                        mode,
                        position[0],
                        position[1],
                        position[2],
                        receiver.state_count(),
                    )
                )
                sys.stdout.flush()

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\n[record] interrupted.")
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        if recorder.active:
            recorder.stop()
        try:
            client.unsubscribe(args.state_topic, "record_episodes_state")
        except OSError:
            pass
        for t in (args.topic, move_goal_topic, grasp_goal_topic):
            try:
                client.unadvertise(t)
            except OSError:
                pass
        joystick.stop()
        receiver.stop()
        if video is not None:
            video.close_preview()
            video.stop()
        client.close()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_SERVER[0], help="rosbridge UDP host")
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER[1], help="rosbridge UDP port")
    parser.add_argument(
        "--device", default=DEFAULT_DEVICE, help="joystick device, e.g. /dev/input/js0"
    )
    parser.add_argument(
        "--topic", default=DEFAULT_TOPIC, help="equilibrium_pose topic to publish"
    )
    parser.add_argument(
        "--state-topic",
        default=DEFAULT_STATE_TOPIC,
        help="franka_states topic used for the initial pose and recording",
    )
    parser.add_argument(
        "--frame-id", default=DEFAULT_FRAME, help="header.frame_id for published poses"
    )
    parser.add_argument(
        "--gripper-ns",
        default=DEFAULT_GRIPPER_NS,
        help="franka_gripper action namespace (e.g. /franka_gripper)",
    )
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="root directory under which episode_<timestamp>/ folders are created",
    )
    parser.add_argument(
        "--camera",
        default=DEFAULT_CAMERA,
        help=(
            "camera device passed to cv2.VideoCapture. Integer strings become "
            "an index (e.g. '0'); other values are passed through as-is "
            "(e.g. '/dev/video0' or a GStreamer pipeline)."
        ),
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=DEFAULT_CAMERA_WIDTH,
        help="requested camera frame width in pixels",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=DEFAULT_CAMERA_HEIGHT,
        help="requested camera frame height in pixels",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        default=DEFAULT_CAMERA_FPS,
        help="requested camera frame rate (used for the FFV1 VideoWriter)",
    )
    preview_group = parser.add_mutually_exclusive_group()
    preview_group.add_argument(
        "--preview",
        dest="preview",
        action="store_true",
        help=(
            "show a live OpenCV camera preview window; this is the default when "
            "a GUI display is detected"
        ),
    )
    preview_group.add_argument(
        "--no-preview",
        dest="preview",
        action="store_false",
        help="disable the live camera preview window while keeping video recording enabled",
    )
    parser.set_defaults(preview=None)
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="disable camera capture / video recording entirely",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run(parse_args()))
