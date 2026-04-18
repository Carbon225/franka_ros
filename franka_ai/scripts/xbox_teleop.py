"""
Xbox controller teleop for the Panda over rosbridge (UDP).

Reads a wired/wireless Xbox controller from /dev/input/jsX and publishes
geometry_msgs/PoseStamped to the cartesian impedance controller's
equilibrium_pose topic through rosbridge.

Control scheme (defaults):
    Left stick X    -> translate +X right / -X left (base frame)
    Left stick Y    -> translate +Y up / -Y down
    Right stick Y   -> translate +Z up / -Z down
    LB              -> open gripper (franka_gripper/move to GRIPPER_OPEN_WIDTH)
    RB              -> grasp      (franka_gripper/grasp to GRIPPER_GRASP_WIDTH)
    A               -> toggle motion enable (starts DISABLED for safety)
    Start           -> reset target to current robot pose
    Back            -> quit

Usage:
    python3 xbox_teleop.py
    python3 xbox_teleop.py --device /dev/input/js1 \\
        --topic /cartesian_impedance_example_controller/equilibrium_pose
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
from typing import Dict, Optional, Tuple

DEFAULT_SERVER = ("127.0.0.1", 9090)
DEFAULT_DEVICE = "/dev/input/js0"
DEFAULT_TOPIC = "/cartesian_impedance_example_controller/equilibrium_pose"
DEFAULT_STATE_TOPIC = "/franka_state_controller/franka_states"
DEFAULT_FRAME = "panda_link0"
DEFAULT_GRIPPER_NS = "/franka_gripper"

# Gripper goal parameters
GRIPPER_OPEN_WIDTH = 0.08   # [m]
GRIPPER_GRASP_WIDTH = 0.0   # [m] desired finger gap while grasping
GRIPPER_GRASP_FORCE = 20.0  # [N]
GRIPPER_GRASP_EPSILON_INNER = 0.08  # [m] tolerate any smaller width (success even if empty)
GRIPPER_GRASP_EPSILON_OUTER = 0.08  # [m] tolerate any larger width
GRIPPER_SPEED = 0.1         # [m/s]

POSITION_LIMITS = ((-0.6, 0.6), (-0.6, 0.6), (0.05, 0.9))

# Commanded end-effector yaw about base Z
YAW_DEG = 0.0

_yaw_half = math.radians(YAW_DEG) * 0.5
COMMAND_QUAT = (math.cos(_yaw_half), math.sin(_yaw_half), 0.0, 0.0)

LINEAR_SPEED = 0.15
PUBLISH_RATE_HZ = 100.0
AXIS_DEADZONE = 0.08

JS_EVENT_STRUCT = struct.Struct("IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_Y = 4

BTN_A = 0
BTN_LB = 4
BTN_RB = 5
BTN_BACK = 6
BTN_START = 7


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class RosbridgeUDPClient:
    def __init__(self, server: Tuple[str, int]) -> None:
        self._server = server
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(5.0)

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def send(self, payload: Dict) -> None:
        self._sock.sendto(json.dumps(payload).encode("utf-8"), self._server)

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


def fetch_initial_position(
    client: RosbridgeUDPClient, topic: str
) -> Tuple[float, float, float]:
    """Subscribe once to franka_states and return the current EE position."""
    sub_id = "xbox_teleop_init"
    client.subscribe(topic, "franka_msgs/FrankaState", sub_id)
    try:
        while True:
            msg = client.recv(timeout=5.0)
            if msg.get("op") == "publish" and msg.get("topic", "").endswith(
                topic.lstrip("/")
            ):
                state = msg["msg"]
                break
            if msg.get("op") == "publish" and "O_T_EE" in msg.get("msg", {}):
                state = msg["msg"]
                break
    finally:
        client.unsubscribe(topic, sub_id)

    o_t_ee = state["O_T_EE"]
    return (float(o_t_ee[12]), float(o_t_ee[13]), float(o_t_ee[14]))


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
        """Return True once per press of a given button (rising edge)."""
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


def _ros_stamp_now() -> Dict:
    now = time.time()
    secs = int(now)
    nsecs = int((now - secs) * 1e9)
    return {"secs": secs, "nsecs": nsecs}


def build_move_goal(width: float, speed: float, goal_id: str) -> Dict:
    """franka_gripper/MoveActionGoal"""
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
    """franka_gripper/GraspActionGoal"""
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


def run_teleop(args: argparse.Namespace) -> int:
    client = RosbridgeUDPClient((args.host, args.port))

    print(f"[xbox_teleop] fetching initial pose from {args.state_topic} ...")
    try:
        position = fetch_initial_position(client, args.state_topic)
    except socket.timeout:
        print(
            "[xbox_teleop] ERROR: timed out waiting for franka_states. "
            "Is rosbridge_udp running on {}:{} and is the robot publishing?".format(
                args.host, args.port
            ),
            file=sys.stderr,
        )
        client.close()
        return 1

    print(
        "[xbox_teleop] initial position: "
        f"({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
        f"| hardcoded quat=({COMMAND_QUAT[0]:.3f}, {COMMAND_QUAT[1]:.3f}, "
        f"{COMMAND_QUAT[2]:.3f}, {COMMAND_QUAT[3]:.3f})"
    )

    initial_position = position

    try:
        joystick = JoystickReader(args.device)
    except OSError as exc:
        print(
            f"[xbox_teleop] ERROR: cannot open joystick {args.device}: {exc}",
            file=sys.stderr,
        )
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

    enabled = False
    gripper_seq = 0
    dt = 1.0 / PUBLISH_RATE_HZ
    print(
        f"[xbox_teleop] publishing {args.topic} at {PUBLISH_RATE_HZ:.0f} Hz. "
        "Press A to ENABLE, LB to open gripper, RB to grasp, Start to reset, "
        "Back to quit."
    )

    try:
        next_tick = time.monotonic()
        last_status = 0.0
        while True:
            if joystick.pop_press(BTN_BACK):
                print("[xbox_teleop] Back pressed, exiting.")
                break
            if joystick.pop_press(BTN_A):
                enabled = not enabled
                print(f"[xbox_teleop] motion {'ENABLED' if enabled else 'DISABLED'}")
            if joystick.pop_press(BTN_START):
                position = initial_position
                print("[xbox_teleop] target reset to initial position")
            if joystick.pop_press(BTN_LB):
                gripper_seq += 1
                goal_id = f"xbox_teleop_open_{gripper_seq}"
                client.publish(
                    move_goal_topic,
                    build_move_goal(GRIPPER_OPEN_WIDTH, GRIPPER_SPEED, goal_id),
                )
                print(
                    f"[xbox_teleop] gripper OPEN -> move width={GRIPPER_OPEN_WIDTH:.3f} m"
                )
            if joystick.pop_press(BTN_RB):
                gripper_seq += 1
                goal_id = f"xbox_teleop_grasp_{gripper_seq}"
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
                    f"[xbox_teleop] gripper GRASP -> width={GRIPPER_GRASP_WIDTH:.3f} m "
                    f"force={GRIPPER_GRASP_FORCE:.1f} N"
                )

            lx = apply_deadzone(joystick.axis(AXIS_LEFT_X), AXIS_DEADZONE)
            ly = apply_deadzone(joystick.axis(AXIS_LEFT_Y), AXIS_DEADZONE)
            ry = apply_deadzone(joystick.axis(AXIS_RIGHT_Y), AXIS_DEADZONE)

            vx = lx * LINEAR_SPEED
            vy = -ly * LINEAR_SPEED
            vz = -ry * LINEAR_SPEED

            if enabled:
                position = (
                    clamp(position[0] + vx * dt, *POSITION_LIMITS[0]),
                    clamp(position[1] + vy * dt, *POSITION_LIMITS[1]),
                    clamp(position[2] + vz * dt, *POSITION_LIMITS[2]),
                )

            client.publish(args.topic, build_pose_msg(args.frame_id, position, COMMAND_QUAT))

            now = time.monotonic()
            if now - last_status > 0.5:
                last_status = now
                sys.stdout.write(
                    "\r[xbox_teleop] enabled={:<5} pos=({:+.3f},{:+.3f},{:+.3f})   ".format(
                        "ON" if enabled else "OFF",
                        position[0], position[1], position[2],
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
        print("\n[xbox_teleop] interrupted.")
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        for t in (args.topic, move_goal_topic, grasp_goal_topic):
            try:
                client.unadvertise(t)
            except OSError:
                pass
        joystick.stop()
        client.close()

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_SERVER[0], help="rosbridge UDP host")
    parser.add_argument("--port", type=int, default=DEFAULT_SERVER[1], help="rosbridge UDP port")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="joystick device, e.g. /dev/input/js0")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="equilibrium_pose topic to publish")
    parser.add_argument(
        "--state-topic",
        default=DEFAULT_STATE_TOPIC,
        help="franka_states topic used to grab the initial pose",
    )
    parser.add_argument("--frame-id", default=DEFAULT_FRAME, help="header.frame_id for published poses")
    parser.add_argument(
        "--gripper-ns",
        default=DEFAULT_GRIPPER_NS,
        help="franka_gripper action namespace (e.g. /franka_gripper)",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run_teleop(parse_args()))
