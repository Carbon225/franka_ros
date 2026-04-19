import json
import os
import socket
import time
from typing import Optional

SERVER = ("127.0.0.1", 9090)

VERBOSE = os.environ.get("ROSBRIDGE_ECHO_VERBOSE", "")


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(5.0)

    sub_msg = {
        "op": "subscribe",
        "id": "sub1",
        "topic": "/franka_state_controller/franka_states",
        "type": "franka_msgs/FrankaState",
        "queue_length": 1,
    }
    sock.sendto(json.dumps(sub_msg).encode("utf-8"), SERVER)

    msg_index = 0
    t_first: Optional[float] = None
    t_prev: Optional[float] = None
    sum_inst_hz = 0.0
    n_intervals = 0
    min_hz = float("inf")
    max_hz = 0.0

    while True:
        data, _addr = sock.recvfrom(65535)
        now = time.monotonic()
        msg = json.loads(data.decode("utf-8"))

        if VERBOSE:
            print(msg)

        msg_index += 1
        if t_first is None:
            t_first = now

        if t_prev is not None:
            dt = now - t_prev
            if dt > 0.0:
                inst_hz = 1.0 / dt
                sum_inst_hz += inst_hz
                n_intervals += 1
                min_hz = min(min_hz, inst_hz)
                max_hz = max(max_hz, inst_hz)

        t_prev = now

        elapsed = now - t_first
        overall_hz = (msg_index - 1) / elapsed if elapsed > 0.0 and msg_index > 1 else 0.0
        avg_inst_hz = sum_inst_hz / n_intervals if n_intervals > 0 else 0.0

        min_s = f"{min_hz:.3f}" if n_intervals > 0 else "—"
        max_s = f"{max_hz:.3f}" if n_intervals > 0 else "—"

        print(
            f"n={msg_index}  "
            f"overall={overall_hz:.3f} Hz  "
            f"avg_inst={avg_inst_hz:.3f} Hz  "
            f"min_inst={min_s} Hz  "
            f"max_inst={max_s} Hz  "
            f"elapsed={elapsed:.3f} s"
        )


if __name__ == "__main__":
    main()
