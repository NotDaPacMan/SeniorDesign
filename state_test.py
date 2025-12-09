#!/usr/bin/env python3
import socket, json, time, threading, signal, sys
from pymycobot import MyCobot280

# ===== CONFIG =====
ROBOT_PORT   = "/dev/ttyAMA0"
ROBOT_BAUD   = 1000000

RECORDER_IP  = "172.16.51.114"   # <-- your PC IP
STATE_PORT   = 5556              # record_dataset_test.py listens here for state
ACTION_PORT  = 5557              # record_dataset_test.py listens here for action
CONTROL_PORT = 5558              # TCP port that accepts control commands from the recorder
SEND_HZ      = 10.0              # state/action update rate (Hz)
ENCODER_SAMPLE_HZ = 60.0         # background encoder polling for smooth velocities
# ==================

_running = True
mc = None
sock_state = None
sock_action = None
control_server = None
robot_lock = threading.Lock()
current_run_id = None
encoder_monitor = None
ENCODER_TO_DEG = 360.0 / 4096.0

def _now(): return time.time()
def safe_list(x): return list(x) if x is not None else None

def send_udp_json(sock, ip, port, obj):
    try:
        sock.sendto(json.dumps(obj).encode("utf-8"), (ip, port))
    except Exception as e:
        print(f"[WARN udp:{port}] {e}")

def send_action(
    *,
    joint_pos=None,
    speed=None,
    joint_vel=None,
    gripper_pos=None,
    gripper_vel=None,
    cartesian_pos=None,
    cartesian_vel=None,
):
    """Echo an action packet the recorder expects."""
    if joint_vel is not None:
        joint_velocity = safe_list(joint_vel)
    elif speed is not None and joint_pos is not None:
        joint_velocity = [speed] * len(joint_pos)
    else:
        joint_velocity = None

    pkt = {
        "type": "action",
        "timestamp": _now(),
        "joint_position": joint_pos,
        "joint_velocity": joint_velocity,
        "cartesian_position": cartesian_pos,
        "cartesian_velocity": cartesian_vel,
        "gripper_position": gripper_pos,
        "gripper_velocity": gripper_vel,
    }
    send_udp_json(sock_action, RECORDER_IP, ACTION_PORT, pkt)


class EncoderMonitor:
    """Poll encoders at high frequency to expose smoother joint velocities."""

    def __init__(self, sample_hz: float):
        self.sample_dt = 1.0 / max(1.0, sample_hz)
        self._thread = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._angles = None
        self._velocities = None
        self._timestamp = None

    def start(self):
        if self._thread is not None or mc is None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def snapshot(self):
        with self._lock:
            angles = self._angles.copy() if self._angles else None
            velocities = self._velocities.copy() if self._velocities else None
            ts = self._timestamp
        return angles, velocities, ts

    def _run(self):
        prev_angles = None
        prev_t = None
        while not self._stop.is_set() and _running:
            try:
                t = _now()
                with robot_lock:
                    encoders = safe_list(mc.get_encoders())
                angles = None
                if encoders and len(encoders) >= 6:
                    angles = [round(val * ENCODER_TO_DEG, 4) for val in encoders[:6]]
                else:
                    with robot_lock:
                        angles = safe_list(mc.get_angles())

                vel = None
                if angles is not None and prev_angles is not None and len(angles) == len(prev_angles):
                    dt = t - prev_t if prev_t is not None else self.sample_dt
                    if dt <= 0:
                        dt = self.sample_dt
                    vel = [(curr - prev) / dt for curr, prev in zip(angles, prev_angles)]

                with self._lock:
                    self._angles = angles
                    self._velocities = vel
                    self._timestamp = t

                prev_angles = angles.copy() if angles else None
                prev_t = t
            except Exception as exc:
                print("[WARN encoder_monitor]", exc)
            finally:
                self._stop.wait(self.sample_dt)

def _process_control_command(cmd: dict) -> dict:
    """Execute robot control commands received over TCP."""
    global current_run_id

    if mc is None:
        return {"status": "error", "error": "robot not initialized"}

    ctype = cmd.get("cmd")
    if ctype == "ping":
        return {"status": "ok", "pong": _now()}
    if ctype == "start_run":
        current_run_id = cmd.get("run_id")
        return {"status": "ok", "run_id": current_run_id}
    if ctype == "end_run":
        current_run_id = None
        return {"status": "ok"}

    if ctype == "send_angles":
        angles = cmd.get("angles")
        speed = cmd.get("speed", 50)
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            return {"status": "error", "error": "angles must be length-6 list"}
        try:
            angles = [float(a) for a in angles]
            speed = int(speed)
        except (TypeError, ValueError):
            return {"status": "error", "error": "invalid angle or speed value"}
        with robot_lock:
            mc.send_angles(angles, speed)
        return {"status": "ok"}

    if ctype == "set_gripper":
        state = cmd.get("state")
        speed = cmd.get("speed", 50)
        if state not in (0, 1):
            return {"status": "error", "error": "gripper state must be 0 or 1"}
        try:
            speed = int(speed)
        except (TypeError, ValueError):
            return {"status": "error", "error": "invalid gripper speed"}
        with robot_lock:
            mc.set_gripper_state(state, max(speed, 0), 1)
        return {"status": "ok"}

    if ctype == "stop":
        with robot_lock:
            try:
                mc.stop()
            except AttributeError:
                pass
        return {"status": "ok"}

    if ctype == "drag_clear":
        with robot_lock:
            length = mc.drag_clear_record_data()
        return {"status": "ok", "length": length}

    if ctype == "drag_start":
        with robot_lock:
            length = mc.drag_start_record()
        return {"status": "ok", "length": length}

    if ctype == "drag_stop":
        with robot_lock:
            length = mc.drag_end_record()
        return {"status": "ok", "length": length}

    if ctype == "drag_length":
        with robot_lock:
            length = mc.drag_get_record_len()
        return {"status": "ok", "length": length}

    if ctype == "drag_get_data":
        with robot_lock:
            data = mc.drag_get_record_data()
        if data in (-1, None):
            return {"status": "empty"}
        return {"status": "ok", "data": data, "timestamp": _now()}

    return {"status": "error", "error": f"unknown command '{ctype}'"}

def _handle_control_connection(conn, addr):
    """Handle a single TCP client that streams newline-delimited JSON commands."""
    print(f"[CTRL] connection from {addr}")
    try:
        with conn, conn.makefile("rwb") as fp:
            while _running:
                line = fp.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    cmd = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError as exc:
                    resp = {"status": "error", "error": f"bad json: {exc}"}
                else:
                    resp = _process_control_command(cmd)
                fp.write(json.dumps(resp).encode("utf-8") + b"\n")
                fp.flush()
    except Exception as e:
        print(f"[CTRL] connection error: {e}")
    finally:
        print(f"[CTRL] connection from {addr} closed")

def control_server_loop():
    global control_server
    try:
        control_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        control_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        control_server.bind(("", CONTROL_PORT))
        control_server.listen(1)
        print(f"[CTRL] listening on :{CONTROL_PORT}")
    except Exception as e:
        print(f"[CTRL] failed to start control server: {e}")
        return

    while _running:
        try:
            conn, addr = control_server.accept()
        except OSError:
            break
        threading.Thread(target=_handle_control_connection, args=(conn, addr), daemon=True).start()

def state_loop():
    """Background thread that streams state and derived joint velocities."""
    period = 1.0 / max(1.0, SEND_HZ)
    next_t = time.perf_counter()
    print(f"[STATE] → {RECORDER_IP}:{STATE_PORT} @ {SEND_HZ} Hz")

    last_angles = None
    last_gripper = None
    last_sample_time = None

    while _running:
        t0 = _now()
        try:
            with robot_lock:
                coords  = safe_list(mc.get_coords())     # [x,y,z,rx,ry,rz]
                gripper = mc.get_gripper_value()         # int/float

            angles = None
            joint_vel = None
            if encoder_monitor is not None:
                angles, joint_vel, _ = encoder_monitor.snapshot()

            if angles is None:
                with robot_lock:
                    encoders = safe_list(mc.get_encoders())
                if encoders and len(encoders) >= 6:
                    angles = [round(val * ENCODER_TO_DEG, 3) for val in encoders[:6]]
                else:
                    angles = safe_list(mc.get_angles())

            # Estimate joint velocities (deg/sec) from successive angle readings when monitor missing.
            if joint_vel is None and angles is not None:
                if last_angles is not None and len(angles) == len(last_angles):
                    dt = (t0 - last_sample_time) if last_sample_time is not None else period
                    if dt <= 0:
                        dt = period
                    joint_vel = [(curr - prev) / dt for curr, prev in zip(angles, last_angles)]
                last_angles = angles.copy()
            elif angles is not None:
                last_angles = angles.copy()

            gripper_vel = None
            if gripper is not None and last_gripper is not None:
                dt = (t0 - last_sample_time) if last_sample_time is not None else period
                if dt <= 0:
                    dt = period
                gripper_vel = (gripper - last_gripper) / dt
            last_gripper = gripper

            state_pkt = {
                "type": "state",
                "t": t0,
                "coords": coords,
                "angles": angles,
                "gripper": gripper,
                "joint_velocity": joint_vel,
                "gripper_velocity": gripper_vel,
            }
            send_udp_json(sock_state, RECORDER_IP, STATE_PORT, state_pkt)

            if joint_vel is not None or gripper_vel is not None:
                send_action(
                    joint_pos=angles,
                    joint_vel=joint_vel,
                    gripper_pos=gripper,
                    gripper_vel=gripper_vel,
                )
            last_sample_time = t0

        except Exception as e:
            print("[WARN state]", e)
        next_t += period
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)

# === Your sequence (6 joints) ===
def sequence():
    # pose 1
    angles = [0, 0, 0, 0, 0, -45]
    mc.send_angles(angles, 100)

    # open
    mc.set_gripper_state(0, 80, 1)

    # pose 3
    angles = [-17, -84, -1, -1, 2, 33]
    mc.send_angles(angles, 100)


    # close
    mc.set_gripper_state(1, 50, 1)
    
    # pose 7
    angles = [10,-43, -1, -40,-1, -45]
    mc.send_angles(angles, 100)
    
    # pose 4
    angles = [31, -81, -1, -2, 2, -98]
    mc.send_angles(angles, 100)


    # open
    mc.set_gripper_state(0, 80, 1)

    # pose 5
    angles = [35, -74, -1, -6, -1, -98]
    mc.send_angles(angles, 100)

    # home
    angles = [0, 0, 0, 0, 0, -45]
    mc.send_angles(angles, 100)
    time.sleep(1)

def _shutdown(*_):
    global _running
    _running = False
    print("\n[SHUTDOWN] stopping…")
    try:
        if encoder_monitor is not None:
            encoder_monitor.stop()
    except Exception:
        pass
    try:
        if control_server is not None:
            control_server.close()
    except Exception:
        pass

def main():
    global mc, sock_state, sock_action, encoder_monitor
    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print("[INIT] Opening robot…")
    mc = MyCobot280(ROBOT_PORT, ROBOT_BAUD)
    time.sleep(0.2)

    sock_state  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_action = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    encoder_monitor = EncoderMonitor(sample_hz=ENCODER_SAMPLE_HZ)
    encoder_monitor.start()

    # start threads
    state_thread = threading.Thread(target=state_loop, daemon=True)
    state_thread.start()
    control_thread = threading.Thread(target=control_server_loop, daemon=True)
    control_thread.start()

    try:
        while _running:
            time.sleep(0.5)
    except KeyboardInterrupt:
        _shutdown()
    finally:
        state_thread.join(timeout=2.0)
        try:
            if encoder_monitor is not None:
                encoder_monitor.stop()
        except Exception:
            pass
        try:
            sock_state.close()
        except Exception:
            pass
        try:
            sock_action.close()
        except Exception:
            pass
        print("[EXIT] done.")

if __name__ == "__main__":
    main()
