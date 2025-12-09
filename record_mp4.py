#!/usr/bin/env python3
import os, cv2, json, socket, time, math, random
import threading
from datetime import datetime

# ==== CONFIG ====
STATE_PORT = 5556           # matches pi_state_server.py state stream
ACTION_PORT = 5557          # matches pi_state_server.py action echoq
ROBOT_CONTROL_HOST = "10.75.228.44"  # robot controller IP for control channel
ROBOT_CONTROL_PORT = 5558
USE_ROBOT_CONTROL = True
RUN_SEQUENCE_ON_START = True
USE_SMOOTH_IDLE_MOTION = False #ooth sinusoid idle motion instead of fixed waypoints
FPS = 10
SAVE_ROOT = "dataset"
LANGUAGE_INSTRUCTION = "help me paint a sunset" 

TASK_NAME = "basic_movement"
PREVIEW_HEIGHT = 480        # display height for each view in the mosaic

SMOOTH_IDLE_CONFIG = {
    "base_pose":     [5, -55, -15,  5,  10, -60],
    "amplitude":     [8,  18,  10, 20,  12,  15],
    "frequency":     [0.07, 0.05, 0.09, 0.06, 0.08, 0.11],
    "update_period": 0.35,
    "speed":         65,
    "gripper_period": 9.0,
    "phase_jitter":  0.02,
}

# Camera indexes (check with: ls /dev/video*)
CAMERAS = {
    "exterior_image_1_left": 0,  # Overhead camera
    "wrist_image_left": 2,       # Wrist camera
}

DEFAULT_COMMAND_SEQUENCE = [
    {"command": "send_angles", "angles": [0, -0, 0, 0, 0, -45], "speed": 100, "wait": .2},
    { "command": "sleep", "duration": 1 },

  { "command": "set_gripper", "state": 0, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [30.14, -70.48, -0.87, -8, 3.33, -19.68],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 1, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [50, -50, -1, 12, -2, -45],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 0, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [0, 0, 0, 0, 0, -45],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "send_angles",
    "angles": [37.79, -42.53, -53.08, 8.08, 3.42, 3.86],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 1, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [50, -50, -1, 12, -2, -45],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 0, "speed": 100, "wait": 0.2 },

  { "command": "sleep", "duration": 20 },

  { "command": "set_gripper", "state": 1, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [50, -26.63, -2.37, -17.05, -4.83, -38.23],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "send_angles",
    "angles": [37.35, -36.03, -57.12, 12.39, 0.26, -18.54],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 0, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [0, 0, 0, 0, 0, -45],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "send_angles",
    "angles": [50, -50, -1, 12, -2, -45],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "sleep", "duration": 2 },

  { "command": "set_gripper", "state": 1, "speed": 100, "wait": 0.2 },

  { "command": "send_angles",
    "angles": [50, -26.63, -2.37, -17.05, -4.83, -38.23],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "send_angles",
    "angles": [30.76, -59.15, -23.11, 6.67, -1.05, -12.83],
    "speed": 100,
    "wait": 0.2
  },

  { "command": "set_gripper", "state": 0, "speed": 100, "wait": 0.2 }
]
# =================
SEQUENCE_START_DELAY_SEC = 0.5

class RobotControlClient:
    def __init__(self, host, port, *, enabled: bool = True):
        self.host = host
        self.port = port
        self.enabled = enabled
        self._sock = None
        self._stream = None

    @property
    def connected(self) -> bool:
        return self.enabled and self._sock is not None and self._stream is not None

    def connect(self) -> bool:
        if not self.enabled:
            return False/home/ilovejafari/openpi/scripts/checkpoints/pi0_droid_low_mem_finetune/mycobot_run/99
        if self.connected:
            return True
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=5)
            self._stream = self._sock.makefile("rwb")
            print(f"[CTRL] Connected to robot control at {self.host}:{self.port}")
            return True
        except OSError as exc:
            print(f"[CTRL] Unable to connect to robot control ({exc}); disabling control channel.")
            self.close()
            self.enabled = False
            return False

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None
        self._stream = None

    def _send_command(self, payload: dict) -> dict | None:
        if not self.connected:
            return None
        try:
            encoded = json.dumps(payload).encode("utf-8") + b"\n"
            self._stream.write(encoded)
            self._stream.flush()
            line = self._stream.readline()
            if not line:
                raise ConnectionError("control server closed connection")
            return json.loads(line.decode("utf-8"))
        except Exception as exc:
            print(f"[CTRL] command failed: {exc}")
            self.close()
            return None

    def start_run(self, run_id: str | None = None) -> None:
        if run_id is None:
            run_id = datetime.now().isoformat()
        resp = self._send_command({"cmd": "start_run", "run_id": run_id})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] start_run error: {resp}")

    def end_run(self) -> None:
        resp = self._send_command({"cmd": "end_run"})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] end_run error: {resp}")

    def send_angles(self, angles, speed: int = 50) -> None:
        resp = self._send_command({"cmd": "send_angles", "angles": angles, "speed": speed})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] send_angles error: {resp}")

    def set_gripper(self, state: int, speed: int = 50) -> None:
        resp = self._send_command({"cmd": "set_gripper", "state": state, "speed": speed})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] set_gripper error: {resp}")

    def drag_clear(self) -> None:
        resp = self._send_command({"cmd": "drag_clear"})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] drag_clear error: {resp}")

    def drag_start_record(self) -> None:
        resp = self._send_command({"cmd": "drag_start"})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] drag_start error: {resp}")

    def drag_stop_record(self) -> None:
        resp = self._send_command({"cmd": "drag_stop"})
        if resp and resp.get("status") != "ok":
            print(f"[CTRL] drag_stop error: {resp}")

    def fetch_drag_samples(self, max_batch: int = 50):
        """Pull available drag-record samples from the robot."""
        samples = []
        if not self.connected:
            return samples
        for _ in range(max_batch):
            resp = self._send_command({"cmd": "drag_get_data"})
            if not resp:
                break
            status = resp.get("status")
            if status == "empty":
                break
            if status != "ok":
                print(f"[CTRL] drag_get_data error: {resp}")
                break
            samples.append({"data": resp.get("data"), "timestamp": resp.get("timestamp")})
        return samples


def _convert_drag_values(values):
    if not isinstance(values, (list, tuple)):
        return None
    converted = []
    for val in values:
        if val is None:
            converted.append(None)
            continue
        if isinstance(val, (int, float)):
            abs_val = abs(val)
            if abs_val > 3600:
                converted.append(round(val / 4096.0 * 360.0, 6))
            elif abs_val > 720:
                converted.append(round(val / 100.0, 6))
            else:
                converted.append(float(val))
        else:
            converted.append(None)
    return converted


def _decode_drag_sample(payload, index: int):
    if not isinstance(payload, dict):
        return None
    raw = payload.get("data")
    timestamp = payload.get("timestamp")
    if raw in (None, -1):
        return None
    encoders = None
    speeds = None
    if isinstance(raw, list):
        if raw and isinstance(raw[0], list) and len(raw) >= 2:
            encoders = raw[0]
            speeds = raw[1]
        elif len(raw) == 2 and all(isinstance(item, (list, tuple)) for item in raw):
            encoders = list(raw[0])
            speeds = list(raw[1])
        elif len(raw) == 6:
            encoders = raw
        else:
            # Attempt to split even-sized list into two groups.
            mid = len(raw) // 2
            encoders = raw[:mid]
            speeds = raw[mid:]
    converted_angles = _convert_drag_values(encoders) if encoders is not None else None
    converted_vel = _convert_drag_values(speeds) if speeds is not None else None
    return {
        "index": index,
        "timestamp": timestamp,
        "joint_position": converted_angles,
        "joint_velocity": converted_vel,
        "raw": raw,
    }


class DragSampleFetcher:
    def __init__(self, client: RobotControlClient, *, poll_interval: float = 0.05, max_batch: int = 10):
        self.client = client
        self.poll_interval = poll_interval
        self.max_batch = max_batch
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._samples: list[dict] = []
        self._latest: dict | None = None

    @property
    def samples(self) -> list[dict]:
        with self._lock:
            return list(self._samples)

    def latest_sample(self) -> dict | None:
        with self._lock:
            return self._latest.copy() if self._latest else None

    def start(self) -> None:
        if not self.client.connected or self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            if not self.client.connected:
                break
            payloads = self.client.fetch_drag_samples(max_batch=self.max_batch)
            if payloads:
                with self._lock:
                    for payload in payloads:
                        idx = len(self._samples)
                        decoded = _decode_drag_sample(payload, idx)
                        if decoded:
                            self._samples.append(decoded)
                            self._latest = decoded
                continue
            time.sleep(self.poll_interval)


def _to_float(value):
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _coerce_vector(values):
    if not isinstance(values, (list, tuple)):
        return None
    coerced = []
    for val in values:
        if isinstance(val, (int, float)):
            coerced.append(float(val))
        elif val is None:
            coerced.append(None)
        else:
            try:
                coerced.append(float(val))
            except (TypeError, ValueError):
                coerced.append(None)
    return coerced


class SmoothIdleMotion:
    """Generate continuous idle motion using sinusoidal joints and slow gripper waves."""

    def __init__(self, client: RobotControlClient, config: dict):
        self.client = client
        self.update_period = max(0.15, float(config.get("update_period", 0.35)))
        self.speed = int(config.get("speed", 60))
        self.gripper_period = float(config.get("gripper_period", 0))
        self.phase_jitter = float(config.get("phase_jitter", 0))
        self._base = self._normalize(config.get("base_pose"), fallback=[0, -50, -10, 0, 0, -45])
        self._amp = self._normalize(config.get("amplitude"), fallback=[10, 10, 10, 10, 10, 10])
        self._freq = self._normalize(config.get("frequency"), fallback=[0.08] * 6)
        self._phases = [random.uniform(0, 2 * math.pi) for _ in range(6)]
        self._thread = None
        self._stop = threading.Event()
        self._rng = random.Random()
        self._gripper_state = None

    def _normalize(self, values, fallback):
        base = list(fallback)
        if isinstance(values, (list, tuple)):
            for idx in range(min(6, len(values))):
                try:
                    base[idx] = float(values[idx])
                except (TypeError, ValueError):
                    continue
        return base

    def start(self):
        if not self.client.connected or self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self):
        start_ts = time.perf_counter()
        while not self._stop.is_set():
            t = time.perf_counter() - start_ts
            pose = []
            for i in range(6):
                base = self._base[i]
                amp = self._amp[i]
                freq = self._freq[i]
                phase = self._phases[i]
                pose.append(base + amp * math.sin(2 * math.pi * freq * t + phase))
            self.client.send_angles([round(p, 3) for p in pose], self.speed)

            if self.gripper_period > 0:
                cycle = math.sin(2 * math.pi * t / self.gripper_period)
                next_state = 1 if cycle > 0 else 0
                if next_state != self._gripper_state:
                    self.client.set_gripper(next_state, 60)
                    self._gripper_state = next_state

            if self.phase_jitter > 0:
                self._phases = [
                    (phase + self._rng.uniform(-self.phase_jitter, self.phase_jitter)) % (2 * math.pi)
                    for phase in self._phases
                ]

            if self._stop.wait(self.update_period):
                break


def run_robot_sequence(client: RobotControlClient, sequence) -> None:
    if client is None or not client.connected:
        return
    for step in sequence:
        cmd = step.get("command")
        wait_s = float(step.get("wait", 0.0) or 0.0)
        if cmd == "send_angles":
            client.send_angles(step.get("angles", []), step.get("speed", 50))
        elif cmd == "set_gripper":
            client.set_gripper(step.get("state", 0), step.get("speed", 50))
        elif cmd == "sleep":
            wait_s += float(step.get("duration", 0.0) or 0.0)
        else:
            print(f"[CTRL] Unknown sequence command: {cmd}")
        if wait_s > 0:
            time.sleep(wait_s)

# --- setup folders ---
os.makedirs(SAVE_ROOT, exist_ok=True)
ep_name = datetime.now().strftime("episode_%Y-%m-%d_%H-%M-%S")
ep_dir = os.path.join(SAVE_ROOT, ep_name)
video_dir = os.path.join(ep_dir, "recordings", "MP4")
step_dir = os.path.join(ep_dir, "steps")
os.makedirs(video_dir, exist_ok=True)
os.makedirs(step_dir, exist_ok=True)

# --- UDP listeners ---
latest_state = None
latest_action = None

sock_state = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_state.bind(("", STATE_PORT))     # listen on all interfaces, port 5556
sock_state.setblocking(False)

sock_action = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_action.bind(("", ACTION_PORT))   # listen on all interfaces, port 5557
sock_action.setblocking(False)

# === Receivers ===
def recv_state():
    """Non-blocking receive for state messages on UDP :5556."""
    global latest_state
    try:
        data, _ = sock_state.recvfrom(4096)  # buffer size in bytes
        msg = json.loads(data.decode())
        # Accept either explicit state type or no type (your Pi code may omit it)
        if msg.get("type") == "state" or "type" not in msg:
            latest_state = msg
    except BlockingIOError:
        pass
    except Exception as e:
        print("[WARN recv_state]", e)

def recv_action():
    """Non-blocking receive for action messages on UDP :5557."""
    global latest_action
    try:
        data, _ = sock_action.recvfrom(4096)
        msg = json.loads(data.decode())
        # Expect type="action" (your Pi echoes this); accept missing type if needed
        if msg.get("type") == "action" or "type" not in msg:
            latest_action = msg
    except BlockingIOError:
        pass
    except Exception as e:
        print("[WARN recv_action]", e)

# --- camera setup ---
caps = {}
video_writers = {}
for name, idx in CAMERAS.items():
    cam = cv2.VideoCapture(idx)
    cam.set(cv2.CAP_PROP_FPS, FPS)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # optional
    if not cam.isOpened():
        print(f"[ERROR] Could not open camera {idx} ({name})")
    caps[name] = cam
    width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
    height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_path = os.path.join(video_dir, f"{name}.mp4")
    writer = cv2.VideoWriter(video_path, fourcc, FPS if FPS > 0 else 10.0, (width, height))
    if not writer.isOpened():
        print(f"[ERROR] Could not open video writer for {video_path}")
    video_writers[name] = writer

# --- episode metadata ---
start_time = time.time()

# DROID-style metadata file describing cameras
metadata_payload = {
    "episode_id": ep_name,
    "language_instruction": LANGUAGE_INSTRUCTION,
    "timestamp": datetime.fromtimestamp(start_time).isoformat(),
    "camera_serial_numbers": {name: f"camera_{idx}" for name, idx in CAMERAS.items()},
    "camera_type": {  # mimic DROID mapping: wrist=0 (hand), exterior=1 (varied)
        name: (0 if "wrist" in name else 1)
        for name in CAMERAS
    },
    "recordings": {
        "mp4": {name: os.path.join("recordings", "MP4", f"{name}.mp4") for name in CAMERAS},
    },
    "fps": FPS,
    "start_time": start_time,
    "end_time": None,
}
metadata_name = f"metadata_{ep_name}.json"
metadata_path = os.path.join(ep_dir, metadata_name)
with open(metadata_path, "w") as f:
    json.dump(metadata_payload, f, indent=2)

control_client = RobotControlClient(
    ROBOT_CONTROL_HOST,
    ROBOT_CONTROL_PORT,
    enabled=USE_ROBOT_CONTROL,
)
drag_enabled = False
drag_fetcher = None
if control_client.enabled and control_client.connect():
    drag_enabled = True
    control_client.drag_clear()
    control_client.drag_start_record()
    control_client.start_run(ep_name)
    drag_fetcher = DragSampleFetcher(control_client)
    drag_fetcher.start()
else:
    drag_enabled = False
    if USE_ROBOT_CONTROL:
        print("[CTRL] Control channel disabled; continuing without scripted robot sequence.")

print(f"[Recorder] Recording {ep_name}")
print("Preview shows OVERHEAD | WRIST side-by-side. Press 'q' (window) or Ctrl+C (terminal) to stop.")

# Create preview window up front
cv2.namedWindow("Recording", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Recording", 1280, 720)

sequence_thread = None
idle_motion = None
if control_client.connected and RUN_SEQUENCE_ON_START:
    if USE_SMOOTH_IDLE_MOTION:
        idle_motion = SmoothIdleMotion(control_client, SMOOTH_IDLE_CONFIG)
        idle_motion.start()
    elif DEFAULT_COMMAND_SEQUENCE:
        def _sequence_worker():
            delay = max(0.0, SEQUENCE_START_DELAY_SEC)
            if delay > 0:
                time.sleep(delay)
            print("[CTRL] Running default robot sequence...")
            run_robot_sequence(control_client, DEFAULT_COMMAND_SEQUENCE)

        sequence_thread = threading.Thread(target=_sequence_worker, daemon=True)
        sequence_thread.start()

interval = 1.0 / FPS
last = 0.0
step_idx = 0
prev_joint_pos = None
prev_gripper = None
prev_state_time = None
drag_samples_all = []
latest_drag_sample = None

def resize_to_height(img, height):
    h, w = img.shape[:2]
    if h == 0: return img
    scale = height / float(h)
    return cv2.resize(img, (int(w * scale), height))

try:
    while True:
        now = time.time()
        if now - last < interval:
            recv_state()
            recv_action()
            # Always poll keys, even when skipping a frame
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'): break
            if cv2.getWindowProperty("Recording", cv2.WND_PROP_VISIBLE) < 1: break
            continue

        last = now
        recv_state()
        recv_action()

        if drag_fetcher:
            sample = drag_fetcher.latest_sample()
            if sample is not None:
                latest_drag_sample = sample

        obs = {}
        raw_frames = {}

        # capture both cameras
        for name, cam in caps.items():
            ret, frame = cam.read()
            if not ret:
                continue
            writer = video_writers.get(name)
            if writer is not None:
                writer.write(frame)
            rel_video_path = os.path.join("recordings", "MP4", f"{name}.mp4")
            obs[name] = {
                "video_path": rel_video_path,
                "frame_index": step_idx,
            }
            raw_frames[name] = frame  # keep for preview

        if not obs:
            # poll keys and continue if no frames this tick
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'): break
            if cv2.getWindowProperty("Recording", cv2.WND_PROP_VISIBLE) < 1: break
            continue

        # build DROID-style step
        state = dict(latest_state) if isinstance(latest_state, dict) else {}
        action = dict(latest_action) if isinstance(latest_action, dict) else {}
        drag_sample_for_step = latest_drag_sample
        drag_positions = _coerce_vector(drag_sample_for_step.get("joint_position")) if drag_sample_for_step else None
        drag_velocities = _coerce_vector(drag_sample_for_step.get("joint_velocity")) if drag_sample_for_step else None
        drag_sample_timestamp = drag_sample_for_step.get("timestamp") if drag_sample_for_step else None

        state_ts = _to_float(state.get("t"))
        if state_ts is None:
            state_ts = now

        state_joint_angles = _coerce_vector(state.get("angles"))
        action_joint_angles = _coerce_vector(action.get("joint_position"))
        joint_angles_list = drag_positions or state_joint_angles or action_joint_angles
        if joint_angles_list is not None:
            state["angles"] = joint_angles_list
            action["joint_position"] = joint_angles_list

        gripper_scalar = _to_float(state.get("gripper")) or _to_float(action.get("gripper_position"))
        if gripper_scalar is not None:
            state["gripper"] = gripper_scalar
            action["gripper_position"] = gripper_scalar

        dt_state = interval
        if prev_state_time is not None:
            delta = state_ts - prev_state_time
            if isinstance(delta, (int, float)) and delta > 0:
                dt_state = delta

        computed_joint_vel = drag_velocities or _coerce_vector(state.get("joint_velocity")) or _coerce_vector(action.get("joint_velocity"))
        computed_gripper_vel = _to_float(state.get("gripper_velocity")) or _to_float(action.get("gripper_velocity"))

        if computed_joint_vel is None and joint_angles_list is not None and prev_joint_pos is not None and len(joint_angles_list) == len(prev_joint_pos):
            computed_joint_vel = [(curr - prev) / dt_state for curr, prev in zip(joint_angles_list, prev_joint_pos)]

        if computed_gripper_vel is None and gripper_scalar is not None and prev_gripper is not None:
            computed_gripper_vel = (gripper_scalar - prev_gripper) / dt_state

        if joint_angles_list is not None:
            prev_joint_pos = joint_angles_list.copy()
        if gripper_scalar is not None:
            prev_gripper = gripper_scalar
        prev_state_time = state_ts

        observation_joint_position = joint_angles_list if joint_angles_list is not None else state.get("angles")
        observation_gripper_position = gripper_scalar if gripper_scalar is not None else state.get("gripper")
        action_joint_position = joint_angles_list if joint_angles_list is not None else action.get("joint_position")
        action_gripper_position = gripper_scalar if gripper_scalar is not None else action.get("gripper_position")
        action_joint_velocity = computed_joint_vel if computed_joint_vel is not None else action.get("joint_velocity")
        action_gripper_velocity = computed_gripper_vel if computed_gripper_vel is not None else action.get("gripper_velocity")

        synthetic_timestamp = step_idx * interval

        step_data = {
            "step_index": step_idx,
            "timestamp": synthetic_timestamp,
            "drag_sample_index": (drag_sample_for_step["index"] if drag_sample_for_step else None),
            "drag_sample_timestamp": drag_sample_timestamp,

            "episode_metadata": {
                "recording_folderpath": ep_dir,
                "file_path": None,
            },

            "is_first": (step_idx == 0),
            "is_last": False,
            "is_terminal": False,

            # Put your instruction explicitly (was incorrectly pulled from obs before)
            "language_instruction": LANGUAGE_INSTRUCTION,

            "observation": {
                "gripper_position": observation_gripper_position,
                "cartesian_position": state.get("coords"),
                "joint_position": observation_joint_position,
                "wrist_image_left": obs.get("wrist_image_left"),
                "exterior_image_1_left": obs.get("exterior_image_1_left"),
                "exterior_image_2_left": obs.get("exterior_image_2_left"),
            },

            "action_dict": {
                "gripper_position":   action_gripper_position,
                "gripper_velocity":   action_gripper_velocity,
                "cartesian_position": action.get("cartesian_position"),
                "cartesian_velocity": action.get("cartesian_velocity"),
                "joint_position":     action_joint_position,
                "joint_velocity":     action_joint_velocity,
            },

            "discount": 1,     # <-- fixed key
            "reward": None,
            "action": None,    # (optional) low-level action vector if you want to store it
        }

        step_path = os.path.join(step_dir, f"step_{step_idx:06d}.json")
        with open(step_path, "w") as f:
            json.dump(step_data, f)

        # ---- PREVIEW: show OVERHEAD and WRIST side-by-side if available ----
        views = []
        coords_text = f"xyz={(state.get('coords', 'N/A'))}"
        for key, label in [("exterior_image_1_left", "OVERHEAD"), ("wrist_image_left", "WRIST")]:
            if key in raw_frames:
                img = raw_frames[key].copy()
                cv2.putText(img, f"{label}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.putText(img, coords_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                img = resize_to_height(img, PREVIEW_HEIGHT)
                views.append(img)

        if len(views) == 1:
            preview = views[0]
        elif len(views) >= 2:
            preview = cv2.hconcat(views[:2])
        else:
            preview = None

        if preview is not None:
            cv2.imshow("Recording", preview)

        step_idx += 1

        # ALWAYS poll keys once per loop
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        if cv2.getWindowProperty("Recording", cv2.WND_PROP_VISIBLE) < 1: break

except KeyboardInterrupt:
    pass
finally:
    if idle_motion:
        idle_motion.stop()

    if control_client.connected and drag_enabled:
        control_client.drag_stop_record()

    if drag_fetcher:
        drag_fetcher.stop()
        drag_samples_all = drag_fetcher.samples
    elif control_client.connected and drag_enabled:
        final_payloads = control_client.fetch_drag_samples(max_batch=500)
        if final_payloads:
            for payload in final_payloads:
                sample_index = len(drag_samples_all)
                decoded = _decode_drag_sample(payload, sample_index)
                if decoded:
                    drag_samples_all.append(decoded)
                    latest_drag_sample = decoded

    if control_client.connected:
        control_client.end_run()
    control_client.close()
    if sequence_thread is not None and sequence_thread.is_alive():
        sequence_thread.join(timeout=1.0)

    # mark terminal if we wrote at least one step
    if step_idx > 0:
        last_step = os.path.join(step_dir, f"step_{step_idx-1:06d}.json")
        try:
            with open(last_step, "r") as f:
                s = json.load(f)
            s["is_terminal"] = True
            with open(last_step, "w") as f:
                json.dump(s, f)
        except Exception as e:
            print("[WARN] Could not mark terminal:", e)

    # append end_time to metadata
    try:
        with open(metadata_path, "r") as f:
            m = json.load(f)
        m["end_time"] = time.time()
        with open(metadata_path, "w") as f:
            json.dump(m, f, indent=2)
    except Exception as e:
        print("[WARN] Could not update metadata:", e)

    # cleanup
    for cam in caps.values():
        try: cam.release()
        except Exception: pass
    try: sock_state.close()
    except Exception: pass
    try: sock_action.close()
    except Exception: pass
    for writer in video_writers.values():
        try: writer.release()
        except Exception: pass
    cv2.destroyAllWindows()

drag_record_path = None
if drag_samples_all:
    drag_record_path = os.path.join(ep_dir, "drag_recording.json")
    with open(drag_record_path, "w") as f:
        json.dump(drag_samples_all, f, indent=2)

print(f"[Recorder] Saved episode at {ep_dir}")
if drag_record_path:
    print(f"[Recorder] Saved {len(drag_samples_all)} drag samples to {drag_record_path}")
