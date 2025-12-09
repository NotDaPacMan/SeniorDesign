#!/usr/bin/env python3
"""MyCobot execution loop that consumes DROID-style policy actions.

This script mirrors the original ``main.py`` rollout flow, but adapts the
inputs/outputs so they match the MyCobot fine-tuning pipeline:

* Connects to a policy server launched via ``serve_policy.py``.
* Streams the exterior-left and wrist camera feeds, matching the LeRobot /
  DROID observation layout used during training.
* Converts predicted DROID-format action chunks (joint velocity + gripper)
  into MyCobot position commands, mirroring the inverse of ``record_mp4.py``.
  
  
  source ~/openpi-env/bin/activate
  
python main_mycobot.py --policy-host 172.16.51.114 --policy-port 8000 --robot-port /dev/ttyAMA0 --exterior-camera-index 0 --wrist-camera-index 2 --wrist-fourcc MJPG --wrist-width 640 --wrist-height 480 --camera-fps 30

change camera indeces as needed
"""

from __future__ import annotations

import contextlib
import dataclasses
import datetime
import os
import signal
import sys
import time
from collections.abc import Iterator
from pathlib import Path

import cv2
import numpy as np
import tyro

try:
    from pymycobot import MyCobot280
except ImportError as exc:  # pragma: no cover - hardware import
    raise SystemExit(
        "pymycobot must be installed on the robot controller. "
        "Try `pip install pymycobot` (or use the same conda env as training)."
    ) from exc


DROID_CONTROL_FREQUENCY = 15.0  # Hz


def _ensure_openpi_client() -> None:
    """Ensure ``openpi_client`` can be imported, even if only the repo is synced."""
    try:
        import openpi_client  # type: ignore  # noqa: F401
        return
    except ImportError:
        pass

    repo_root = Path(__file__).resolve().parents[1]
    script_dir = Path(__file__).resolve().parent
    env_hint = os.environ.get("OPENPI_CLIENT_PATH")
    candidates = [
        script_dir / "openpi-client" / "src",
        script_dir / "openpi_client",
        repo_root / "packages" / "openpi-client" / "src",
    ]
    if env_hint:
        candidates.insert(0, Path(env_hint).expanduser())

    search_paths: list[Path] = []
    for root in candidates:
        root = root.expanduser().resolve()
        search_paths.append(root)
        search_paths.append(root / "openpi_client")

    for candidate in search_paths:
        if candidate.is_dir():
            path_str = str(candidate)
            if path_str not in sys.path:
                sys.path.insert(0, path_str)

    try:
        import openpi_client  # type: ignore  # noqa: F401
    except ImportError as exc:  # pragma: no cover - configuration error
        raise SystemExit(
            "Unable to import openpi_client. Install it with "
            "`pip install ./packages/openpi-client` (repo layout) or set OPENPI_CLIENT_PATH "
            "to the directory containing the package."
        ) from exc


_ensure_openpi_client()

from openpi_client import image_tools  # noqa: E402  (import after path fix)
from openpi_client.websocket_client_policy import WebsocketClientPolicy  # noqa: E402


@dataclasses.dataclass
class Args:
    # Policy server connection.
    policy_host: str = "192.168.0.10"
    policy_port: int = 8000

    # Robot serial connection.
    robot_port: str = "/dev/ttyAMA0"
    robot_baud: int = 1_000_000

    # Camera indices (adjust to match `ls /dev/video*` on the controller).
    exterior_camera_index: int = 0
    wrist_camera_index: int = 2
    exterior_fourcc: str | None = None
    wrist_fourcc: str | None = "YUY2"
    exterior_width: int = 1280
    exterior_height: int = 720
    wrist_width: int = 1280
    wrist_height: int = 720
    camera_fps: float = 10.0

    # Rollout / control parameters.
    control_hz: float = DROID_CONTROL_FREQUENCY
    max_timesteps: int = 600
    open_loop_horizon: int = 10

    # Action scaling (convert normalized velocities into joint targets).
    joint_velocity_scale_deg_per_s: float = 30.0
    max_joint_step_deg: float = 4.0
    gripper_close_value: int = 100
    gripper_open_value: int = 0
    gripper_threshold: float = 0.5
    dry_run: bool = False

    # Prompt handling.
    prompt: str | None = None
    prompt_2: str | None = None
    prompt_3: str | None = None


@contextlib.contextmanager
def prevent_keyboard_interrupt() -> Iterator[None]:
    """Delay Ctrl+C until the protected block completes (matches main.py behaviour)."""
    interrupted = False
    original_handler = signal.getsignal(signal.SIGINT)

    def handler(signum, frame):  # noqa: ARG001
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, handler)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, original_handler)
        if interrupted:
            raise KeyboardInterrupt


def _open_camera(index: int, label: str, *, fourcc: str | None, width: int, height: int, fps: float) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise SystemExit(f"Failed to open {label} camera at index {index}.")
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def _read_rgb_frame(cap: cv2.VideoCapture, label: str) -> np.ndarray:
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f"Failed to capture frame from {label} camera.")
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def _normalize_joint_reading(values: object) -> np.ndarray | None:
    if values is None:
        return None
    try:
        arr = np.asarray(values, dtype=np.float32)
    except (TypeError, ValueError):
        return None
    if arr.ndim == 0 or arr.size < 6:
        return None
    return arr.reshape(-1)[:6]


def _get_joint_positions(robot: MyCobot280, *, retries: int = 5, delay_s: float = 0.05) -> np.ndarray:
    last_exc: Exception | None = None
    for _ in range(max(1, retries)):
        try:
            joint_pos = _normalize_joint_reading(robot.get_angles())
        except Exception as exc:  # pragma: no cover - hardware call
            last_exc = exc
            joint_pos = None
        if joint_pos is not None:
            return joint_pos
        if delay_s > 0:
            time.sleep(delay_s)
    msg = "Failed to read joint angles from MyCobot. Is the robot connected and out of emergency stop?"
    if last_exc:
        raise RuntimeError(msg) from last_exc
    raise RuntimeError(msg)


def _get_gripper_position(robot: MyCobot280, *, retries: int = 5, delay_s: float = 0.05) -> float:
    last_exc: Exception | None = None
    for _ in range(max(1, retries)):
        try:
            value = robot.get_gripper_value()
        except Exception as exc:  # pragma: no cover - hardware call
            last_exc = exc
            value = None
        if value is not None:
            try:
                return float(value)
            except (TypeError, ValueError) as exc:
                last_exc = exc
        if delay_s > 0:
            time.sleep(delay_s)
    msg = "Failed to read gripper value from MyCobot."
    if last_exc:
        raise RuntimeError(msg) from last_exc
    raise RuntimeError(msg)


def _prepare_observation(
    exterior_cap: cv2.VideoCapture,
    wrist_cap: cv2.VideoCapture,
    robot: MyCobot280,
) -> dict[str, np.ndarray]:
    exterior_rgb = _read_rgb_frame(exterior_cap, "exterior")
    wrist_rgb = _read_rgb_frame(wrist_cap, "wrist")

    resized_exterior = image_tools.resize_with_pad(exterior_rgb, 224, 224)
    resized_wrist = image_tools.resize_with_pad(wrist_rgb, 224, 224)

    joint_pos = _get_joint_positions(robot)
    gripper_pos = np.asarray([_get_gripper_position(robot)], dtype=np.float32)

    return {
        "observation/exterior_image_1_left": resized_exterior,
        "observation/wrist_image_left": resized_wrist,
        "observation/joint_position": joint_pos,
        "observation/gripper_position": gripper_pos,
    }


def _format_action(action: np.ndarray) -> tuple[np.ndarray, float]:
    """Map DROID action vector -> (joint velocities, gripper signal)."""
    if action.ndim != 1:
        action = np.asarray(action).reshape(-1)
    if action.size < 6:
        raise ValueError(f"Expected at least 6 action dims, but received shape {action.shape}.")

    joint_vel = action[:6]
    gripper_signal = float(action[6]) if action.size > 6 else 0.0
    return joint_vel, gripper_signal


def _apply_action(
    robot: MyCobot280,
    joint_vel: np.ndarray,
    gripper_signal: float,
    *,
    dt: float,
    joint_velocity_scale_deg_per_s: float,
    max_joint_step_deg: float,
    gripper_close_value: int,
    gripper_open_value: int,
    gripper_threshold: float,
    dry_run: bool,
) -> None:
    joint_vel = np.clip(joint_vel, -1.0, 1.0) * joint_velocity_scale_deg_per_s

    current_angles = _get_joint_positions(robot)
    target_angles = current_angles + joint_vel * dt
    delta = np.clip(target_angles - current_angles, -max_joint_step_deg, max_joint_step_deg)
    target_angles = current_angles + delta

    gripper_cmd = (
        gripper_close_value if gripper_signal >= gripper_threshold else gripper_open_value
    )

    print(
        f"[ACT] joint_vel={joint_vel.round(2)} target={target_angles.round(2)} "
        f"gripper_signal={gripper_signal:.3f} gripper_cmd={gripper_cmd}",
        flush=True,
    )

    if dry_run:
        return

    robot.send_angles(target_angles.tolist(), speed=50)
    robot.set_gripper_state(1 if gripper_cmd else 0, abs(gripper_cmd), 1)


def main(args: Args) -> None:
    print(f"[INFO] Connecting to MyCobot on {args.robot_port} @ {args.robot_baud} baud.")
    robot = MyCobot280(args.robot_port, args.robot_baud)
    time.sleep(0.5)

    print(f"[INFO] Connecting to policy server at {args.policy_host}:{args.policy_port}.")
    policy = WebsocketClientPolicy(args.policy_host, args.policy_port)
    print(f"[INFO] Policy metadata: {policy.get_server_metadata()}")

    print("[INFO] Opening cameras.")
    exterior_cap = _open_camera(
        args.exterior_camera_index,
        "exterior",
        fourcc=args.exterior_fourcc,
        width=args.exterior_width,
        height=args.exterior_height,
        fps=args.camera_fps,
    )
    wrist_cap = _open_camera(
        args.wrist_camera_index,
        "wrist",
        fourcc=args.wrist_fourcc,
        width=args.wrist_width,
        height=args.wrist_height,
        fps=args.camera_fps,
    )

    dt = 1.0 / args.control_hz

    try:
        prompt_schedule = [p.strip() for p in (args.prompt, args.prompt_2, args.prompt_3) if p and p.strip()]
        scheduled_mode = bool(prompt_schedule)
        episode_idx = 0
        while True:
            if not scheduled_mode:
                instruction = input("Enter instruction (leave empty to exit): ").strip()
                if instruction == "":
                    print("[INFO] Exiting at user request.")
                    break
            else:
                if episode_idx >= len(prompt_schedule):
                    break
                instruction = prompt_schedule[episode_idx]

            print("[INFO] Starting rollout... Press Ctrl+C to stop.")
            actions_from_chunk = 0
            pred_action_chunk = None
            step = 0
            rollout_start = datetime.datetime.now().isoformat()

            try:
                for step in range(args.max_timesteps):
                    loop_start = time.perf_counter()
                    observation = _prepare_observation(exterior_cap, wrist_cap, robot)
                    observation["prompt"] = instruction

                    if pred_action_chunk is None or actions_from_chunk >= args.open_loop_horizon:
                        actions_from_chunk = 0
                        with prevent_keyboard_interrupt():
                            response = policy.infer(observation)
                        pred_action_chunk = np.asarray(response["actions"], dtype=np.float32)
                        if pred_action_chunk.ndim != 2:
                            raise ValueError(
                                f"Expected action chunk of shape (horizon, dim), got {pred_action_chunk.shape}."
                            )

                    action = pred_action_chunk[actions_from_chunk]
                    actions_from_chunk += 1

                    joint_vel, gripper_signal = _format_action(action)
                    _apply_action(
                        robot,
                        joint_vel,
                        gripper_signal,
                        dt=dt,
                        joint_velocity_scale_deg_per_s=args.joint_velocity_scale_deg_per_s,
                        max_joint_step_deg=args.max_joint_step_deg,
                        gripper_close_value=args.gripper_close_value,
                        gripper_open_value=args.gripper_open_value,
                        gripper_threshold=args.gripper_threshold,
                        dry_run=args.dry_run,
                    )

                    elapsed = time.perf_counter() - loop_start
                    remaining = dt - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
            except KeyboardInterrupt:
                print("\n[INFO] Rollout interrupted by user.")

            print(f"[INFO] Completed {step + 1} steps (started at {rollout_start}).")
            episode_idx += 1
            if scheduled_mode and episode_idx >= len(prompt_schedule):
                break
    finally:
        exterior_cap.release()
        wrist_cap.release()
        cv2.destroyAllWindows()
        print("[INFO] Cameras released and OpenCV windows closed.")


if __name__ == "__main__":
    main(tyro.cli(Args))
