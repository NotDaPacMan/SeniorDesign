#!/usr/bin/env python3
"""
Convert MyCobot episodes (DROID-ish JSON + MP4 videos) to LeRobot format.

Folder layout per episode:
  episode_.../
    recordings/
      MP4/
        exterior_image_1_left.mp4
        wrist_image_left.mp4
        ...
    steps/
      step_000001.json
      step_000002.json
      ...

Run:
export HF_LEROBOT_HOME=/home/ilovejafari/openpi/scripts/dataset
python Lerobot_convert_mp4.py 
    --data_dir /home/ilovejafari/openpi/scripts/dataset/basic_movement/move_right \
    --repo_id local/mycobot_dataset --overwrite

note: just change data_dir based on what folder you want to convert
"""

import json
import shutil
from pathlib import Path
from typing import List, Tuple

import numpy as np
import tyro
import cv2

from lerobot.common.datasets.lerobot_dataset import HF_LEROBOT_HOME
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

#updated package trying: 
#from lerobot.constants import HF_LEROBOT_HOME
#from lerobot.datasets.lerobot_dataset import LeRobotDataset

'''
try:
except Exception:
    HF_LEROBOT_HOME = Path.home() / ".cache" / "huggingface" / "lerobot"
'''


def _normalize_instruction(value, *, fallback: str = "") -> str:
    if isinstance(value, str):
        text = value.strip()
        return text if text else fallback
    if value is None:
        return fallback
    text = str(value).strip()
    return text if text else fallback

def _sorted_step_files(steps_dir: Path) -> List[Path]:
    def key(p: Path):
        s = p.stem
        digits = "".join(ch for ch in s if ch.isdigit())
        return int(digits) if digits else s
    return sorted(steps_dir.glob("step_*.json"), key=key)


def _find_video_readers(video_dir: Path) -> dict[str, "cv2.VideoCapture"]:
    import cv2

    readers = {}
    if not video_dir.exists():
        return readers
    for mp4_path in video_dir.glob("*.mp4"):
        reader = cv2.VideoCapture(str(mp4_path))
        if reader.isOpened():
            readers[mp4_path.stem] = reader
    return readers


def _estimate_fps(tstamps: List[float]) -> int:
    if len(tstamps) < 3:
        return 10
    dts = np.diff(sorted(tstamps))
    dts = dts[np.isfinite(dts) & (dts > 0)]
    if len(dts) == 0:
        return 10
    med = float(np.median(dts))
    fps = int(round(1.0 / med)) if med > 0 else 10
    return max(1, min(fps, 60))


def _find_episodes(data_dir: Path) -> List[Path]:
    """
    Discover episode directories by looking for expected sub-folders instead of relying on naming.
    """
    episodes = [
        p
        for p in data_dir.iterdir()
        if p.is_dir()
        and (p / "steps").is_dir()
        and (p / "recordings" / "MP4").is_dir()
    ]
    return sorted(episodes)


def convert(
    data_dir: Path,
    repo_id: str,
    robot_type: str = "mycobot",
    image_size: int = 256,
    overwrite: bool = False,
    push_to_hub: bool = False,
    private_hub: bool = False,
    license_str: str = "apache-2.0",
) -> None:
    out_path = HF_LEROBOT_HOME / repo_id
    if out_path.exists() and overwrite:
        shutil.rmtree(out_path)

    episodes = _find_episodes(data_dir)
    if not episodes:
        raise FileNotFoundError(f"No episodes found under {data_dir}")

    # Define LeRobot schema (tuned to your JSON)
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        robot_type=robot_type,
        fps=10,  # default; we recompute per-episode
        features={
            "image": {
                "dtype": "image",
                "shape": (image_size, image_size, 3),
                "names": ["height", "width", "channel"],
            },
            "wrist_image": {
                "dtype": "image",
                "shape": (image_size, image_size, 3),
                "names": ["height", "width", "channel"],
            },
            "state": {  # [6 joint, 6 cartesian, 1 gripper] = 13
                "dtype": "float32",
                "shape": (13,),
                "names": ["state"],
            },
            "actions": {  # [6 joint_velocity, 1 gripper_velocity] = 7
                "dtype": "float32",
                "shape": (7,),
                "names": ["actions"],
            },
            "language_instruction": {
                "dtype": "string",
                "shape": (1,),
                "names": None,
            },
            "language_instruction_2": {
                "dtype": "string",
                "shape": (1,),
                "names": None,
            },
            "language_instruction_3": {
                "dtype": "string",
                "shape": (1,),
                "names": None,
            },
        },
        image_writer_threads=8,
        image_writer_processes=4,
    )

    for ep_dir in episodes:
        steps_dir = ep_dir / "steps"
        video_dir = ep_dir / "recordings" / "MP4"
        video_readers = _find_video_readers(video_dir)
        step_files = _sorted_step_files(steps_dir)
        if not step_files:
            continue

        timestamps = []
        prev_joint_pos = None
        prev_grip = None
        prev_raw_ts = None
        # First pass: gather timestamps to estimate FPS
        for sf in step_files:
            with open(sf, "r") as f:
                j = json.load(f)
            timestamps.append(j.get("timestamp", None))
        timestamps = [t for t in timestamps if isinstance(t, (int, float))]
        estimated_fps = _estimate_fps(timestamps) if timestamps else dataset.fps
        fps = float(estimated_fps) if estimated_fps else float(dataset.fps)

        # Optional: read episode-level instruction if you have an episode.json
        episode_task = ""
        ep_meta_file = ep_dir / "episode.json"
        if ep_meta_file.exists():
            try:
                with open(ep_meta_file, "r") as f:
                    ep_meta = json.load(f)
                episode_task = _normalize_instruction(ep_meta.get("language_instruction", ""))
            except Exception:
                episode_task = ""

        base_timestamp = None
        prev_timestamp = None
        timestep_s = 1.0 / float(dataset.fps) if dataset.fps > 0 else 0.1

        frames_added = 0

        for frame_idx, sf in enumerate(step_files):
            with open(sf, "r") as f:
                j = json.load(f)

            obs = j.get("observation", {})
            actdict = j.get("action_dict", {}) or {}
            primary_instruction = _normalize_instruction(j.get("language_instruction", ""), fallback=episode_task)
            instruction_2 = _normalize_instruction(j.get("language_instruction_2", ""))
            instruction_3 = _normalize_instruction(j.get("language_instruction_3", ""))
            instruction_pool = [inst for inst in (primary_instruction, instruction_2, instruction_3) if inst]
            task = instruction_pool[0] if instruction_pool else ""

            # ----- Video frame extraction -----
            main_ob = obs.get("exterior_image_1_left")
            wrist_ob = obs.get("wrist_image_left")

            def _fetch_frame(ob_entry):
                if not isinstance(ob_entry, dict):
                    return None
                video_path = ob_entry.get("video_path")
                frame_index = ob_entry.get("frame_index")
                if video_path is None or frame_index is None:
                    return None
                stem = Path(video_path).stem
                reader = video_readers.get(stem)
                if reader is None:
                    mp4_path = (ep_dir / video_path).resolve()
                    if mp4_path.exists():
                        reader = cv2.VideoCapture(str(mp4_path))
                        if reader.isOpened():
                            video_readers[stem] = reader
                        else:
                            return None
                    else:
                        return None
                # Seek and read frame
                reader.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
                ok, frame = reader.read()
                if not ok:
                    return None
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(frame_rgb, (image_size, image_size), interpolation=cv2.INTER_LINEAR)
                return resized

            main_img = _fetch_frame(main_ob)
            wrist_img = _fetch_frame(wrist_ob)

            if main_img is None or wrist_img is None:
                # Skip frames if videos are missing or frame index invalid
                continue

            # ----- State mapping -----
            # joint_position: length 6 (MyCobot), cartesian_position: length 6, gripper_position: scalar
            joint = np.asarray(obs.get("joint_position", []), dtype=np.float32).reshape(-1)
            cart = np.asarray(obs.get("cartesian_position", []), dtype=np.float32).reshape(-1)
            grip = np.asarray([obs.get("gripper_position", 0.0)], dtype=np.float32)

            # Validate/clip lengths
            if joint.size != 6 or cart.size != 6:
                # If lengths are off, pad/trim to 6
                joint = (np.pad(joint, (0, max(0, 6 - joint.size)))[:6]).astype(np.float32)
                cart = (np.pad(cart, (0, max(0, 6 - cart.size)))[:6]).astype(np.float32)

            state = np.concatenate([joint, cart, grip], dtype=np.float32)  # (13,)

            # ----- Action mapping (graceful if missing) -----
            raw_ts = j.get("timestamp", None)
            if isinstance(raw_ts, (int, float)) and np.isfinite(raw_ts) and base_timestamp is None:
                base_timestamp = float(raw_ts)

            dt = timestep_s
            if isinstance(raw_ts, (int, float)) and np.isfinite(raw_ts) and isinstance(prev_raw_ts, (int, float)) and np.isfinite(prev_raw_ts):
                delta = raw_ts - prev_raw_ts
                if delta > 0:
                    dt = float(delta)

            raw_jv = actdict.get("joint_velocity")
            if isinstance(raw_jv, (list, tuple, np.ndarray)) and len(raw_jv) >= 6:
                jvel = np.asarray(raw_jv, dtype=np.float32).reshape(-1)[:6]
            elif prev_joint_pos is not None and dt > 0:
                jvel = (joint - prev_joint_pos) / dt
            else:
                jvel = np.zeros(6, dtype=np.float32)
            if jvel.size < 6:
                jvel = np.pad(jvel, (0, 6 - jvel.size))[:6]
            jvel = np.nan_to_num(jvel, nan=0.0, posinf=0.0, neginf=0.0)

            raw_gv = actdict.get("gripper_velocity")
            if isinstance(raw_gv, (int, float)) and np.isfinite(raw_gv):
                gvel = np.array([float(raw_gv)], dtype=np.float32)
            elif prev_grip is not None and dt > 0:
                gvel = np.array([(grip - prev_grip) / dt], dtype=np.float32).reshape(1)
            else:
                gvel = np.array([0.0], dtype=np.float32)
            gvel = np.nan_to_num(gvel, nan=0.0, posinf=0.0, neginf=0.0)

            actions = np.concatenate([jvel, gvel], dtype=np.float32)  # (7,)

            prev_joint_pos = joint.copy()
            prev_grip = grip.copy()
            if isinstance(raw_ts, (int, float)) and np.isfinite(raw_ts):
                prev_raw_ts = raw_ts

            # Force timestamps onto an evenly spaced grid derived from fps.
            ts_value = frame_idx * timestep_s

            if prev_timestamp is not None and ts_value <= prev_timestamp:
                ts_value = prev_timestamp + timestep_s

            prev_timestamp = ts_value

            dataset.add_frame(
                {
                    "image": main_img.astype(np.uint8),
                    "wrist_image": wrist_img.astype(np.uint8),
                    "state": state,
                    "actions": actions,
                    "task": str(task),
                    "language_instruction": primary_instruction,
                    "language_instruction_2": instruction_2,
                    "language_instruction_3": instruction_3,
                    "timestamp": np.array([ts_value], dtype=np.float32),
                }
            )
            frames_added += 1

            # Collapse timestamp to scalar so it matches episode_index shape during validation.
            dataset.episode_buffer["timestamp"][-1] = np.float32(prev_timestamp)

        if frames_added == 0:
            # No valid frames collected for this episode, skip saving.
            continue

        dataset.save_episode()

    if push_to_hub:
        dataset.push_to_hub(
            tags=["mycobot", "droid", "openpi", "lerobot"],
            private=private_hub,
            push_videos=True,
            license=license_str,
        )


def main(
    data_dir: str,
    repo_id: str,
    robot_type: str = "mycobot",
    image_size: int = 256,
    overwrite: bool = False,
    push_to_hub: bool = False,
    private_hub: bool = False,
    license_str: str = "apache-2.0",
):
    convert(
        data_dir=Path(data_dir),
        repo_id=repo_id,
        robot_type=robot_type,
        image_size=image_size,
        overwrite=overwrite,
        push_to_hub=push_to_hub,
        private_hub=private_hub,
        license_str=license_str,
    )


if __name__ == "__main__":
    tyro.cli(main)
