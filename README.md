# SeniorDesign
This is an overview of what the plan is for our senior design project.

#### 9/7/2025
<img width="1739" height="1308" alt="image" src="https://github.com/user-attachments/assets/69d013b8-ca17-4df0-99fb-c5ec709309ab" />
Color Picker now functional!
TODO: Get a block, and get coordinates for said block

## What we Physically Need
1. MyCobot 280
2. Camera setup at Rutgers
3. [Camera on the robot](https://shop.elephantrobotics.com/products/camera-flange-2-0?srsltid=AfmBOorrMenfHOfeE5Xx8k1fwQRWLs7Ss3pe-imzu8PcDf_VmWWRW31G) <mark>Please order this ASAP!</mark>
## Our Process
Our MyCobot is built up in three stages. These are:
1. Identify what object to pick up
2. Pick up the object
3. Drop it in the user's open hand
## Identifying what to pick up
This part shouldn't be too bad, although there isn't any direct examples of it anywhere, it shouldn't be too difficult. Make sure to have a conda environment set up within ROS, as you'll be heavily relying on Python within ROS. Also, make sure to use Linux or WSL (if you're developing on a Windows PC), as ROS really only plays nice with Linux. If you develop directly on the Pi, it might be slightly slower than on your own computer, and also you won't be able to bring the MyCobot home. However, developing on another computer will require SSH into the computer, and then sending ROS topics over a network.

Someone was able to create a [Mediapipe Wrapper](https://github.com/chrisywong/ros_mediapipe?tab=readme-ov-file) within ROS that we should be able to use for this step. Just follow the instructions for how to get the hand pose estimation to work, and then vibecode how to get the color that you're pointing at.

**Alternative:** Using a mouse to click on a color would be much, much easier to do.
## Pick up the object
This should be the simplest part, as there is an [example](https://docs.elephantrobotics.com/docs/gitbook-en/13-AdvancedKit/13.1%E4%BA%BA%E5%B7%A5%E6%99%BA%E8%83%BD/13.1.4-%E5%9B%BE%E5%83%8F%E8%AF%86%E5%88%AB/mycobot_280.html) already online made by Elephant Robotics themselves. Just follow the instructions, and you should be set.
## Drop it in the user's open hand
This part should be the hardest, but it shouldn't necessarily be too hard. We're essentially making a more advanced version of [this](https://www.hackster.io/adambeedle/full-auto-nerf-gun-that-shoots-you-in-face-using-opencv-19a241). The camera on the flange should first identify where the open hand is, and then it would iteratively move closer and closer to the hand so the it takes up a certain amount of space in the middle of the camera's field of view before releasing the object. We'd still be using the Mediapipe Wrapper from before.
## Some Concerns
I don't exactly know what ROS version to use, you would have to check the documentation of the wrapper and the ER example to know for sure (I think ROS Noetic should be the right one). Also, defining what success is might be a bit difficult (probably via confidence levels, I don't know what else to put).
## How long should this take?
ChatGPT said 2-3 months, honestly if we keep pushing we should be done in a month, I would be surprised if we don't finish in that timeframe because of how much code we can rip off/vibecode.
From ChatGPT:

Yes—at **8–12 hrs/week on the myCobot 280 Pi**, expect **\~8–12 weeks** for the two-camera plan (overhead for select/pick, wrist for hand-off) using MediaPipe in a Conda env. Here’s a concrete week-by-week you can copy into your repo.

# Week-by-week (solo, part-time)

### Week 1 — Bring-up on the 280 Pi

* Flash/verify Ubuntu + ROS1 (Noetic) on the Pi; `mycobot_ros` working; jog joints & gripper.
* Plug in **both cameras**; confirm `/dev/video0/1` stable; publish both image topics.
* Create **Conda env** for MediaPipe (python=3.8), add an **env-loader** script so ROS nodes launch inside Conda; verify a minimal `rospy` + MediaPipe import.
  **Done when:** `roslaunch` starts the arm and both cameras; a test node in Conda prints MediaPipe version.

### Week 2 — Overhead camera calibration + selector MVP

* Calibrate **intrinsics** (overhead), save `camera_info`.
* Do a **planar homography** to table (printable grid or 4-point corners).
* Implement **selector\_overhead**:

  * Start **simple**: color masks + choose **most centered blob** (or a mouse click).
  * Publish `/target_pixel` + `/target_color`.
    **Done when:** clicking or centering a block yields stable `/target_pixel` at ≥10–15 FPS.

### Week 3 — Pixel→Pose + pick execution

* `pixel_to_pose_overhead`: apply homography → table (x,y) → add grasp Z + TCP offset.
* `pick_server`: approach → close → lift → retreat (MoveIt waypoints or driver script).
  **Done when:** **≥80% pick success** on 3–4 colors, 10 trials each.

### Week 4 — Wrist camera bring-up + intrinsics

* Mount flange camera; calibrate **intrinsics**.
* Create `handoff_wrist` package; run MediaPipe Hands on that stream inside Conda.
* Add simple **open-palm** detector (e.g., “fingers extended” heuristic using landmarks).
  **Done when:** node publishes `/hand_bbox` + confidence at ≥8–12 FPS on the Pi.

### Week 5 — Hand–eye (wrist) + safe approach pose

* Calibrate **hand–eye** once to get $T^{ee}_{cam\_wrist}$; verify TF with a checker.
* Define a **handoff staging pose** (arm near user, camera view centered).
  **Done when:** you can transform a wrist-pixel ray to `base_link` plausibly; staging pose is reachable and safe.

### Weeks 6–7 — Hand-off controller v1

* Control loop (low speed): if palm bbox **inside center window** and **area > N%** for **K frames**, open gripper; else step closer.
* Add **abort/retreat** if palm lost **M frames** or out of bounds.
* Clamp speeds, add E-stop & workspace fence.
  **Done when:** **≥80%** success into a **stationary** open palm; drop ≤5 cm.

### Week 8 — Orchestrator + retries + demo script

* `orchestrator` state machine: **SELECT (overhead) → PICK → HANDOFF (wrist)**; enable/disable camera nodes per state to save CPU.
* Add grasp **retry** (re-plan once), and a **demo script** that runs end-to-end.
  **Done when:** 5 back-to-back runs with no operator intervention.

### Weeks 9–12 (as needed) — Robustness polish

* Lighting tolerance (per-session color range calibration), temporal smoothing, log & simple UI, documentation, video.

---

## Acceptance criteria (so “done” is objective)

* **Selector:** ≥90% correct target in your tabletop layout.
* **Pick:** ≥80% success across colors (10 trials/color).
* **Hand-off:** ≥80% success to a stationary open hand; drop height ≤5 cm.
* **Throughput:** ≤20 s per full cycle in demo mode.
* **Safety:** speed limits on; E-stop reachable; abort on palm loss.

---

## 280 Pi–specific tips (keeps you on schedule)

**Performance on Pi**

* Expect MediaPipe Hands on a Pi to run \~8–15 FPS at **640×480** with `model_complexity=0`, `max_num_hands=1`. If it stutters:

  * Downscale to **480p or 360p** for the wrist feed during HANDOFF.
  * Use `bgr8` frames, single-threaded processing loop, and **temporal voting** (K=5–8) to hide jitter.
* Keep OpenCV/MediaPipe **inside Conda**. Avoid mixing with system `cv_bridge`; read/write `sensor_msgs/Image` directly (no `cv_bridge`) in the Conda node to prevent ABI clashes.

**USB & cameras**

* Two UVC cams on a Pi share bandwidth; set **MJPEG** at modest fps (e.g., 640×480 @ 15–20 fps).
* Pin device indices with udev rules (symlinks `/dev/cam_over`, `/dev/cam_wrist`) so launch files are stable.

**Power & thermals**

* Give the Pi a solid 5V/3A supply; add a small fan. Thermal throttling will tank your FPS.

**Services over SSH**

* Turn long-running nodes into **systemd services** on the Pi (arm bring-up, cameras, MediaPipe). Use SSH only to inspect logs/restart—not per command.

**If the Pi falls behind**

* Keep a **Plan B**: run MediaPipe on your laptop and send results over ROS. You’ve already built topic interfaces, so offloading is trivial if needed.

---

## What to decide up front (prevents rework)

* **Homography vs AprilTag** for overhead extrinsic (pick one and stick to it).
* **Grasp Z & TCP offset** values (measure once; check per color block).
* **Exact success metrics** (copy the criteria above into your README and test against it weekly).

---

### Bottom line

At your pace (8–12 hrs/week) **2–3 months** is the right expectation. Follow the week-by-week, keep vision inside Conda on the Pi, and lock calibration early. If you want, I can turn this into a one-page checklist or drop a skeleton `orchestrator` + launch files wired for two cameras and the Conda env-loader.
