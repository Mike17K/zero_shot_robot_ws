# Vision: zero-shot box pose from the gripper camera

Status (2026-09-25): the perception pipeline is implemented, runs on demand, and **the demo uses it**: at the pick node it aligns over the biggest detected box, then makes the force-guarded approach (see [Demo integration](#6-demo-integration)). Accuracy against ground truth still has to be measured in sim (see [Verification](#verification)).

## 1. What it does

Given one colour + depth frame from the robot's wrist camera, it finds every box **top face** in view and returns, for each one:
- the face centre;
- its orientation: long edge, short edge, and face normal. The normal points towards the camera, so the suction approach direction is −normal;
- the face size.

Poses are expressed in `world`. No training or box models are involved ("zero-shot"): a generic segmenter proposes regions and geometry decides which of them are box tops.

```
 camera/color ──► MobileSAM automatic masks ──┐
                                              ├─► per mask: back-project ─► RANSAC plane ─► min-area rectangle ─► filters
 camera/depth ────────────────────────────────┘                                                                    │
 camera/camera_info (K) ─────────────────────────────────────────────────────────────────────────────────────────┘
                                                                    dedupe ─► sort by distance ─► TF to world ─► publish
```

## 2. Sensor side (already part of the robot)

The camera is defined in `src/robots/group_a/group_a_description/urdf/group_a_macro.xacro`:
- **Mounting:** at the gripper plate's +Y edge, looking along the suction cups (the tool's +Z). It sits behind the force pad, flush with the cup tips.
- **Two Gazebo sensors on `camera_link`:** `depth_camera` (32FC1, metres) and `rgb_camera` (R8G8B8). Both are 320×240 at 10 Hz, horizontal FOV 1.212 rad, clip 0.10–3.0 m. They share pose, FOV and resolution, so the **images are pixel-aligned** and share one `camera_info`.
- **Optical frame:** `camera_optical_frame` (REP-103: z forward, x right, y down), set as each sensor's `optical_frame_id`.
- **Bridge:** `group_a_bringup/config/gz_bridge.yaml` bridges `/<ns>/camera/color`, `/<ns>/camera/depth` and `/<ns>/camera/camera_info` into ROS.
- **TF:** on the robot's own `/<ns>/tf` (not `/tf`), like everything robot-specific in this workspace.

The live feed is also visible in the teleop UI's **Camera** tab.

## 3. External (non-ROS) libraries

These live in `external/` as pinned git submodules and are put on `sys.path` at runtime. They are not installed with pip or rosdep.

| Path | What | Pin |
|---|---|---|
| `external/MobileSAM` | MobileSAM (`mobile_sam` package): SAM with a TinyViT image encoder | master `f706ad9` |
| `external/pytorch-image-models` | `timm`, needed by MobileSAM's TinyViT | `v1.0.30` |
| `external/weights/mobile_sam.pt` | ViT-T checkpoint (~40 MB), **gitignored** | — |

- **Setup:** `scripts/setup_external.sh` (idempotent) adds and updates the submodules, pins `timm`, and copies or downloads the weights.
- **Runtime lookup:** `src/shared_utils/shared_utils/external.py`. `add_external_to_path('MobileSAM', 'pytorch-image-models')` prepends `external/<name>` to `sys.path`, and `external_file('weights', 'mobile_sam.pt')` builds paths. The external directory is `$ZSR_EXTERNAL_DIR`, else `/workspaces/isaac_ros-dev/external`. A missing library raises an `ImportError` that says to run the setup script.
- **From the container:** torch and torchvision with CUDA (NVIDIA image), numpy, OpenCV. `open3d` is deliberately not used.

## 4. The detector node: `box_pose_estimator`

The code is in `src/vision/vision/box_pose_estimator.py` (ROS side) and `src/vision/vision/box_geometry.py` (pure numpy/OpenCV geometry, unit-testable without ROS or torch). Launch file: `src/vision/launch/box_pose.launch.py`.

### Interface (node `/<ns>/box_pose_estimator`, default ns `robot_1`)

| | Name | Type | Notes |
|---|---|---|---|
| sub | `camera/color` | `sensor_msgs/Image` | rgb8 (bgr8 accepted) |
| sub | `camera/depth` | `sensor_msgs/Image` | 32FC1 metres (16UC1 mm accepted) |
| sub | `camera/camera_info` | `sensor_msgs/CameraInfo` | K and optical frame id |
| srv | `~/detect` | `std_srvs/Trigger` | runs one estimate on the latest frames; the reply message is a summary |
| pub | `~/detections` | `geometry_msgs/PoseArray` | top-face centres in `target_frame`, closest to the camera first; +Z = face normal |
| pub | `~/markers` | `visualization_msgs/MarkerArray` | `box_top` flat cubes (`scale` = face size) + `box_label` texts |
| pub | `~/debug_image` | `sensor_msgs/Image` | masks tinted, fitted rectangles projected back; latched (transient local), one image per estimate. Shown in the teleop Camera tab as **detections** |

It runs **on demand** by default: SAM shares the 4 GB GPU with cuMotion. `rate_hz > 0` adds a timer for continuous mode, which skips a tick while an estimate is still running.

### Pipeline, step by step
1. **Frames:** the latest colour, depth and K are kept. The colour and depth stamps must be within `max_pair_dt` (0.15 s). Images are converted with plain numpy, without `cv_bridge`.
2. **Masks:** MobileSAM `vit_t` with `SamAutomaticMaskGenerator`, on CUDA (CPU fallback with a warning), under `torch.inference_mode()`:
   - a 16×16 prompt grid, decoded 8 prompts per batch;
   - `pred_iou_thresh` 0.88, `stability_score_thresh` 0.90;
   - the CUDA cache is emptied after every estimate, since cuMotion shares the 4 GB GPU;
   - running out of GPU memory makes `~/detect` fail with a message instead of killing the node.
3. **Mask filters** (before any geometry):
   - area < `min_mask_area` (300 px) → rejected as noise;
   - area > `max_mask_fraction` (50 %) of the image → rejected as the belt or floor;
   - touching the image border (within `border_margin`, 2 px) → rejected, because a box cut off by the frame edge would give a wrong centre and size.
4. **Back-projection** (`backproject`): the mask's pixels with valid depth (0.10–3.0 m) become 3D points in the optical frame via K. Fewer than 50 points → rejected.
5. **Plane** (`fit_plane`): RANSAC over 100 iterations with a 5 mm inlier tolerance, then refined by SVD on the inliers. The normal is flipped to face the camera. Fewer than 80 inliers → rejected. This keeps the largest planar part of the mask, the top face, even when SAM's mask also covers a side face.
6. **Tilt check:** the angle between the normal and the camera axis must be ≤ `max_tilt_deg` (35°). Steeper planes are side faces, or the camera is looking too obliquely.
7. **Rectangle:** the inliers are projected onto an in-plane basis, and `cv2.minAreaRect` gives the centre, the two edge lengths and the edge directions. Edges outside `min_size`–`max_size` (0.05–0.50 m, which covers box_factory's range) → rejected.
8. **Face frame:** columns are the long edge, the short edge (normal × long edge) and the normal. The centre is the rectangle centre on the plane.
8b. **Raised above its surroundings:** in a 6 px ring just outside the mask, at least `min_drop_fraction` (50 %) of the pixels must lie `min_height` (2 cm) or more below the face. Pixels with no depth return count as lower. This rejects flat marks and belt surfaces. It is deliberately local: an earlier version fitted one global "belt plane" to the whole depth image, and that plane locked onto the box top itself when the box filled most of the view, so real boxes got rejected. Belt rollers are also caught by `max_aspect` in step 7.
8c. **Merge split faces:** faces in the same plane (normals within ~11°, within 2 × `plane_tolerance` of each other) whose masks touch are joined into one mask and refitted. SAM often splits one top face along a shadow line or around a label. Caveat: two same-height boxes standing flush against each other can merge into one, and then usually fail the size or aspect limits.
9. **Dedupe:** SAM proposes nested masks (whole box, top face, parts of it). Faces whose centres are within `dedupe_distance` (3 cm) of a face with more plane inliers are dropped.
10. **Order and transform:** faces are sorted by distance from the camera, then each face pose goes optical frame → `target_frame` (`world`) through `shared_utils.geometry.TfHelper` on `/<ns>/tf`.
11. **Publish:** the pose array, markers, the debug image (if subscribed) and a log line. The log line and the service reply look like `2 box(es) from 17 masks in 180 ms: 0.30x0.20 m at (x, y, z); …`, or list the rejection reasons when nothing passed (small, belt-sized, cut off by the image edge, few depth points, no clear plane, tilted, wrong size, duplicate).

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `weights` | `''` | checkpoint; empty = `external/weights/mobile_sam.pt` |
| `device` | `cuda` | torch device (falls back to `cpu`) |
| `target_frame` | `world` | frame of the published poses |
| `rate_hz` | `0.0` | continuous estimates per second; 0 = only `~/detect` |
| `max_pair_dt` | `0.15` | s between the colour and depth frames used together |
| `points_per_side` | `16` | SAM prompt grid |
| `points_per_batch` | `8` | prompts decoded at once. Each one costs ~50–100 MB of VRAM; SAM's default of 64 runs out of memory next to cuMotion |
| `pred_iou_thresh` / `stability_score_thresh` | `0.88` / `0.90` | SAM mask quality thresholds |
| `min_mask_area` | `300` | px |
| `max_mask_fraction` | `0.5` | of the image |
| `border_margin` | `2` | px |
| `dedupe_distance` | `0.03` | m |
| `plane_tolerance` | `0.005` | m, RANSAC inlier distance |
| `min_inliers` | `80` | plane points |
| `max_tilt_deg` | `35.0` | face normal vs camera axis |
| `min_size` / `max_size` | `0.05` / `0.50` | m, face edge limits |
| `max_aspect` | `4.0` | long / short edge; belt rollers and rails are long thin strips |
| `min_height` | `0.02` | m a face must stand above its surroundings |
| `min_drop_fraction` | `0.5` | share of the ring around the mask that must be `min_height` lower |

## 5. Ground-truth check: `box_pose_check`

`src/vision/vision/box_pose_check.py` measures accuracy in sim:
- **Box sizes:** from `/box_factory/spawned` (`custom_msgs/SpawnedBox`).
- **Live box poses:** read natively from gz-transport `/world/<world>/dynamic_pose/info`, like `gripper_manager.py` does, because `ros_gz_bridge` drops entity names.
- **Expected top-face centre:** box centre + ½ height along the box's +Z.
- **For each detected face** (read from `~/markers`, which carry the face size), it finds the nearest ground-truth box within 0.25 m and logs position error, yaw error (mod 90°) and size error, plus running mean and max.
- With `trigger_period > 0` it calls `~/detect` itself, so it runs hands-off.

## 6. Demo integration

**Client (`src/shared_utils/shared_utils/perception.py`):** `BoxDetectionClient(node, namespace).detect()` calls `~/detect` and returns `DetectionResult(success, message, boxes)`. Each `BoxDetection` has a `pose` (PoseStamped, top-face centre, +X = long edge, +Z = face normal) and a `size` (long, short).

`~/detect` only replies with a text summary. The client therefore counts the `~/markers` messages it has received, calls the service, and waits (≤ 2 s) for the next markers message, which the estimator publishes just before replying. It reads the faces from that message.

**Demo (`src/workcell/workcell_demo/workcell_demo/pick_place_demo.py`), with `use_vision: true`:**
1. navigate to `inspect_node` (default `5`, the InspectionPoint, which is also the home pose, so usually the arm is already there). Node 6 is no longer used;
2. `detect()`, then keep the **biggest** face (by area). Failing to detect, or seeing no box, is a step failure (ERROR);
3. **grasp pose** (`grasp_pose_from_face`):
   - the tool's +Z (out of the cups) goes **into** the face (−normal);
   - the plate's long side (tool +Y, 0.40 m) lies along the face's long edge;
   - of the two orientations 180° apart about the normal, it takes the one closer to the current tool orientation, so the wrist turns as little as possible;
   - position = face centre + `grasp_standoff` (0.10 m) along the normal;
4. **align:** a straight tool line from the inspection pose to that pose (`NavigatorClient.move_to_pose`, the new `use_target_pose` mode of `cartesian_move`) at `align_speed`;
5. **approach:** the existing force-guarded Cartesian move, now along the **tool** +Z, so tilted faces work too (max `approach_max_distance`, stop at `contact_force`);
6. then as before: suction on, wait for the grasp, lift `lift_distance` straight up, place, home.

A missed contact or a failed grasp backs out along the same tool axis. With `use_vision: false`, the approach goes straight down from `inspect_node` itself.

The camera has to see the whole box from `inspect_node`: the detector drops boxes cut off by the image edge. The align move is one straight line from there, so the grasp pose must be reachable along a line from the inspection pose.

## 7. How to run

```bash
# once, inside the container (done): libraries + weights
./scripts/setup_external.sh
colcon build --symlink-install --packages-select shared_utils vision && source install/setup.bash

# with the workcell running and the gripper looking at the infeed belt:
ros2 launch vision box_pose.launch.py                     # rate_hz:=1.0 for continuous
ros2 service call /robot_1/box_pose_estimator/detect std_srvs/srv/Trigger
ros2 run rqt_image_view rqt_image_view /robot_1/box_pose_estimator/debug_image
ros2 run vision box_pose_check --ros-args -p trigger_period:=5.0
```
In RViz, add a MarkerArray display on `/robot_1/box_pose_estimator/markers`, with the fixed frame `world` and TF remapped to `/robot_1/tf` (see `workcell_bringup/launch/rviz.launch.py`).

## Verification

| Check | Result |
|---|---|
| Geometry on synthetic depth renders (camera tilted 0° and 20°, 1 mm depth noise) | centre error ≈ 0.1 mm, sizes exact, long edge within 0.1° |
| Side face at 60° | rejected (`tilted`) |
| Whole node with a fake mask generator (2 boxes, 1 cut off at the image edge, the belt) | 2 faces at the right depth and size; cut-off box and belt rejected; all outputs published |
| In Gazebo with real MobileSAM | **to do**: target < 1 cm position, < 5° yaw with `box_pose_check`; watch `nvidia-smi` for VRAM next to cuMotion |

## 8. Known limitations / next steps
- **Speed:** the automatic mask generator prompts a 16×16 grid and SAM resizes the image to 1024 px internally, so each estimate costs a few hundred ms on the GPU. Options:
  - fewer prompt points (`points_per_side`);
  - prompting only the centre region, or with the depth-based foreground (pixels closer than the belt);
  - running SAM's image encoder once and prompting with `SamPredictor`.
- **TF timing:** poses use the *latest* transform, not the one at the image stamp. Estimate while the arm is still (the demo's pick pose is).
- **Top face only:** the box height isn't estimated. The force-guarded approach finds the actual surface anyway. Height could come from the distance between the face and the belt plane.
- **Demo picks the biggest face only;** the others are ignored until the next cycle.
- **Clutter:** touching boxes of similar colour can come out as one SAM mask. They then fail the size check, or give one wide face.
- **nvblox:** its launch and config remain in the `vision` package but are unused (see `docs/STRUCTURAL_DESISIONS.md`).
