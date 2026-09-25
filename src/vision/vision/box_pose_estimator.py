#!/usr/bin/env python3
"""Zero-shot box pose from the gripper's RGB-D camera.

MobileSAM (external/MobileSAM, automatic mask generator) proposes masks on the
colour image; each mask is lifted to 3D with the depth image and reduced to a
box top face - largest plane (RANSAC), oriented minimum-area rectangle - see
box_geometry.py. Faces are expressed in target_frame via the robot's TF.

The colour and depth sensors share pose, FOV and resolution
(group_a_macro.xacro), so the images are pixel-aligned and share camera_info.

Runs on demand (~/detect) - SAM next to cuMotion on a 4 GB GPU is not free -
or continuously with rate_hz > 0.

ROS interface (node <ns>/box_pose_estimator):
  subscribes  camera/color (rgb8), camera/depth (32FC1), camera/camera_info
  ~/detect        std_srvs/Trigger              run one estimate on the latest frames
  ~/detections    geometry_msgs/PoseArray       top-face centres, closest to the camera first;
                                                +Z = face normal (suction approach = -Z)
  ~/markers       visualization_msgs/MarkerArray  one flat cube per face (scale = face size) + label
  ~/debug_image   sensor_msgs/Image (rgb8, latched)  masks tinted, fitted rectangles drawn
"""
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from shared_utils.external import add_external_to_path, external_file
from shared_utils.geometry import TfHelper, matrix_to_pose

from .box_geometry import FaceParams, TopFace, backproject, dedupe, drop_fraction, project, top_face

COLORS = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48),
          (145, 30, 180), (70, 240, 240), (240, 50, 230)]


def image_to_numpy(msg: Image) -> np.ndarray:
    """rgb8/bgr8 -> (H, W, 3) uint8 RGB; 32FC1 -> (H, W) float32 metres; 16UC1 (mm) -> metres."""
    h, w, enc = msg.height, msg.width, msg.encoding
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, msg.step)[:, :w * 3].reshape(h, w, 3)
        return img[..., ::-1].copy() if enc == 'bgr8' else img.copy()
    if enc == '32FC1':
        return buf.view(np.float32).reshape(h, msg.step // 4)[:, :w].copy()
    if enc == '16UC1':
        return buf.view(np.uint16).reshape(h, msg.step // 2)[:, :w].astype(np.float32) * 1e-3
    raise ValueError(f'unsupported image encoding {enc!r}')


def _reason_category(why: str) -> str:
    """Group top_face()'s rejection reasons for the summary line."""
    for key, label in (('depth points', 'few depth points'), ('inliers', 'no clear plane'),
                       ('no plane', 'no clear plane'), ('tilted', 'tilted (side face)'),
                       ('size', 'wrong size'), ('aspect', 'strip (aspect)')):
        if key in why:
            return label
    return why


class BoxPoseEstimator(Node):
    def __init__(self):
        super().__init__('box_pose_estimator')
        p = self.declare_parameter
        p('weights', '')                   # empty = external/weights/mobile_sam.pt
        p('device', 'cuda')
        p('target_frame', 'world')
        p('rate_hz', 0.0)
        p('max_pair_dt', 0.15)              # s between the colour and depth frames used together
        p('points_per_side', 16)            # SAM prompt grid (fewer = faster; the scene is just boxes)
        p('points_per_batch', 8)            # prompts decoded at once: ~50-100 MB of VRAM each (SAM default 64 OOMs next to cuMotion)
        p('pred_iou_thresh', 0.88)
        p('stability_score_thresh', 0.90)
        p('min_mask_area', 300)             # px
        p('max_mask_fraction', 0.5)         # of the image: bigger masks are the belt / floor
        p('border_margin', 2)               # px: masks touching the image edge are cut-off boxes
        p('dedupe_distance', 0.03)          # m between face centres
        p('min_height', 0.02)               # m a face must stand above its surroundings (belt / floor)
        p('min_drop_fraction', 0.5)         # of a 6 px ring around the mask that must lie min_height lower
        defaults = FaceParams()
        face_names = ('plane_tolerance', 'min_inliers', 'max_tilt_deg', 'min_size', 'max_size', 'max_aspect')
        for name in face_names:
            p(name, getattr(defaults, name))
        self.face_params = FaceParams(**{
            name: type(getattr(defaults, name))(self.get_parameter(name).value) for name in face_names})

        self._mask_gen = self._load_sam()
        ns = self.get_namespace().strip('/') or 'robot_1'
        self.tf = TfHelper(self, f'/{ns}/tf')

        self._lock = threading.Lock()          # latest frames
        self._busy = threading.Lock()          # one estimate at a time
        self._color = self._depth = None
        self._K = None
        self._optical_frame = 'camera_optical_frame'
        images = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, 'camera/color', self._on_color, qos_profile_sensor_data,
                                 callback_group=images)
        self.create_subscription(Image, 'camera/depth', self._on_depth, qos_profile_sensor_data,
                                 callback_group=images)
        self.create_subscription(CameraInfo, 'camera/camera_info', self._on_info, qos_profile_sensor_data,
                                 callback_group=images)

        self._poses_pub = self.create_publisher(PoseArray, '~/detections', 10)
        self._markers_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        # Latched: a viewer opened after an estimate still gets its result.
        self._debug_pub = self.create_publisher(
            Image, '~/debug_image', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        work = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/detect', self._on_detect, callback_group=work)
        rate = float(self.get_parameter('rate_hz').value)
        if rate > 0.0:
            self.create_timer(1.0 / rate, self._on_timer, callback_group=work)
        self.get_logger().info(
            f'box_pose_estimator ready ({"every %.1f s" % (1.0 / rate) if rate > 0 else "on ~/detect"}), '
            f'poses in {self.get_parameter("target_frame").value}')

    def _load_sam(self):
        add_external_to_path('MobileSAM', 'pytorch-image-models')
        import torch
        from mobile_sam import SamAutomaticMaskGenerator, sam_model_registry

        device = self.get_parameter('device').value
        if device.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn('CUDA not available to torch - running MobileSAM on the CPU (slow)')
            device = 'cpu'
        weights = self.get_parameter('weights').value or external_file('weights', 'mobile_sam.pt')
        sam = sam_model_registry['vit_t'](checkpoint=weights)
        sam.to(device).eval()
        self._torch = torch
        self.get_logger().info(f'MobileSAM (vit_t) on {device} from {weights}')
        return SamAutomaticMaskGenerator(
            sam,
            points_per_side=int(self.get_parameter('points_per_side').value),
            pred_iou_thresh=float(self.get_parameter('pred_iou_thresh').value),
            stability_score_thresh=float(self.get_parameter('stability_score_thresh').value),
            points_per_batch=int(self.get_parameter('points_per_batch').value),
            min_mask_region_area=int(self.get_parameter('min_mask_area').value))

    # ── Inputs ───────────────────────────────────────────────────────────────

    def _on_color(self, msg: Image) -> None:
        with self._lock:
            self._color = msg

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._depth = msg

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if msg.header.frame_id:
                self._optical_frame = msg.header.frame_id

    def _frames(self):
        with self._lock:
            color, depth, K = self._color, self._depth, self._K
        if color is None or depth is None or K is None:
            return None, 'no camera frames yet (camera/color, camera/depth, camera/camera_info)'
        stamp = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9  # noqa: E731
        dt = abs(stamp(color) - stamp(depth))
        if dt > float(self.get_parameter('max_pair_dt').value):
            return None, f'colour and depth frames {dt:.2f} s apart'
        return (color, depth, K), ''

    # ── Estimate ─────────────────────────────────────────────────────────────

    def _on_detect(self, request, response):
        ok, response.message = self.estimate()
        response.success = ok
        return response

    def _on_timer(self) -> None:
        if not self._busy.locked():
            self.estimate()

    def estimate(self):
        with self._busy:
            frames, why = self._frames()
            if frames is None:
                return False, why
            color_msg, depth_msg, K = frames
            rgb, depth = image_to_numpy(color_msg), image_to_numpy(depth_msg)
            if rgb.shape[:2] != depth.shape:
                return False, f'colour {rgb.shape[:2]} and depth {depth.shape} sizes differ'

            t0 = time.monotonic()
            try:
                with self._torch.inference_mode():
                    masks = self._mask_gen.generate(rgb)
            except self._torch.OutOfMemoryError as exc:
                return False, f'MobileSAM ran out of GPU memory - lower points_per_batch ({exc})'.split('\n')[0]
            finally:
                # Give the decoder's scratch memory back: cuMotion shares the GPU.
                if self._torch.cuda.is_available():
                    self._torch.cuda.empty_cache()
            t_sam = time.monotonic() - t0

            faces, face_masks, rejected = self._faces(masks, depth, K)
            order = np.argsort([np.linalg.norm(f.center) for f in faces])
            faces = [faces[i] for i in order]
            face_masks = [face_masks[i] for i in order]

            poses = self._to_target(faces, color_msg.header.stamp)
            if poses is None:
                return False, f'TF {self._optical_frame} -> {self.get_parameter("target_frame").value} unavailable'
            self._publish(poses, faces, color_msg.header.stamp)
            self._publish_debug(rgb, faces, face_masks, K, color_msg.header)

            summary = '; '.join(
                f'{f.size[0]:.2f}x{f.size[1]:.2f} m at ({ps.pose.position.x:+.3f}, '
                f'{ps.pose.position.y:+.3f}, {ps.pose.position.z:+.3f})'
                for f, ps in zip(faces, poses))
            msg = (f'{len(faces)} box(es) from {len(masks)} masks in {t_sam * 1000:.0f} ms'
                   + (f': {summary}' if faces else f' (rejected: {rejected})'))
            self.get_logger().info(msg)
            return True, msg

    def _faces(self, masks, depth, K):
        h, w = depth.shape
        border = int(self.get_parameter('border_margin').value)
        max_area = float(self.get_parameter('max_mask_fraction').value) * h * w
        min_area = int(self.get_parameter('min_mask_area').value)
        fp = self.face_params
        faces, face_masks, reasons = [], [], {}

        def reject(reason):
            reasons[reason] = reasons.get(reason, 0) + 1

        for m in masks:
            seg = m['segmentation']
            if m['area'] < min_area:
                reject('small')
                continue
            if m['area'] > max_area:
                reject('belt-sized')
                continue
            if border > 0 and (seg[:border].any() or seg[-border:].any()
                               or seg[:, :border].any() or seg[:, -border:].any()):
                reject('cut off by the image edge')
                continue
            face, why = top_face(backproject(seg, depth, K, fp.near, fp.far), fp)
            if face is None:
                reject(_reason_category(why))
                continue
            # Raised above its surroundings? Local, so it can't be fooled the way a
            # global "belt plane" is when a box top fills most of the image.
            if drop_fraction(face, seg, depth, K, fp, float(self.get_parameter('min_height').value)) \
                    < float(self.get_parameter('min_drop_fraction').value):
                reject('not raised (flat on the belt)')
                continue
            faces.append(face)
            face_masks.append(seg)
        faces, face_masks, merged = self._merge_coplanar(faces, face_masks, depth, K)
        if merged:
            reject(f'merged x{merged}')
        kept = dedupe(faces, float(self.get_parameter('dedupe_distance').value))
        mask_of = {id(f): seg for f, seg in zip(faces, face_masks)}
        kept_masks = [mask_of[id(f)] for f in kept]
        if len(kept) < len(faces):
            reject(f'duplicate x{len(faces) - len(kept)}')
        return kept, kept_masks, ', '.join(f'{k} {v}' for k, v in reasons.items()) or 'none'

    def _merge_coplanar(self, faces, masks, depth, K):
        """Join faces that lie in the same plane and whose masks touch - one
        box top that SAM split in two (a shadow line, a label) - and refit the
        union. Repeats until nothing merges; returns (faces, masks, merges)."""
        fp = self.face_params
        kernel = np.ones((7, 7), np.uint8)
        merges = 0
        changed = True
        while changed:
            changed = False
            for i in range(len(faces)):
                for j in range(i + 1, len(faces)):
                    a, b = faces[i], faces[j]
                    if (abs(a.R[:, 2] @ b.R[:, 2]) < 0.98
                            or abs(a.R[:, 2] @ (b.center - a.center)) > 2 * fp.plane_tolerance):
                        continue
                    grown = cv2.dilate(masks[i].astype(np.uint8), kernel).astype(bool)
                    if not (grown & masks[j]).any():
                        continue
                    union = masks[i] | masks[j]
                    face, _ = top_face(backproject(union, depth, K, fp.near, fp.far), fp)
                    if face is None:
                        continue
                    faces[i], masks[i] = face, union
                    del faces[j], masks[j]
                    merges += 1
                    changed = True
                    break
                if changed:
                    break
        return faces, masks, merges

    def _to_target(self, faces, stamp):
        target = self.get_parameter('target_frame').value
        out = []
        for f in faces:
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = self._optical_frame, stamp
            ps.pose = matrix_to_pose(f.matrix())
            tp = self.tf.transform_pose(ps, target)
            if tp is None:
                return None
            out.append(tp)
        return out

    # ── Outputs ──────────────────────────────────────────────────────────────

    def _publish(self, poses, faces, stamp) -> None:
        target = self.get_parameter('target_frame').value
        pa = PoseArray()
        pa.header.frame_id, pa.header.stamp = target, stamp
        pa.poses = [ps.pose for ps in poses]
        self._poses_pub.publish(pa)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for i, (ps, f) in enumerate(zip(poses, faces)):
            cube = Marker()
            cube.header = pa.header
            cube.ns, cube.id, cube.type, cube.action = 'box_top', i, Marker.CUBE, Marker.ADD
            cube.pose = ps.pose
            cube.scale.x, cube.scale.y, cube.scale.z = f.size[0], f.size[1], 0.005
            r, g, b = COLORS[i % len(COLORS)]
            cube.color.r, cube.color.g, cube.color.b, cube.color.a = r / 255, g / 255, b / 255, 0.7
            text = Marker()
            text.header = pa.header
            text.ns, text.id, text.type, text.action = 'box_label', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y = ps.pose.position.x, ps.pose.position.y
            text.pose.position.z = ps.pose.position.z + 0.05
            text.scale.z = 0.04
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'#{i} {f.size[0] * 100:.0f}x{f.size[1] * 100:.0f} cm'
            markers.markers += [cube, text]
        self._markers_pub.publish(markers)

    def _publish_debug(self, rgb, faces, face_masks, K, header) -> None:
        img = rgb.copy()
        for i, (f, seg) in enumerate(zip(faces, face_masks)):
            color = np.array(COLORS[i % len(COLORS)], dtype=np.float32)
            img[seg] = (0.55 * img[seg] + 0.45 * color).astype(np.uint8)
            corners = project(f.corners(), K).round().astype(np.int32)
            cv2.polylines(img, [corners.reshape(-1, 1, 2)], True, tuple(int(c) for c in color), 2)
            c = project(f.center[None], K)[0].round().astype(int)
            cv2.putText(img, f'#{i}', (int(c[0]) - 8, int(c[1]) + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (255, 255, 255), 1, cv2.LINE_AA)
        msg = Image()
        msg.header = header
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = 'rgb8', img.shape[1] * 3
        msg.data = img.tobytes()
        self._debug_pub.publish(msg)


def main():
    rclpy.init()
    node = BoxPoseEstimator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
