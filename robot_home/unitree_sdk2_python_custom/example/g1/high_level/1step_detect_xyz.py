#!/usr/bin/env python3
"""
Совмещенный просмотр RGB + Depth (ROS 2, RealSense).


pkill -f realsense2_camera

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  align_depth.enable:=true \
  depth_module.profile:="640,480,30" \
  rgb_camera.profile:="640,480,30"

Сохраняет RGB и глубину одновременно по нажатию 's' в dataset_rgbd/:
  {ts}_rgb.jpg
  {ts}_depth_mm.npy
  {ts}_depth_mm.png
  {ts}_depth_preview.jpg
  {ts}_meta.json


Запуск:

python3 1step_detect_xyz.py \
  --detector server \
  --server-host 192.168.0.100 \
  --server-user arutiunyan_ag \
  --server-in-dir /home/arutiunyan_ag/owlv2_test/dataset_targets \
  --server-out-dir /home/arutiunyan_ag/owlv2_test/dataset_results


  source /opt/ros/foxy/setup.bash
  python3 1step_detect_xyz.py
  
  python3 pipeline.py
  
  python3 rgbd_view.py /camera/color/image_raw /camera/aligned_depth_to_color/image_raw

Горячие клавиши:
  q/Esc - выход
  c     - переключить колормап глубины
  s     - сохранить RGB+Depth одним timestamp
"""

import argparse
import json
import os
import posixpath
import sys
import time
from typing import Optional

import cv2
import numpy as np
import paramiko

try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
except ImportError as e:
    print("Нужны ROS 2 и cv_bridge. source /opt/ros/foxy/setup.bash", file=sys.stderr)
    print(e, file=sys.stderr)
    sys.exit(1)


WIN_RGB = "RGB (s=save, q=quit)"
WIN_DEPTH = "Depth (c=colormap, s=save, q=quit)"
WIN_SELECT = "Select Object ROI (Enter=ok, C=cancel)"

DEFAULT_RGB_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
MAX_DEPTH_MM = 3000

# Дефолтные extrinsics камеры относительно пелвиса (можно править под вашу установку).
PELVIS_TX_M = 0.05366
PELVIS_TY_M = 0.01753
PELVIS_TZ_M = 0.47387
PELVIS_ROLL_DEG = 0.0
PELVIS_PITCH_DEG = 50.0
PELVIS_YAW_DEG = 0.0

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) or "."
SAVE_DIR = os.path.join(SCRIPT_DIR, "dataset_rgbd")
os.makedirs(SAVE_DIR, exist_ok=True)
CURRENT_TARGET_PATH = os.path.join(SAVE_DIR, "current_target.txt")

TARGET_DX_M = -0.12
TARGET_DY_RIGHT_M = -0.025
TARGET_DY_LEFT_M = 0.025
TARGET_DZ_M = 0.15

RGB_CAMERA_INFO_TOPIC = "/camera/color/camera_info"
DEPTH_CAMERA_INFO_TOPIC = "/camera/aligned_depth_to_color/camera_info"

RGB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

DEPTH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class ServerDetectorConfig:
    def __init__(
        self,
        host: str,
        port: int,
        user: str,
        password: str,
        remote_in_dir: str,
        remote_out_dir: str,
        poll_sec: float,
        timeout_sec: float,
        key_path: str,
    ):
        self.host = host
        self.port = port
        self.user = user
        self.password = password
        self.remote_in_dir = remote_in_dir
        self.remote_out_dir = remote_out_dir
        self.poll_sec = poll_sec
        self.timeout_sec = timeout_sec
        self.key_path = key_path


def _infer_camera_info_topic(image_topic: str, default_topic: str) -> str:
    if image_topic.endswith("/image_raw"):
        return image_topic[: -len("/image_raw")] + "/camera_info"
    if image_topic.endswith("/image_rect_raw"):
        return image_topic[: -len("/image_rect_raw")] + "/camera_info"
    return default_topic


class RGBDViewer(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        max_depth_mm: int,
        detector_mode: str = "manual",
        server_cfg: Optional[ServerDetectorConfig] = None,
        server_fallback_manual: bool = True,
        target_dx_m: float = TARGET_DX_M,
        target_dz_m: float = TARGET_DZ_M,
        target_dy_m: float = 0.0,
        auto_save_and_exit: bool = False,
        auto_wait_sec: float = 1.0,
    ):
        super().__init__("rgbd_viewer")
        self.bridge = CvBridge()
        self.max_depth_mm = max_depth_mm
        self.use_colormap = True
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.detector_mode = detector_mode
        self.server_cfg = server_cfg
        self.server_fallback_manual = server_fallback_manual
        self.target_dx_m = float(target_dx_m)
        self.target_dz_m = float(target_dz_m)
        self.target_dy_m = float(target_dy_m)
        self.auto_save_and_exit = bool(auto_save_and_exit)
        self.auto_wait_sec = max(0.0, float(auto_wait_sec))
        self._auto_ready_since = None

        self.rgb = None
        self.depth_mm = None
        self.rgb_info = None
        self.depth_info = None

        self.rgb_info_topic = _infer_camera_info_topic(rgb_topic, RGB_CAMERA_INFO_TOPIC)
        self.depth_info_topic = _infer_camera_info_topic(depth_topic, DEPTH_CAMERA_INFO_TOPIC)

        self.create_subscription(Image, rgb_topic, self._cb_rgb, RGB_QOS)
        self.create_subscription(Image, depth_topic, self._cb_depth, DEPTH_QOS)
        self.create_subscription(CameraInfo, self.rgb_info_topic, self._cb_rgb_info, 10)
        self.create_subscription(CameraInfo, self.depth_info_topic, self._cb_depth_info, 10)

        self.get_logger().info(
            f"RGB: {rgb_topic}, Depth: {depth_topic}. "
            f"q/Esc - выход, c - колормап, s - сохранить в {SAVE_DIR}. "
            f"detector={self.detector_mode}, auto_save_and_exit={self.auto_save_and_exit}"
        )

    def _cb_rgb(self, msg: Image):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Ошибка RGB кадра: {e}")

    def _cb_depth(self, msg: Image):
        try:
            if msg.encoding in ("16UC1", "mono16"):
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                self.depth_mm = depth.astype(np.float32)
                return
            if msg.encoding in ("32FC1", "32FC2"):
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                self.depth_mm = depth * 1000.0
                return

            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                self.depth_mm = depth.astype(np.float32)
            elif depth.dtype == np.float32:
                self.depth_mm = depth * 1000.0
            else:
                self.depth_mm = depth.astype(np.float32)
        except Exception as e:
            self.get_logger().warn(f"Ошибка Depth кадра: {e}")

    def _cb_rgb_info(self, msg: CameraInfo):
        self.rgb_info = msg

    def _cb_depth_info(self, msg: CameraInfo):
        self.depth_info = msg

    def _make_depth_vis(self, depth_mm: np.ndarray) -> np.ndarray:
        d = np.clip(depth_mm, 0, self.max_depth_mm)
        vis = (d / self.max_depth_mm * 255).astype(np.uint8)
        if self.use_colormap:
            return cv2.applyColorMap(vis, cv2.COLORMAP_INFERNO)
        return cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)

    def _depth_at(self, depth_mm: np.ndarray, u: float, v: float, win: int = 5) -> float:
        h, w = depth_mm.shape[:2]
        ui = int(round(u))
        vi = int(round(v))
        if 0 <= vi < h and 0 <= ui < w and depth_mm[vi, ui] > 0:
            return float(depth_mm[vi, ui])

        x0 = max(0, ui - win)
        x1 = min(w, ui + win + 1)
        y0 = max(0, vi - win)
        y1 = min(h, vi + win + 1)
        patch = depth_mm[y0:y1, x0:x1]
        vals = patch[np.isfinite(patch) & (patch > 0)]
        if vals.size == 0:
            return float("nan")
        return float(np.median(vals))

    def _cam_to_pelvis(self, x_m: float, y_m: float, z_m: float):
        # СК пелвиса: X вперед, Y влево, Z вверх.
        # Базовое преобразование из камеры: Xp=Zc, Yp=-Xc, Zp=-Yc
        base_r = np.array(
            [
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ],
            dtype=np.float64,
        )

        rr = np.deg2rad(PELVIS_ROLL_DEG)
        rp = np.deg2rad(PELVIS_PITCH_DEG)
        ry = np.deg2rad(PELVIS_YAW_DEG)
        cr, sr = np.cos(rr), np.sin(rr)
        cp, sp = np.cos(rp), np.sin(rp)
        cy, sy = np.cos(ry), np.sin(ry)

        r_rpy = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )
        r = r_rpy @ base_r
        t = np.array([PELVIS_TX_M, PELVIS_TY_M, PELVIS_TZ_M], dtype=np.float64)
        p_cam = np.array([x_m, y_m, z_m], dtype=np.float64)
        p_pelvis = r @ p_cam + t
        return float(p_pelvis[0]), float(p_pelvis[1]), float(p_pelvis[2])

    def _select_bbox(self, rgb: np.ndarray):
        # Блокирующее окно ручной разметки: Enter/Space - подтвердить, C - отмена.
        cv2.namedWindow(WIN_SELECT, cv2.WINDOW_NORMAL)
        x, y, w, h = cv2.selectROI(WIN_SELECT, rgb, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow(WIN_SELECT)
        if w <= 0 or h <= 0:
            return None
        return {"x": int(x), "y": int(y), "w": int(w), "h": int(h)}

    def _ensure_remote_dir(self, sftp: paramiko.SFTPClient, remote_dir: str):
        parts = [p for p in remote_dir.split("/") if p]
        current = "/" if remote_dir.startswith("/") else ""
        for part in parts:
            current = posixpath.join(current, part) if current else part
            try:
                sftp.stat(current)
            except FileNotFoundError:
                sftp.mkdir(current)

    def _select_bbox_server(self, rgb: np.ndarray, ts: str):
        if self.server_cfg is None:
            self.get_logger().warn("Server detector config is not set.")
            return None

        cfg = self.server_cfg
        name = f"{ts}_rgb.jpg"
        remote_img = posixpath.join(cfg.remote_in_dir, name)
        remote_json = posixpath.join(cfg.remote_out_dir, f"{ts}_rgb.json")

        ok, enc = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            self.get_logger().warn("Не удалось закодировать RGB в JPEG для отправки на сервер.")
            return None

        ssh = None
        sftp = None
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            connect_kwargs = dict(
                hostname=cfg.host,
                port=cfg.port,
                username=cfg.user,
                timeout=10,
            )
            if cfg.password:
                connect_kwargs["password"] = cfg.password
            connect_kwargs["allow_agent"] = True
            connect_kwargs["look_for_keys"] = True
            if cfg.key_path:
                connect_kwargs["key_filename"] = cfg.key_path
            ssh.connect(**connect_kwargs)
            sftp = ssh.open_sftp()
            self._ensure_remote_dir(sftp, cfg.remote_in_dir)
            self._ensure_remote_dir(sftp, cfg.remote_out_dir)

            with sftp.file(remote_img, "wb") as f:
                f.write(enc.tobytes())
            self.get_logger().info(f"Отправлен кадр на сервер: {remote_img}")

            deadline = time.time() + cfg.timeout_sec
            while time.time() < deadline:
                try:
                    with sftp.file(remote_json, "rb") as f:
                        data = json.loads(f.read().decode("utf-8"))
                    box = data.get("box_xyxy")
                    if not isinstance(box, list) or len(box) != 4:
                        return None
                    x1, y1, x2, y2 = [int(round(float(v))) for v in box]
                    h, w = rgb.shape[:2]
                    x1 = max(0, min(w - 1, x1))
                    x2 = max(0, min(w - 1, x2))
                    y1 = max(0, min(h - 1, y1))
                    y2 = max(0, min(h - 1, y2))
                    bw = x2 - x1
                    bh = y2 - y1
                    if bw <= 0 or bh <= 0:
                        return None
                    return {"x": x1, "y": y1, "w": bw, "h": bh}
                except FileNotFoundError:
                    time.sleep(cfg.poll_sec)
                    continue
                except json.JSONDecodeError:
                    time.sleep(cfg.poll_sec)
                    continue

            self.get_logger().warn(
                f"Таймаут ожидания результата детекции: {remote_json}"
            )
            return None
        except Exception as e:
            self.get_logger().warn(f"Ошибка server detector: {e}")
            return None
        finally:
            if sftp is not None:
                sftp.close()
            if ssh is not None:
                ssh.close()

    def _save_rgbd(self):
        if self.rgb is None or self.depth_mm is None:
            self.get_logger().warn("Нет RGB или Depth кадра для сохранения.")
            return

        ts = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        base = os.path.join(SAVE_DIR, ts)

        rgb = self.rgb.copy()
        depth = self.depth_mm.copy()

        path_rgb = base + "_rgb.jpg"
        path_depth_npy = base + "_depth_mm.npy"
        path_depth_png = base + "_depth_mm.png"
        path_depth_preview = base + "_depth_preview.jpg"
        path_object_crop = base + "_object_crop.jpg"
        path_xyz_cam_txt = base + "_xyz_cam_m.txt"
        path_xyz_pelvis_txt = base + "_xyz_pelvis_m.txt"
        path_meta = base + "_meta.json"

        cv2.imwrite(path_rgb, rgb)
        np.save(path_depth_npy, depth.astype(np.float32))
        cv2.imwrite(path_depth_png, np.clip(depth, 0, 65535).astype(np.uint16))
        cv2.imwrite(path_depth_preview, self._make_depth_vis(depth))
        if self.detector_mode == "server":
            bbox = self._select_bbox_server(rgb, ts)
            if bbox is None and self.server_fallback_manual:
                self.get_logger().warn("Server bbox не получен, fallback на ручной ROI.")
                bbox = self._select_bbox(rgb)
        else:
            bbox = self._select_bbox(rgb)

        meta = {
            "timestamp": ts,
            "detector_mode": self.detector_mode,
            "rgb_image": path_rgb,
            "depth_npy_mm": path_depth_npy,
            "depth_png_mm": path_depth_png,
            "depth_preview": path_depth_preview,
            "rgb_width": int(rgb.shape[1]),
            "rgb_height": int(rgb.shape[0]),
            "depth_width": int(depth.shape[1]),
            "depth_height": int(depth.shape[0]),
            "depth_unit": "mm",
            "topics": {
                "rgb": self.rgb_topic,
                "depth": self.depth_topic,
                "rgb_info": self.rgb_info_topic,
                "depth_info": self.depth_info_topic,
            },
        }
        if bbox is not None:
            x, y, w, h = bbox["x"], bbox["y"], bbox["w"], bbox["h"]
            crop = rgb[y : y + h, x : x + w]
            if crop.size > 0:
                cv2.imwrite(path_object_crop, crop)
                meta["object_crop"] = path_object_crop
            meta["bbox_xywh"] = bbox

            if self.depth_info is not None and len(self.depth_info.k) >= 9:
                k = self.depth_info.k
                fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
                u = x + w / 2.0
                v = y + h / 2.0
                z_mm = self._depth_at(depth, u, v)
                if np.isfinite(z_mm) and z_mm > 0:
                    x_mm = (u - cx) * z_mm / fx
                    y_mm = (v - cy) * z_mm / fy
                    x_m, y_m, z_m = x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0
                    with open(path_xyz_cam_txt, "w", encoding="utf-8") as f:
                        f.write(f"{x_m:.6f} {y_m:.6f} {z_m:.6f}\n")
                    px, py, pz = self._cam_to_pelvis(x_m, y_m, z_m)
                    with open(path_xyz_pelvis_txt, "w", encoding="utf-8") as f:
                        f.write(f"{px:.6f} {py:.6f} {pz:.6f}\n")
                    target_x = px + self.target_dx_m
                    target_z = pz + self.target_dz_m
                    target_y_right = py + TARGET_DY_RIGHT_M + self.target_dy_m
                    target_y_left = py + TARGET_DY_LEFT_M + self.target_dy_m
                    with open(CURRENT_TARGET_PATH, "w", encoding="utf-8") as f:
                        f.write(f"right {target_x:.6f} {target_y_right:.6f} {target_z:.6f}\n")
                        f.write(f"left {target_x:.6f} {target_y_left:.6f} {target_z:.6f}\n")
                    meta["xyz_cam_m"] = {"x": x_m, "y": y_m, "z": z_m}
                    meta["xyz_cam_txt"] = path_xyz_cam_txt
                    meta["xyz_pelvis_m"] = {"x": px, "y": py, "z": pz}
                    meta["xyz_pelvis_txt"] = path_xyz_pelvis_txt
                    meta["current_target_txt"] = CURRENT_TARGET_PATH
                    meta["pelvis_extrinsics"] = {
                        "tx_m": PELVIS_TX_M,
                        "ty_m": PELVIS_TY_M,
                        "tz_m": PELVIS_TZ_M,
                        "roll_deg": PELVIS_ROLL_DEG,
                        "pitch_deg": PELVIS_PITCH_DEG,
                        "yaw_deg": PELVIS_YAW_DEG,
                    }
                    meta["target_offset_m"] = {
                        "dx": self.target_dx_m,
                        "dy_right": TARGET_DY_RIGHT_M,
                        "dy_left": TARGET_DY_LEFT_M,
                        "dy_user": self.target_dy_m,
                        "dz": self.target_dz_m,
                    }
                    meta["bbox_center_uv"] = {"u": float(u), "v": float(v)}
                else:
                    self.get_logger().warn("Не удалось получить глубину в центре bbox для XYZ.")

        if self.rgb_info is not None and len(self.rgb_info.k) >= 9:
            k = self.rgb_info.k
            meta["rgb_intrinsics"] = {
                "fx": k[0],
                "fy": k[4],
                "cx": k[2],
                "cy": k[5],
                "width": self.rgb_info.width,
                "height": self.rgb_info.height,
            }
        if self.depth_info is not None and len(self.depth_info.k) >= 9:
            k = self.depth_info.k
            meta["depth_intrinsics"] = {
                "fx": k[0],
                "fy": k[4],
                "cx": k[2],
                "cy": k[5],
                "width": self.depth_info.width,
                "height": self.depth_info.height,
            }

        with open(path_meta, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        self.get_logger().info(f"Сохранено RGBD: {base}_*")

    def run(self):
        cv2.namedWindow(WIN_RGB, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_DEPTH, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.rgb is not None:
                cv2.imshow(WIN_RGB, self.rgb)
            if self.depth_mm is not None:
                cv2.imshow(WIN_DEPTH, self._make_depth_vis(self.depth_mm))

            if self.auto_save_and_exit:
                if self.rgb is not None and self.depth_mm is not None:
                    if self._auto_ready_since is None:
                        self._auto_ready_since = time.time()
                    elif time.time() - self._auto_ready_since >= self.auto_wait_sec:
                        self.get_logger().info("Auto mode: save one frame and exit.")
                        self._save_rgbd()
                        break
                cv2.waitKey(1)
                continue

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                self.use_colormap = not self.use_colormap
            if key == ord("s"):
                self._save_rgbd()

        cv2.destroyAllWindows()


def _parse_args():
    parser = argparse.ArgumentParser(description="RGBD viewer + bbox + XYZ to pelvis")
    parser.add_argument("rgb_topic", nargs="?", default=DEFAULT_RGB_TOPIC)
    parser.add_argument("depth_topic", nargs="?", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("max_depth_mm", nargs="?", type=int, default=MAX_DEPTH_MM)

    parser.add_argument("--detector", choices=["manual", "server"], default="manual")
    parser.add_argument("--server-host", default="192.168.0.100")
    parser.add_argument("--server-port", type=int, default=22)
    parser.add_argument("--server-user", default="arutiunyan_ag")
    parser.add_argument(
        "--server-in-dir",
        default="/home/arutiunyan_ag/owlv2_test/dataset_targets",
    )
    parser.add_argument(
        "--server-out-dir",
        default="/home/arutiunyan_ag/owlv2_test/dataset_results",
    )
    parser.add_argument("--server-poll-sec", type=float, default=0.5)
    parser.add_argument("--server-timeout-sec", type=float, default=15.0)
    parser.add_argument(
        "--server-password-env",
        default="UNITREE_SERVER_SSH_PASSWORD",
        help="Environment variable with SSH password; if empty, SSH key/agent auth is used",
    )
    parser.add_argument(
        "--server-key-path",
        default="",
        help="Optional SSH private key path for Paramiko",
    )
    parser.add_argument(
        "--target-dx-m",
        type=float,
        default=TARGET_DX_M,
        help="Additional forward X offset in meters added after CV->pelvis conversion",
    )
    parser.add_argument(
        "--target-dz-m",
        type=float,
        default=TARGET_DZ_M,
        help="Vertical target offset in meters added after CV->pelvis conversion",
    )
    parser.add_argument(
        "--target-dy-m",
        type=float,
        default=0.0,
        help="Additional lateral Y offset in meters added to both left/right targets",
    )
    parser.add_argument(
        "--no-server-fallback-manual",
        action="store_true",
        help="Disable manual ROI fallback when server detection fails",
    )
    parser.add_argument(
        "--auto-save-and-exit",
        action="store_true",
        help="Automatically save one RGBD sample and exit (no manual s/q)",
    )
    parser.add_argument(
        "--auto-wait-sec",
        type=float,
        default=1.0,
        help="Delay after first RGB+Depth frame before auto save in --auto-save-and-exit mode",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    server_cfg = None
    if args.detector == "server":
        password = os.getenv(args.server_password_env, "")
        server_cfg = ServerDetectorConfig(
            host=args.server_host,
            port=args.server_port,
            user=args.server_user,
            password=password,
            remote_in_dir=args.server_in_dir,
            remote_out_dir=args.server_out_dir,
            poll_sec=args.server_poll_sec,
            timeout_sec=args.server_timeout_sec,
            key_path=args.server_key_path,
        )

    rclpy.init()
    node = RGBDViewer(
        args.rgb_topic,
        args.depth_topic,
        args.max_depth_mm,
        detector_mode=args.detector,
        server_cfg=server_cfg,
        server_fallback_manual=not args.no_server_fallback_manual,
        target_dx_m=args.target_dx_m,
        target_dz_m=args.target_dz_m,
        target_dy_m=args.target_dy_m,
        auto_save_and_exit=args.auto_save_and_exit,
        auto_wait_sec=args.auto_wait_sec,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

