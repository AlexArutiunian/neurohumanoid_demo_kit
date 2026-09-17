#!/usr/bin/env python3
"""
Capture RGB+Depth from ROS2 topics and upload snapshot bundle to server.

Saves one timestamped bundle (same naming style as dataset_rgbd):
  {ts}_rgb.jpg
  {ts}_depth_mm.npy
  {ts}_depth_mm.png
  {ts}_depth_preview.jpg
  {ts}_meta.json

Hotkeys:
  s - save/upload one bundle
  a - toggle auto capture mode
  c - toggle depth colormap
  q/Esc - quit
"""

import argparse
import io
import json
import posixpath
import sys
import time
from getpass import getpass

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

DEFAULT_RGB_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
MAX_DEPTH_MM = 3000

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


def _infer_camera_info_topic(image_topic: str, default_topic: str) -> str:
    if image_topic.endswith("/image_raw"):
        return image_topic[: -len("/image_raw")] + "/camera_info"
    if image_topic.endswith("/image_rect_raw"):
        return image_topic[: -len("/image_rect_raw")] + "/camera_info"
    return default_topic


class RGBDServerSaver(Node):
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        max_depth_mm: int,
        ssh_host: str,
        ssh_port: int,
        ssh_user: str,
        ssh_password: str,
        remote_dir: str,
        auto_interval_sec: float,
        auto_start: bool,
    ):
        super().__init__("rgbd_server_saver")
        self.bridge = CvBridge()
        self.max_depth_mm = max_depth_mm
        self.use_colormap = True
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.remote_dir = remote_dir
        self.auto_interval_sec = max(0.01, float(auto_interval_sec))
        self.auto_enabled = bool(auto_start)
        self._last_auto_ts = 0.0

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

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname=ssh_host,
            port=ssh_port,
            username=ssh_user,
            password=ssh_password,
            timeout=10,
        )
        self.sftp = self.ssh.open_sftp()
        self._ensure_remote_dir(self.remote_dir)

        self.get_logger().info(
            f"RGB: {rgb_topic}, Depth: {depth_topic}. "
            f"q/Esc - выход, c - колормап, s - 1 кадр, a - авто "
            f"({self.auto_interval_sec:.2f}s), dir: {self.remote_dir}"
        )

    def close(self):
        try:
            self.sftp.close()
        finally:
            self.ssh.close()

    def _ensure_remote_dir(self, remote_dir: str):
        parts = [p for p in remote_dir.split("/") if p]
        current = "/" if remote_dir.startswith("/") else ""
        for part in parts:
            current = posixpath.join(current, part) if current else part
            try:
                self.sftp.stat(current)
            except FileNotFoundError:
                self.sftp.mkdir(current)

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

    def _write_remote_bytes(self, remote_path: str, payload: bytes):
        with self.sftp.file(remote_path, "wb") as f:
            f.write(payload)

    def _save_bundle_to_server(self):
        if self.rgb is None or self.depth_mm is None:
            self.get_logger().warn("Нет RGB или Depth кадра для сохранения.")
            return

        ts = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        rgb = self.rgb.copy()
        depth = self.depth_mm.copy()
        depth_preview = self._make_depth_vis(depth)

        rgb_name = f"{ts}_rgb.jpg"
        depth_npy_name = f"{ts}_depth_mm.npy"
        depth_png_name = f"{ts}_depth_mm.png"
        depth_preview_name = f"{ts}_depth_preview.jpg"
        meta_name = f"{ts}_meta.json"

        rgb_remote = posixpath.join(self.remote_dir, rgb_name)
        depth_npy_remote = posixpath.join(self.remote_dir, depth_npy_name)
        depth_png_remote = posixpath.join(self.remote_dir, depth_png_name)
        depth_preview_remote = posixpath.join(self.remote_dir, depth_preview_name)
        meta_remote = posixpath.join(self.remote_dir, meta_name)

        ok_rgb, rgb_enc = cv2.imencode(".jpg", rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        ok_png, depth_png_enc = cv2.imencode(
            ".png", np.clip(depth, 0, 65535).astype(np.uint16)
        )
        ok_prev, depth_prev_enc = cv2.imencode(
            ".jpg", depth_preview, [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        )
        if not (ok_rgb and ok_png and ok_prev):
            self.get_logger().warn("Не удалось закодировать один из файлов (rgb/depth/png/preview).")
            return

        npy_buf = io.BytesIO()
        np.save(npy_buf, depth.astype(np.float32))
        npy_bytes = npy_buf.getvalue()

        meta = {
            "timestamp": ts,
            "rgb_image": rgb_remote,
            "depth_npy_mm": depth_npy_remote,
            "depth_png_mm": depth_png_remote,
            "depth_preview": depth_preview_remote,
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

        self._write_remote_bytes(rgb_remote, rgb_enc.tobytes())
        self._write_remote_bytes(depth_npy_remote, npy_bytes)
        self._write_remote_bytes(depth_png_remote, depth_png_enc.tobytes())
        self._write_remote_bytes(depth_preview_remote, depth_prev_enc.tobytes())
        self._write_remote_bytes(
            meta_remote, json.dumps(meta, ensure_ascii=False, indent=2).encode("utf-8")
        )

        self.get_logger().info(f"Сохранено на сервер: {self.remote_dir}/{ts}_*")

    def run(self):
        cv2.namedWindow(WIN_RGB, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WIN_DEPTH, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.rgb is not None:
                cv2.imshow(WIN_RGB, self.rgb)
            if self.depth_mm is not None:
                cv2.imshow(WIN_DEPTH, self._make_depth_vis(self.depth_mm))

            if self.auto_enabled:
                now = time.time()
                if (now - self._last_auto_ts) >= self.auto_interval_sec:
                    self._save_bundle_to_server()
                    self._last_auto_ts = now

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                self.use_colormap = not self.use_colormap
            if key == ord("s"):
                self._save_bundle_to_server()
            if key == ord("a"):
                self.auto_enabled = not self.auto_enabled
                state = "ON" if self.auto_enabled else "OFF"
                self.get_logger().info(f"Auto capture: {state} ({self.auto_interval_sec:.2f}s)")
                self._last_auto_ts = 0.0

        cv2.destroyAllWindows()


def _parse_args():
    parser = argparse.ArgumentParser(description="Save full RGBD bundle to server (no detection)")
    parser.add_argument("rgb_topic", nargs="?", default=DEFAULT_RGB_TOPIC)
    parser.add_argument("depth_topic", nargs="?", default=DEFAULT_DEPTH_TOPIC)
    parser.add_argument("max_depth_mm", nargs="?", type=int, default=MAX_DEPTH_MM)

    parser.add_argument("--server-host", default="192.168.0.100")
    parser.add_argument("--server-port", type=int, default=22)
    parser.add_argument("--server-user", default="arutiunyan_ag")
    parser.add_argument(
        "--server-dir",
        default="/home/arutiunyan_ag/owlv2_test/dataset_rgbd_raw",
        help="Remote folder for timestamped RGBD bundles",
    )
    parser.add_argument(
        "--auto-interval-sec",
        type=float,
        default=0.1,
        help="Interval between auto-captured frames in seconds",
    )
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start in auto capture mode immediately",
    )
    return parser.parse_args()


def main():
    args = _parse_args()
    password = getpass(f"SSH password for {args.server_user}@{args.server_host}: ")

    rclpy.init()
    node = RGBDServerSaver(
        rgb_topic=args.rgb_topic,
        depth_topic=args.depth_topic,
        max_depth_mm=args.max_depth_mm,
        ssh_host=args.server_host,
        ssh_port=args.server_port,
        ssh_user=args.server_user,
        ssh_password=password,
        remote_dir=args.server_dir,
        auto_interval_sec=args.auto_interval_sec,
        auto_start=args.auto_start,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
