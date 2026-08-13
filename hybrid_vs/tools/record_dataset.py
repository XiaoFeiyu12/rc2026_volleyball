#!/usr/bin/env python3
"""一键录制排球检测数据集

启动相机+检测节点，打开 rqt 观察画面，根据检测结果智能抽帧保存图像到 bag/ 目录。

用法:
    python tools/record_dataset.py                          # 完整模式
    python tools/record_dataset.py --no-rqt                 # 不启动 rqt
    python tools/record_dataset.py --no-launch              # 自行手动启动节点
    python tools/record_dataset.py --ball-step 3 --bg-step 60
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from volleyball_interfaces.msg import Ball


class RecorderNode(Node):
    """订阅图像和检测结果，智能抽帧保存"""

    def __init__(
        self,
        output_dir: str,
        image_topic: str,
        ball_topic: str,
        ball_step: int,
        bg_step: int,
        image_format: str,
    ):
        super().__init__("dataset_recorder")

        self.output_dir = output_dir
        self.ball_step = ball_step
        self.bg_step = bg_step
        self.image_format = image_format

        self.bridge = CvBridge()

        # 计数器
        self.ball_counter = 0
        self.bg_counter = 0
        self.ball_saved = 0
        self.bg_saved = 0

        # 有球判定：记录最近一次收到 Ball 消息的时间
        self.last_ball_time = 0.0

        # 订阅
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.ball_sub = self.create_subscription(
            Ball, ball_topic, self.ball_callback, 10
        )

        self.get_logger().info(
            f"录制已就绪 — 输出目录: {output_dir}, "
            f"有球间隔: {ball_step} 帧, 无球间隔: {bg_step} 帧"
        )

    def ball_callback(self, msg: Ball) -> None:
        """记录最近一次检测到球的时间"""
        self.last_ball_time = self.get_clock().now().nanoseconds / 1e9

    def image_callback(self, msg: Image) -> None:
        """图像回调：判断是否有球，按间隔保存"""
        now = self.get_clock().now().nanoseconds / 1e9
        ball_detected = (now - self.last_ball_time) < 0.5  # 0.5s 窗口

        if ball_detected:
            self.ball_counter += 1
            if self.ball_counter % self.ball_step == 0:
                self._save_image(msg, "ball")
                self.ball_saved += 1
        else:
            self.bg_counter += 1
            if self.bg_counter % self.bg_step == 0:
                self._save_image(msg, "bg")
                self.bg_saved += 1

    def _save_image(self, msg: Image, prefix: str) -> None:
        """将 ROS Image 消息保存为图片文件"""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge 转换失败: {e}")
            return

        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nanosec
        filename = f"{prefix}_{sec}_{nsec:09d}.{self.image_format}"
        path = os.path.join(self.output_dir, filename)

        cv2.imwrite(path, cv_img)
        self.get_logger().info(f"[{prefix.upper():>4}] saved {filename}")


# ── 进程管理 ──────────────────────────────────────────────

_child_processes: list[subprocess.Popen] = []


def spawn(cmd: str) -> subprocess.Popen:
    """启动子进程并记录"""
    proc = subprocess.Popen(cmd, shell=True)
    _child_processes.append(proc)
    return proc


def cleanup() -> None:
    """终止所有子进程"""
    for proc in _child_processes:
        if proc.poll() is None:  # 还在运行
            proc.terminate()
    for proc in _child_processes:
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()


def shutdown_handler(signum, frame):
    """Ctrl+C 信号处理"""
    print("\n正在停止...")
    cleanup()
    sys.exit(0)


# ── 主入口 ────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="一键录制排球检测数据集"
    )
    parser.add_argument(
        "--ball-step", type=int, default=5,
        help="有球时每 N 帧保存一张（默认 3）"
    )
    parser.add_argument(
        "--bg-step", type=int, default=25,
        help="无球时每 N 帧保存一张（默认 30）"
    )
    parser.add_argument(
        "--format", type=str, default="jpg", choices=["jpg", "png"],
        help="图片格式（默认 jpg）"
    )
    parser.add_argument(
        "--image-topic", type=str,
        default="/camera/camera/color/image_raw",
        help="图像话题"
    )
    parser.add_argument(
        "--ball-topic", type=str,
        default="/detector/ball",
        help="检测结果话题"
    )
    parser.add_argument(
        "--no-rqt", action="store_true",
        help="不启动 rqt"
    )
    parser.add_argument(
        "--no-launch", action="store_true",
        help="不启动相机和检测节点（自行手动启动）"
    )
    args = parser.parse_args()

    # ── 创建输出目录 ──
    session_name = datetime.now().strftime("record_%Y%m%d_%H%M%S")
    output_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "bag", session_name
    )
    os.makedirs(output_dir, exist_ok=True)
    print(f"输出目录: {output_dir}")

    # ── 信号处理 ──
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    # ── 启动子进程 ──
    if not args.no_launch:
        print(">>> 启动相机 + 检测节点...")
        spawn(
            "ros2 launch volleyball_robot bringup.launch.py"
        )
        # 等节点就绪
        print("等待节点就绪 (5s)...")
        time.sleep(5)

    if not args.no_rqt:
        print(">>> 启动 rqt...")
        spawn("rqt &")

    # ── 启动录制 ──
    rclpy.init()
    node = RecorderNode(
        output_dir=output_dir,
        image_topic=args.image_topic,
        ball_topic=args.ball_topic,
        ball_step=args.ball_step,
        bg_step=args.bg_step,
        image_format=args.format,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cleanup()
        print(f"\n录制完成 — 有球: {node.ball_saved} 张, 无球: {node.bg_saved} 张")
        print(f"图片保存在: {output_dir}")


if __name__ == "__main__":
    main()
