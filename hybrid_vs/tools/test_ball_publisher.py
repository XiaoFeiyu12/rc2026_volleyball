#!/usr/bin/env python3
"""排球对攻测试 —— 带线性阻力，匹配LinearDrag3D模型"""

import rclpy
from rclpy.node import Node
from volleyball_interfaces.msg import Ball
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

class BallSimulator(Node):
    def __init__(self):
        super().__init__('ball_simulator')
        self.pub = self.create_publisher(Ball, '/detector/ball', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/test/ground_truth', 10)
        self.freq = 60.0
        self.dt = 1.0 / self.freq
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.g = 9.81
        self.noise_std = 0.03

        # 阻力参数，匹配tracker的 k=0.20, m=0.27
        self.beta = 0.20 / 0.27  # ≈ 0.74

        # 排球场: 18m × 9m, 网在 x=9m
        # 机器人原点 (0,0,0), X+ 指向对方
        # 己方半场 x∈[0,9], 对方半场 x∈[9,18]
        # 每个回合独立设置起点, 球始终不落地
        self.phases = [
            # ① 对方发球: 从 x=12 发向我方, vx=-5, 飞行1.5s
            dict(px=12.0, py=-2.5, pz=2.5, vx=-5.0, vy=0.0, vz=2.5, dur=1.50, name='①对方发球'),
            # ② 我方一传: 从 x=8 垫回对方, vx=+5, 飞行1.2s
            dict(px=8.0,  py=-0.0, pz=1.5, vx=5.0,  vy=0.0, vz=2.0, dur=1.20, name='②我方一传'),
            # ③ 对方回球: 从 x=12 打回我方, vx=-5, 飞行1.2s
            dict(px=12.0, py=-2.0, pz=2.0, vx=-5.0, vy=0.0, vz=2.0, dur=1.20, name='③对方回球'),
            # ④ 我方扣球: 从 x=8 扣向对方, vx=+7, 飞行0.8s
            dict(px=8.0,  py=-0.0, pz=1.8, vx=7.0,  vy=0.0, vz=1.5, dur=0.80, name='④我方扣球'),
            # ⑤ 对方救球: 从 x=13 高弧度吊回, vx=-3, 飞行1.2s
            dict(px=13.0, py=-2.0, pz=1.0, vx=-3.0, vy=0.0, vz=4.0, dur=1.20, name='⑤对方救球'),
            # ⑥ 我方回击: 从 x=9 再打回对方, vx=+5, 飞行0.8s
            dict(px=9.0,  py=-0.0, pz=1.5, vx=5.0,  vy=0.0, vz=2.0, dur=0.80, name='⑥我方回击'),
            # ⑦ 对方扣杀落地: 从 x=13 扣杀, vx=-8, 飞行0.8s
            dict(px=13.0, py=-2.0, pz=2.0, vx=-8.0, vy=0.0, vz=1.0, dur=0.80, name='⑦对方扣杀'),
        ]

        self.drop_prob = 0.0             # 关闭随机丢帧（调试用）
        self.min_drop = 2                # 每次最少连续丢2帧
        self.max_drop = 8                # 每次最多连续丢8帧
        self.drop_remaining = 0          # 当前剩余丢帧计数
        for i, ph in enumerate(self.phases):
            t = ph['dur']
            end_px, end_pz, end_vx, end_vz = self.drag_pos_vel(
                ph['px'], ph['pz'], ph['vx'], ph['vz'], t)
            ph['end_px'] = end_px
            ph['end_pz'] = end_pz
            ph['end_vx'] = end_vx
            ph['end_vz'] = end_vz
            ph['end_py'] = ph['py'] + ph['vy'] / self.beta * (1 - np.exp(-self.beta * t))
            ph['end_vy'] = ph['vy'] * np.exp(-self.beta * t)

            # 前几回合：球不落地，继续飞；最后回合可落地
            if i + 1 < len(self.phases):
                self.phases[i+1]['px'] = ph['end_px']
                self.phases[i+1]['py'] = ph['end_py']
                # 击球瞬间球被垫高/扣起，至少 z=0.5m
                self.phases[i+1]['pz'] = max(ph['end_pz'], 0.5)

            self.get_logger().info(
                f"{ph['name']}: ({ph['px']:.1f},{ph['pz']:.1f})"
                f" -> ({ph['end_px']:.1f},{ph['end_pz']:.1f}) {t:.1f}s"
                f"  v=({ph['vx']:.1f},{ph['vz']:.1f})"
            )

            # 计算该回合的落点（如果球在该回合内落地）
            ph['landing_x'] = None
            ph['landing_y'] = 0.0
            if end_pz < 0.0:
                t_land = self.find_landing_time(ph['pz'], ph['vz'])
                ph['landing_x'] = ph['px'] + ph['vx'] / self.beta * (1 - np.exp(-self.beta * t_land))
                ph['landing_y'] = ph['py'] + ph['vy'] / self.beta * (1 - np.exp(-self.beta * t_land))
                self.get_logger().info(
                    f"  ↳ 落点: x={ph['landing_x']:.1f}, y={ph['landing_y']:.1f}, t={t_land:.2f}s"
                )

        self.start_time = self.get_clock().now()
        self.frame = 0

    def find_landing_time(self, pz, vz, max_t=10.0):
        """二分法求落地时间 z(t) = 0"""
        vz_inf = self.g / self.beta
        def z(t):
            ebt = np.exp(-self.beta * t)
            return pz + (vz + vz_inf) / self.beta * (1 - ebt) - vz_inf * t
        lo, hi = 0.0, max_t
        for _ in range(50):
            mid = (lo + hi) / 2.0
            if z(mid) > 0:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2.0

    def drag_pos_vel(self, px, pz, vx, vz, t):
        """线性阻力抛体：水平有阻力，垂直有阻力+重力"""
        ebt = np.exp(-self.beta * t)
        # 水平
        x = px + vx / self.beta * (1 - ebt)
        vx_out = vx * ebt
        # 垂直（含重力）
        vz_inf = self.g / self.beta
        z = pz + (vz + vz_inf) / self.beta * (1 - ebt) - vz_inf * t
        vz_out = (vz + vz_inf) * ebt - vz_inf
        return x, z, vx_out, vz_out

    def timer_callback(self):
        self.frame += 1
        t_total = (self.frame - 1) * self.dt

        # ---- 随机丢帧：模拟检测器偶尔漏检 ----
        if self.drop_remaining > 0:
            self.drop_remaining -= 1
            return

        # 随机触发丢帧（不在丢帧期间内再次触发）
        if np.random.random() < self.drop_prob:
            self.drop_remaining = np.random.randint(self.min_drop, self.max_drop + 1)
            self.get_logger().info(
                f'[{t_total:.1f}s] 随机丢检 {self.drop_remaining} 帧'
            )
            self.drop_remaining -= 1  # 本帧也算
            return

        phase_t = t_total
        current_phase = None
        for ph in self.phases:
            if phase_t < ph['dur']:
                current_phase = ph
                break
            phase_t -= ph['dur']

        if current_phase is None:
            # 所有回合结束，停止发布
            return
        else:
            t = phase_t
            px, pz, vx, vz = self.drag_pos_vel(
                current_phase['px'], current_phase['pz'],
                current_phase['vx'], current_phase['vz'], t)
            py = current_phase['py'] + current_phase['vy'] / self.beta * (1 - np.exp(-self.beta * t))
            vy = current_phase['vy'] * np.exp(-self.beta * t)
            if pz < 0:
                pz = 0.0
            if t < self.dt:
                self.get_logger().info(
                    f'[{t_total:.1f}s] {current_phase["name"]}! '
                    f'({px:.1f},{pz:.1f}) v=({vx:.1f},{vz:.1f})'
                )

        px += np.random.normal(0, self.noise_std)
        py += np.random.normal(0, self.noise_std)
        pz += np.random.normal(0, self.noise_std * 2)

        msg = Ball()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.x, msg.y, msg.z = float(px), float(py), float(pz)
        msg.vx, msg.vy, msg.vz = float(vx), float(vy), float(vz)
        self.pub.publish(msg)

        # ---- 可视化：位置球 + 速度方向箭头 ----
        markers = MarkerArray()

        # 位置球
        pos_marker = Marker()
        pos_marker.header.stamp = msg.header.stamp
        pos_marker.header.frame_id = 'odom'
        pos_marker.ns = 'ground_truth'
        pos_marker.id = 0
        pos_marker.type = Marker.SPHERE
        pos_marker.action = Marker.ADD
        pos_marker.pose.position.x = float(px)
        pos_marker.pose.position.y = float(py)
        pos_marker.pose.position.z = float(pz)
        pos_marker.scale.x = pos_marker.scale.y = pos_marker.scale.z = 0.21
        pos_marker.color.a = 0.6
        pos_marker.color.r = 1.0
        pos_marker.lifetime.sec = 1
        markers.markers.append(pos_marker)

        # 速度方向箭头
        speed = math.sqrt(vx*vx + vy*vy + vz*vz)
        if speed > 0.01:
            arrow_marker = Marker()
            arrow_marker.header.stamp = msg.header.stamp
            arrow_marker.header.frame_id = 'odom'
            arrow_marker.ns = 'ground_truth'
            arrow_marker.id = 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            # 箭头起点 = 球位置
            start_pt = Point()
            start_pt.x = float(px)
            start_pt.y = float(py)
            start_pt.z = float(pz)
            # 箭头终点 = 球位置 + 速度方向（缩放2m）
            end_pt = Point()
            scale = min(2.0, speed * 0.3)  # 最大2m，最小按速度比例
            end_pt.x = float(px + vx/speed * scale)
            end_pt.y = float(py + vy/speed * scale)
            end_pt.z = float(pz + vz/speed * scale)
            arrow_marker.points = [start_pt, end_pt]
            arrow_marker.scale.x = 0.05  # 箭杆直径
            arrow_marker.scale.y = 0.10  # 箭头宽度
            arrow_marker.scale.z = 0.15  # 箭头长度
            arrow_marker.color.a = 0.8
            arrow_marker.color.g = 1.0
            arrow_marker.lifetime.sec = 1
            markers.markers.append(arrow_marker)

        self.marker_pub.publish(markers)

def main():
    rclpy.init()
    rclpy.spin(BallSimulator())

if __name__ == '__main__':
    main()
