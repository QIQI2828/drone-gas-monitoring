#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
高级动态羽流气体可视化（真实 plume 版本）

功能：
1. PointCloud2 高性能显示
2. 工业羽流 plume 效果
3. 多气源
4. 风场漂移
5. 动态湍流
6. 高浓度红色核心
7. 横向扩散
8. RViz 热力图显示

RViz:
Add -> PointCloud2
Topic: /gas_cloud
Fixed Frame: world

显示参数：
Style = Flat Squares
Size = 6~10
Color Transformer = RGB8
"""

import rospy
import random
import math
import struct

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import sensor_msgs.point_cloud2 as pc2


class AdvancedGasVisualizer:

    def __init__(self):

        rospy.init_node("advanced_gas_visualizer")

        self.pub = rospy.Publisher(
            "/gas_cloud",
            PointCloud2,
            queue_size=1
        )

        # =========================
        # 风场参数（匹配服务器）
        # =========================

        self.wind_x = rospy.get_param("~wind_x", 3.0)
        self.wind_y = rospy.get_param("~wind_y", 1.5)
        self.wind_speed = math.hypot(self.wind_x, self.wind_y)
        
        # 风向角
        self.wind_angle = math.atan2(self.wind_y, self.wind_x)

        # =========================
        # 扩散参数（匹配服务器）
        # =========================
        self.sigma_y0 = 2.0      # 初始横向扩散
        self.sigma_z0 = 1.5      # 初始垂直扩散
        self.sigma_y_coeff = 0.25 # 横向扩散系数
        self.sigma_z_coeff = 0.15 # 垂直扩散系数

        # =========================
        # 气源（匹配服务器）
        # (x,y,z,strength)
        # =========================

        self.sources = [

            (-25, 15, 24, 1.0),
            (30, -18, 22, 0.8),
            (-30, -25, 11, 0.6),

        ]

        self.t = 0.0

        rospy.loginfo("Advanced Gas Visualizer Started")
        rospy.loginfo(f"扩散参数: sigma_y0={self.sigma_y0}, sigma_z0={self.sigma_z0}")
        rospy.loginfo(f"扩散系数: sigma_y_coeff={self.sigma_y_coeff}, sigma_z_coeff={self.sigma_z_coeff}")

        self.run()

    # =====================================================
    # 浓度颜色映射
    # =====================================================

    def concentration_to_rgb(self, c):

        c = max(0.0, min(c, 1.0))

        # 蓝 -> 青
        if c < 0.2:

            r = 0
            g = int(255 * c * 5)
            b = 255

        # 青 -> 黄
        elif c < 0.5:

            ratio = (c - 0.2) / 0.3

            r = int(255 * ratio)
            g = 255
            b = int(255 * (1.0 - ratio))

        # 黄 -> 红
        else:

            ratio = (c - 0.5) / 0.5

            r = 255
            g = int(255 * (1.0 - ratio))
            b = 0

        return r, g, b

    # =====================================================
    # 创建羽流点云
    # =====================================================

    def create_cloud(self):

        points = []

        # 风场摆动（真实感）
        current_wind_angle = self.wind_angle + math.sin(self.t * 0.15) * 0.15

        # =================================================
        # 每个气源生成 plume
        # =================================================

        for source in self.sources:

            sx, sy, sz, strength = source

            # 粒子数量
            for _ in range(5000):

                # =========================================
                # 沿风向距离（指数分布，近处粒子多）
                # =========================================

                d = random.expovariate(1.0 / 30.0)
                d = min(d, 80.0)

                # =========================================
                # 计算扩散参数（匹配服务器公式）
                # 使用幂律：sigma = sigma0 + coeff * d^0.8
                # =========================================
                
                d_power = d ** 0.8
                sigma_y = self.sigma_y0 + self.sigma_y_coeff * d_power
                sigma_z = self.sigma_z0 + self.sigma_z_coeff * d_power

                # =========================================
                # 横向扩散（高斯分布，标准差=sigma_y）
                # =========================================

                lateral = random.gauss(0, sigma_y)

                # =========================================
                # 垂直扩散（高斯分布，标准差=sigma_z）
                # =========================================

                vertical = random.gauss(0, sigma_z)

                # =========================================
                # 主方向（沿风向）
                # =========================================

                x = sx + d * math.cos(current_wind_angle)
                y = sy + d * math.sin(current_wind_angle)
                z = sz + vertical

                # =========================================
                # 横向偏移
                # =========================================

                x += lateral * math.cos(
                    current_wind_angle + math.pi / 2.0
                )

                y += lateral * math.sin(
                    current_wind_angle + math.pi / 2.0
                )

                # =========================================
                # 湍流扰动（保持不变）
                # =========================================

                x += math.sin(
                    self.t + y * 0.08
                ) * 0.4

                y += math.cos(
                    self.t + x * 0.08
                ) * 0.4

                z += math.sin(
                    self.t + x * 0.05
                ) * 0.15

                # =========================================
                # 浓度衰减（基于高斯公式）
                # =========================================
                
                # 计算理论浓度（用于颜色）
                cross_term = math.exp(-lateral**2 / (2 * sigma_y**2))
                vert_term = math.exp(-vertical**2 / (2 * sigma_z**2))
                
                # 简化的浓度计算（用于可视化）
                concentration = strength * cross_term * vert_term * math.exp(-d / 40.0)

                if concentration < 0.015:
                    continue

                # =========================================
                # RGB颜色
                # =========================================

                r, g, b = self.concentration_to_rgb(
                    concentration
                )

                # =========================================
                # Alpha透明度
                # =========================================

                alpha = int(
                    min(
                        255,
                        40 + concentration * 215
                    )
                )

                # ROS PointCloud2 RGB打包
                rgb = struct.unpack(
                    'I',
                    struct.pack(
                        'BBBB',
                        b,
                        g,
                        r,
                        alpha
                    )
                )[0]

                points.append([
                    x,
                    y,
                    z,
                    rgb
                ])

        # =================================================
        # PointCloud2 Header
        # =================================================

        header = Header()

        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        fields = [

            PointField(
                'x',
                0,
                PointField.FLOAT32,
                1
            ),

            PointField(
                'y',
                4,
                PointField.FLOAT32,
                1
            ),

            PointField(
                'z',
                8,
                PointField.FLOAT32,
                1
            ),

            PointField(
                'rgb',
                12,
                PointField.UINT32,
                1
            )
        ]

        # =================================================
        # 正确创建 PointCloud2
        # =================================================

        cloud = pc2.create_cloud(
            header,
            fields,
            points
        )

        return cloud

    # =====================================================
    # 主循环
    # =====================================================

    def run(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            cloud = self.create_cloud()

            self.pub.publish(cloud)

            self.t += 0.08

            rate.sleep()


# =========================================================
# Main
# =========================================================

if __name__ == "__main__":

    try:

        AdvancedGasVisualizer()

    except rospy.ROSInterruptException:

        pass