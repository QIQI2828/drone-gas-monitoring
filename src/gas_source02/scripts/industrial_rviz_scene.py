#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
industrial_rviz_scene.py

功能：
-----------------------------------
将 Gazebo 工业园区同步到 RViz

自动发布：
- 建筑
- 道路
- 储罐
- 烟囱
- 围墙
- 气源点

显示方式：
visualization_msgs/MarkerArray

RViz 效果：
-----------------------------------
完整工业园区 + 气体羽流 + 无人机

作者：
ChatGPT
"""

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class IndustrialScenePublisher:

    def __init__(self):

        rospy.init_node("industrial_scene_rviz")

        self.pub = rospy.Publisher(
            "/industrial_scene",
            MarkerArray,
            queue_size=1
        )

        rospy.loginfo("Industrial RViz Scene Started")

        self.run()

    # =========================================================
    # 创建立方体
    # =========================================================
    def create_cube(self, mid, x, y, z,
                    sx, sy, sz,
                    r, g, b, a=1.0):

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "industrial_scene"

        marker.id = mid

        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    # =========================================================
    # 创建圆柱
    # =========================================================
    def create_cylinder(self, mid, x, y, z,
                        radius, height,
                        r, g, b, a=1.0):

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "industrial_scene"

        marker.id = mid

        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = height

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    # =========================================================
    # 创建球体
    # =========================================================
    def create_sphere(self, mid, x, y, z,
                      radius,
                      r, g, b, a=1.0):

        marker = Marker()

        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "industrial_scene"

        marker.id = mid

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.w = 1.0

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = radius * 2

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    # =========================================================
    # 主场景
    # =========================================================
    def build_scene(self):

        arr = MarkerArray()

        mid = 0

        # =====================================================
        # 围墙
        # =====================================================

        walls = [
            (0, 55, 1.5, 110, 0.6, 3),
            (0, -55, 1.5, 110, 0.6, 3),
            (55, 0, 1.5, 0.6, 110, 3),
            (-55, 0, 1.5, 0.6, 110, 3),
        ]

        for w in walls:
            arr.markers.append(
                self.create_cube(
                    mid,
                    *w,
                    0.3, 0.3, 0.3, 1.0
                )
            )
            mid += 1

        # =====================================================
        # 道路
        # =====================================================

        roads = [
            (0, 0, 0.02, 100, 10, 0.05),
            (0, 0, 0.03, 10, 100, 0.05),
            (-15, 30, 0.03, 40, 7, 0.05),
            (20, -30, 0.03, 35, 6, 0.05),
        ]

        for r in roads:
            arr.markers.append(
                self.create_cube(
                    mid,
                    *r,
                    0.15, 0.15, 0.15, 1.0
                )
            )
            mid += 1

        # =====================================================
        # 主厂房
        # =====================================================

        arr.markers.append(
            self.create_cube(
                mid,
                -30, -25, 4,
                22, 18, 8,
                0.55, 0.55, 0.6, 0.95
            )
        )
        mid += 1

        # 顶塔
        arr.markers.append(
            self.create_cube(
                mid,
                -30, -25, 8,
                5, 5, 3,
                0.65, 0.65, 0.7, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 办公楼
        # =====================================================

        arr.markers.append(
            self.create_cube(
                mid,
                -40, 35, 3,
                12, 12, 6,
                0.85, 0.85, 0.8, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 仓库
        # =====================================================

        arr.markers.append(
            self.create_cube(
                mid,
                35, 25, 3,
                24, 16, 6,
                0.4, 0.55, 0.65, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 精馏塔
        # =====================================================

        arr.markers.append(
            self.create_cylinder(
                mid,
                20, -20, 8,
                2.5, 16,
                0.7, 0.7, 0.75, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 冷却塔
        # =====================================================

        arr.markers.append(
            self.create_cylinder(
                mid,
                15, -35, 7,
                4.5, 14,
                0.5, 0.5, 0.5, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 储罐
        # =====================================================

        arr.markers.append(
            self.create_cylinder(
                mid,
                -45, -40, 2.5,
                3.0, 5,
                0.6, 0.6, 0.65, 0.95
            )
        )
        mid += 1

        arr.markers.append(
            self.create_cylinder(
                mid,
                45, 40, 2.8,
                2.6, 5.6,
                0.62, 0.62, 0.67, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 球罐
        # =====================================================

        arr.markers.append(
            self.create_sphere(
                mid,
                -15, -35, 3.5,
                3.5,
                0.55, 0.55, 0.6, 0.95
            )
        )
        mid += 1

        # =====================================================
        # 烟囱
        # =====================================================

        arr.markers.append(
            self.create_cylinder(
                mid,
                -25, 15, 12,
                1.6, 24,
                0.35, 0.35, 0.38, 1.0
            )
        )
        mid += 1

        arr.markers.append(
            self.create_cylinder(
                mid,
                30, -18, 11,
                1.4, 22,
                0.35, 0.35, 0.38, 1.0
            )
        )
        mid += 1

        return arr

    # =========================================================
    # 循环发布
    # =========================================================
    def run(self):

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            scene = self.build_scene()

            self.pub.publish(scene)

            rate.sleep()


# =============================================================
# main
# =============================================================
if __name__ == "__main__":

    try:
        IndustrialScenePublisher()

    except rospy.ROSInterruptException:
        pass