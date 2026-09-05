#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point
from gas_source02.srv import GetConcentration, GetConcentrationResponse

class GasConcentrationServer:
    def __init__(self):
        rospy.init_node('gas_concentration_server')
        
        # 真实气源坐标（已知）
        self.sources = [
            {"pos": (-25.0, 15.0, 24.0), "Q": 1000.0},
            {"pos": (30.0, -18.0, 22.0), "Q": 800.0},
            {"pos": (-30.0, -25.0, 12.0), "Q": 600.0}
        ]
        
        # 风场参数
        self.wind_u = rospy.get_param("wind_velocity_x", 3.0)
        self.wind_v = rospy.get_param("wind_velocity_y", 1.5)
        self.wind_speed = math.hypot(self.wind_u, self.wind_v)
        
        # 风向单位向量
        self.wind_dir_vec = np.array([self.wind_u, self.wind_v]) / self.wind_speed
        
        # 扩散参数
        self.sigma_y0 = 2.0      # 初始横向扩散
        self.sigma_z0 = 1.5      # 初始垂直扩散
        self.sigma_y_coeff = 0.25 # 横向扩散系数
        self.sigma_z_coeff = 0.15 # 垂直扩散系数
        
        rospy.Service('/get_concentration', GetConcentration, self.handle_concentration)
        rospy.loginfo("Gas concentration server started")
    
    def gaussian_plume(self, x, y, z, source):
        """高斯烟羽模型"""
        # 相对位置
        dx = x - source["pos"][0]
        dy = y - source["pos"][1]
        dz = z - source["pos"][2]
        
        # 顺风距离（投影到风向）
        downwind = dx * self.wind_dir_vec[0] + dy * self.wind_dir_vec[1]
        
        # 上风向浓度极低
        if downwind <= 0:
            return 0.0
        
        # 横风距离
        crosswind = abs(dy * self.wind_dir_vec[0] - dx * self.wind_dir_vec[1])
        
        # 扩散参数随风距增加
        x_power = downwind ** 0.8
        sigma_y = self.sigma_y0 + self.sigma_y_coeff * x_power
        sigma_z = self.sigma_z0 + self.sigma_z_coeff * x_power
        
        # 源高度
        H = source["pos"][2]
        
        # 浓度计算
        cross_term = math.exp(-crosswind**2 / (2 * sigma_y**2))
        vert_term = math.exp(-(dz)**2 / (2 * sigma_z**2))
        ref_term = math.exp(-(dz + 2*H)**2 / (2 * sigma_z**2))
        
        concentration = source["Q"] / (2 * math.pi * self.wind_speed * sigma_y * sigma_z) * cross_term * (vert_term + ref_term)
        
        return min(concentration, 50.0)  # 限幅
    
    def handle_concentration(self, req):
        """处理浓度请求"""
        total = 0.0
        for source in self.sources:
            total += self.gaussian_plume(req.point.x, req.point.y, req.point.z, source)
        return GetConcentrationResponse(concentration=total)

if __name__ == "__main__":
    GasConcentrationServer()
    rospy.spin()