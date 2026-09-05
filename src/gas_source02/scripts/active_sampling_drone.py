#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uav_plume_inversion.py

=========================================================
基于高斯烟羽模型（幂率扩散系数）的无人机采样与浓度反演
=========================================================
"""

import rospy
import math
import numpy as np
from scipy.optimize import minimize

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelStates
from gas_source02.srv import GetConcentration, GetConcentrationRequest

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray


class PlumeInversion:
    def __init__(self):
        rospy.init_node("uav_plume_inversion")
        
        # 检查是否使用模拟时间
        self.sim_time = rospy.get_param("/use_sim_time", False)
        if self.sim_time:
            rospy.loginfo("使用模拟时间模式")
        
        # 服务客户端
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/get_concentration")
        
        self.set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_conc_srv = rospy.ServiceProxy("/get_concentration", GetConcentration)
        
        # 无人机参数
        self.drone_model = "firefly"
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 15.0
        
        self.x_min, self.x_max = -80, 80
        self.y_min, self.y_max = -80, 80
        self.z_min, self.z_max = 8, 35
        
        # 已知气源位置
        self.sources = [
            {"id": 0, "x": -25.0, "y": 15.0, "z": 24.0, "name": "高架源A", "Q": None},
            {"id": 1, "x": 30.0, "y": -18.0, "z": 22.0, "name": "高架源B", "Q": None},
            {"id": 2, "x": -30.0, "y": -25.0, "z": 12.0, "name": "低矮源C", "Q": None},
        ]
        
        self.n_sources = len(self.sources)
        
        # 风场参数
        self.update_wind_parameters()
        
        # 扩散参数（幂率）
        self.sigma_y0 = 2.0
        self.sigma_z0 = 1.5
        self.sigma_y_coeff = 0.25
        self.sigma_z_coeff = 0.15
        self.power = 0.8
        
        # 采样参数
        self.downwind_distances = [3, 5, 8, 10, 12, 15, 20, 25, 30]
        self.cross_length = 35.0
        self.cross_steps = 13
        self.z_offsets = [-2, 0, 2]
        
        # 数据存储
        self.measurements = []
        self.source_data = {}
        self.trajectory_points = []  # 存储轨迹点用于可视化
        for src in self.sources:
            self.source_data[src['id']] = []
        
        # 发布器
        self.marker_pub = rospy.Publisher("/plume_markers", MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher("/drone_path", Path, queue_size=10)
        self.result_pub = rospy.Publisher("/inversion_result", Float64MultiArray, queue_size=10)
        
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_callback)
        
        rospy.loginfo("无人机羽流反演系统启动")
        self.run()
    
    def get_rostime(self):
        """获取ROS时间（支持模拟时间）"""
        if self.sim_time:
            return rospy.Time.now()
        else:
            return rospy.get_rostime()
    
    def update_wind_parameters(self):
        """更新风场参数"""
        self.wind_u = rospy.get_param("wind_velocity_x", 3.0)
        self.wind_v = rospy.get_param("wind_velocity_y", 1.5)
        self.wind_speed = math.hypot(self.wind_u, self.wind_v)
        self.wind_dir = math.atan2(self.wind_v, self.wind_u)
        
        self.wind_vec = np.array([self.wind_u, self.wind_v]) / (self.wind_speed + 1e-6)
        self.cross_vec = np.array([-self.wind_vec[1], self.wind_vec[0]])
    
    def pose_callback(self, msg):
        try:
            idx = msg.name.index(self.drone_model)
            p = msg.pose[idx].position
            self.current_x, self.current_y, self.current_z = p.x, p.y, p.z
            
            # 记录轨迹点
            self.trajectory_points.append((self.current_x, self.current_y, self.current_z))
            # 保留最近1000个点
            if len(self.trajectory_points) > 1000:
                self.trajectory_points.pop(0)
            
            # 实时发布路径
            self.publish_path()
        except:
            pass
    
    def publish_path(self):
        """发布无人机飞行路径"""
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_rostime()
        
        for point in self.trajectory_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
    
    def get_concentration(self, x, y, z):
        req = GetConcentrationRequest()
        req.point = Point(x, y, z)
        try:
            return self.get_conc_srv(req).concentration
        except:
            return 0.0
    
    def move_to(self, tx, ty, tz, duration=0.3):
        req = SetModelStateRequest()
        req.model_state.model_name = self.drone_model
        req.model_state.pose.position.x = tx
        req.model_state.pose.position.y = ty
        req.model_state.pose.position.z = tz
        req.model_state.pose.orientation.w = 1.0
        req.model_state.reference_frame = "world"
        
        try:
            self.set_state_srv(req)
            rospy.sleep(duration)
            return True
        except:
            return False
    
    def get_sigma(self, downwind):
        if downwind <= 0:
            return self.sigma_y0, self.sigma_z0
        x_power = downwind ** self.power
        sigma_y = self.sigma_y0 + self.sigma_y_coeff * x_power
        sigma_z = self.sigma_z0 + self.sigma_z_coeff * x_power
        return sigma_y, sigma_z
    
    def gaussian_plume_concentration(self, downwind, crosswind, z, H, Q):
        if downwind <= 0:
            return 0.0
        
        sigma_y, sigma_z = self.get_sigma(downwind)
        
        cross_term = math.exp(-crosswind**2 / (2 * sigma_y**2))
        dz = z - H
        vert_term = math.exp(-dz**2 / (2 * sigma_z**2))
        ref_term = math.exp(-(dz + 2*H)**2 / (2 * sigma_z**2))
        
        concentration = Q / (2 * math.pi * self.wind_speed * sigma_y * sigma_z) * cross_term * (vert_term + ref_term)
        return concentration
    
    def calculate_cross_section_points(self, source, downwind_dist):
        points = []
        center_x = source['x'] + self.wind_vec[0] * downwind_dist
        center_y = source['y'] + self.wind_vec[1] * downwind_dist
        half_length = self.cross_length / 2
        
        for z_offset in self.z_offsets:
            z = source['z'] + z_offset
            z = max(self.z_min, min(self.z_max, z))
            
            for step in range(self.cross_steps + 1):
                t = step / self.cross_steps
                cross_offset = -half_length + t * self.cross_length
                
                x = center_x + self.cross_vec[0] * cross_offset
                y = center_y + self.cross_vec[1] * cross_offset
                
                points.append({
                    'x': x, 'y': y, 'z': z,
                    'downwind': downwind_dist,
                    'cross_offset': cross_offset
                })
        return points
    
    def collect_measurements(self, source):
        all_measurements = []
        
        # 源点正上方采样
        source_top_z = source['z'] + 2.0
        self.move_to(source['x'], source['y'], source_top_z, duration=1.0)
        base_conc = self.get_concentration(source['x'], source['y'], source_top_z)
        all_measurements.append({
            'x': source['x'], 'y': source['y'], 'z': source_top_z,
            'downwind': 0, 'cross_offset': 0, 'conc': base_conc
        })
        
        # 横切采样
        for dist in self.downwind_distances:
            points = self.calculate_cross_section_points(source, dist)
            
            for pt in points:
                self.move_to(pt['x'], pt['y'], pt['z'], duration=0.15)
                
                concs = []
                for _ in range(3):
                    conc = self.get_concentration(pt['x'], pt['y'], pt['z'])
                    concs.append(conc)
                    rospy.sleep(0.05)
                
                avg_conc = np.mean(concs)
                
                measurement = {
                    'x': pt['x'], 'y': pt['y'], 'z': pt['z'],
                    'downwind': dist, 'cross_offset': pt['cross_offset'],
                    'conc': avg_conc
                }
                all_measurements.append(measurement)
                self.source_data[source['id']].append(measurement)
                self.measurements.append((source['id'], measurement))
                
                # 实时打印位置和浓度
                rospy.loginfo(f"({pt['x']:7.1f}, {pt['y']:7.1f}, {pt['z']:5.1f}) | 浓度 = {avg_conc:8.4f}")
            
            rospy.sleep(0.5)
        
        return all_measurements
    
    def fit_emission(self, source_id):
        data = self.source_data[source_id]
        
        if len(data) < 5:
            return None, None
        
        source = self.sources[source_id]
        H = source['z']
        
        valid_data = [d for d in data if d['conc'] > 0.01]
        if len(valid_data) < 5:
            return None, None
        
        downwinds = np.array([d['downwind'] for d in valid_data])
        cross_offsets = np.array([abs(d['cross_offset']) for d in valid_data])
        z_heights = np.array([d['z'] for d in valid_data])
        concentrations = np.array([d['conc'] for d in valid_data])
        
        weights = concentrations / (np.max(concentrations) + 1e-6)
        
        def loss_function(Q):
            if Q <= 0:
                return 1e10
            predicted = []
            for i in range(len(downwinds)):
                pred = self.gaussian_plume_concentration(
                    downwinds[i], cross_offsets[i], z_heights[i], H, Q[0]
                )
                predicted.append(pred)
            predicted = np.array(predicted)
            residuals = concentrations - predicted
            return np.sum(weights * residuals**2)
        
        max_conc = np.max(concentrations)
        Q_guess = max_conc * 2 * math.pi * self.wind_speed * self.sigma_y0 * self.sigma_z0 * 10
        
        try:
            result = minimize(loss_function, [Q_guess], method='L-BFGS-B', bounds=[(0, None)])
            Q_fit = result.x[0]
            
            predicted_final = np.array([self.gaussian_plume_concentration(
                downwinds[i], cross_offsets[i], z_heights[i], H, Q_fit
            ) for i in range(len(downwinds))])
            
            residuals = concentrations - predicted_final
            rmse = np.sqrt(np.mean(residuals**2))
            relative_error = rmse / (np.mean(concentrations) + 1e-6)
            
            return Q_fit, relative_error
        except:
            return None, None
    
    def visualize(self):
        """实时可视化"""
        marker_array = MarkerArray()
        marker_id = 0
        current_time = self.get_rostime()
        
        # 气源点
        for src in self.sources:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = current_time
            marker.ns = "sources"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = src['x']
            marker.pose.position.y = src['y']
            marker.pose.position.z = src['z']
            marker.scale.x = 1.2
            marker.scale.y = 1.2
            marker.scale.z = 1.2
            marker.lifetime = rospy.Duration(0)  # 永久显示
            
            if src['Q'] is not None:
                marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)
            else:
                marker.color = ColorRGBA(0.5, 0.5, 0.5, 0.5)
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # 文本标签
            text = Marker()
            text.header.frame_id = "world"
            text.header.stamp = current_time
            text.ns = "text"
            text.id = marker_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = src['x']
            text.pose.position.y = src['y']
            text.pose.position.z = src['z'] + 2.5
            text.scale.z = 0.4
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.lifetime = rospy.Duration(0)
            
            if src['Q']:
                text.text = f"{src['name']}\nQ={src['Q']:.1f}"
            else:
                text.text = src['name']
            
            marker_array.markers.append(text)
            marker_id += 1
        
        # 采样点
        for i, (src_id, meas) in enumerate(self.measurements):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = current_time
            marker.ns = "samples"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = meas['x']
            marker.pose.position.y = meas['y']
            marker.pose.position.z = meas['z']
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.lifetime = rospy.Duration(0)
            
            conc = meas['conc']
            if conc < 0.1:
                marker.color = ColorRGBA(0.0, 0.5, 0.0, 0.6)
            elif conc < 0.5:
                marker.color = ColorRGBA(0.5, 0.5, 0.0, 0.7)
            elif conc < 1.0:
                marker.color = ColorRGBA(0.8, 0.3, 0.0, 0.8)
            else:
                marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.9)
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.marker_pub.publish(marker_array)
    
    def sample_source(self, source):
        source_id = source['id']
        source_name = source['name']
        
        rospy.loginfo(f"开始采样: {source_name}")
        
        # 更新风场
        self.update_wind_parameters()
        
        # 执行采样
        self.collect_measurements(source)
        
        # 反演
        rospy.loginfo(f"反演计算中...")
        Q_fit, error = self.fit_emission(source_id)
        
        if Q_fit is not None:
            source['Q'] = Q_fit
            rospy.loginfo(f"{source_name} 反演结果: Q = {Q_fit:.4f}")
        else:
            rospy.logwarn(f"{source_name} 反演失败")
        
        # 实时可视化
        self.visualize()
        
        return Q_fit
    
    def final_report(self):
        rospy.loginfo("")
        rospy.loginfo("="*60)
        rospy.loginfo("反演结果")
        rospy.loginfo("="*60)
        rospy.loginfo(f"{'名称':<12} {'X(m)':<8} {'Y(m)':<8} {'Z(m)':<8} {'排放强度Q':<12}")
        rospy.loginfo("-"*60)
        
        for src in self.sources:
            if src['Q']:
                rospy.loginfo(f"{src['name']:<12} {src['x']:8.1f} {src['y']:8.1f} {src['z']:8.1f} {src['Q']:12.4f}")
            else:
                rospy.loginfo(f"{src['name']:<12} {src['x']:8.1f} {src['y']:8.1f} {src['z']:8.1f} {'未检测到':<12}")
        
        rospy.loginfo("="*60)
        
        # 保存结果
        with open('inversion_results.txt', 'w') as f:
            f.write("="*60 + "\n")
            f.write("反演结果\n")
            f.write("="*60 + "\n")
            f.write(f"{'名称':<12} {'X(m)':<8} {'Y(m)':<8} {'Z(m)':<8} {'排放强度Q':<12}\n")
            f.write("-"*60 + "\n")
            for src in self.sources:
                if src['Q']:
                    f.write(f"{src['name']:<12} {src['x']:8.1f} {src['y']:8.1f} {src['z']:8.1f} {src['Q']:12.4f}\n")
        
        # 发布结果
        result_msg = Float64MultiArray()
        result_data = []
        for src in self.sources:
            result_data.extend([src['x'], src['y'], src['z'], src['Q'] if src['Q'] else 0])
        result_msg.data = result_data
        self.result_pub.publish(result_msg)
    
    def run(self):
        for idx, source in enumerate(self.sources):
            self.sample_source(source)
            
            # 清理数据准备下一个源
            self.source_data[source['id']] = []
            
            if idx < self.n_sources - 1:
                rospy.sleep(2.0)
        
        self.final_report()
        
        # 保持可视化运行
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.visualize()
            self.publish_path()  # 持续发布路径
            rate.sleep()


if __name__ == "__main__":
    try:
        PlumeInversion()
    except rospy.ROSInterruptException:
        pass