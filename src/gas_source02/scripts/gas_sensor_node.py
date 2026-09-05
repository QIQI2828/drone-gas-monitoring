#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry          # RotorS 发布的里程计类型
from gas_source02.srv import GetConcentration, GetConcentrationRequest
from gas_source02.msg import GasConcentration

class GasSensorNode:
    def __init__(self):
        rospy.init_node('gas_sensor_node')
        
        # 等待气体浓度服务
        rospy.loginfo("等待气体浓度服务...")
        rospy.wait_for_service('/get_concentration')
        self.get_concentration = rospy.ServiceProxy('/get_concentration', GetConcentration)
        rospy.loginfo("气体浓度服务连接成功")
        
        # 订阅 RotorS 的里程计话题（无人机真实位姿）
        self.odom_sub = rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, self.odom_callback)
        
        # 发布传感器读数（浓度+位置）
        self.sensor_pub = rospy.Publisher('/uav/gas_reading', GasConcentration, queue_size=10)
        
        # 传感器噪声标准差（模拟真实传感器）
        self.noise_std = 0.01
        
        rospy.loginfo("气体传感器节点已启动，监听无人机")
        
    def odom_callback(self, odom_msg):
        # 提取无人机位置（单位：米）
        pos = odom_msg.pose.pose.position
        
        # 调用气体浓度服务
        req = GetConcentrationRequest()
        req.point = pos
        try:
            resp = self.get_concentration(req)
            true_conc = resp.concentration
            # 添加高斯噪声
            noisy_conc = max(0.0, true_conc + np.random.normal(0, self.noise_std))
            
            # 构造并发布 GasConcentration 消息
            gas_msg = GasConcentration()
            gas_msg.header.stamp = rospy.Time.now()
            gas_msg.header.frame_id = "world"
            gas_msg.position = pos
            gas_msg.concentration = noisy_conc
            self.sensor_pub.publish(gas_msg)
            
            # 每10次打印一次调试信息
            if not hasattr(self, 'count'):
                self.count = 0
            self.count += 1
            if self.count % 10 == 0:
                rospy.loginfo("无人机位置 (%.1f, %.1f, %.1f) -> 浓度: %.6f", 
                             pos.x, pos.y, pos.z, noisy_conc)
        except Exception as e:
            rospy.logerr("浓度服务调用失败: %s", e)

if __name__ == '__main__':
    try:
        GasSensorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass