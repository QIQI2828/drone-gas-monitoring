#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
from gas_source02.srv import GetConcentration, GetConcentrationRequest

class BasicGasClient:
    def __init__(self):
        rospy.init_node('basic_gas_client')
        
        # 等待并连接服务
        rospy.loginfo("连接气体浓度服务...")
        rospy.wait_for_service('get_concentration')
        self.get_concentration = rospy.ServiceProxy('get_concentration', GetConcentration)
        rospy.loginfo("服务连接成功")

    def test_service(self):
        """测试服务调用"""
        # 创建一个测试点
        test_point = Point(-20.0, -15.0, 23.5)
        
        try:
            # 调用服务
            req = GetConcentrationRequest()
            req.point = test_point
            resp = self.get_concentration(req)
            
            rospy.loginfo("测试点 (%.1f, %.1f, %.1f) 的浓度: %.6f", 
                         test_point.x, test_point.y, test_point.z, 
                         resp.concentration)
                         
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)

if __name__ == '__main__':
    try:
        client = BasicGasClient()
        client.test_service()
    except rospy.ROSInterruptException:
        pass