import rclpy  
from rclpy.node import Node  
from std_msgs.msg import Float64  


class BoatController(Node):  
    def __init__(self):  
        super().__init__('circle_controller')  
        self.la_pub = self.create_publisher(Float64, '/wamv1/thrusters/left/pos', 10)  
        self.ra_pub = self.create_publisher(Float64, '/wamv1/thrusters/right/pos', 10)  
        self.lt_pub = self.create_publisher(Float64, '/wamv1/thrusters/left/thrust', 10)  
        self.rt_pub = self.create_publisher(Float64, '/wamv1/thrusters/right/thrust', 10)  
        self.msgs=[ Float64() for i in range(4)]
        self.action='turn'
        self.timer = self.create_timer(5.0, self.publish_angle)  # 每隔 5 秒调用一次 
        self.get_logger().info("创建圆圈控制器成功1")

        self.la_pub2 = self.create_publisher(Float64, '/wamv2/thrusters/left/pos', 10)  
        self.ra_pub2 = self.create_publisher(Float64, '/wamv2/thrusters/right/pos', 10)  
        self.lt_pub2 = self.create_publisher(Float64, '/wamv2/thrusters/left/thrust', 10)  
        self.rt_pub2 = self.create_publisher(Float64, '/wamv2/thrusters/right/thrust', 10)  
        self.msgs2=[ Float64() for i in range(4)]
        self.action2='turn'
        self.timer2 = self.create_timer(5.0, self.publish_angle2)  # 每隔 5 秒调用一次 
        self.get_logger().info("创建圆圈控制器成功2")



    def publish_angle(self):  
        self.get_logger().info(self.action)
        if self.action=='turn':
            self.msgs[0].data= 1.0
            self.msgs[1].data= 1.0 
            self.msgs[2].data= 10.0
            self.msgs[3].data= 10.0
            self.action='go'
        elif self.action=='go':
            self.msgs[0].data= 0.0
            self.msgs[1].data= 0.0 
            self.msgs[2].data= 100.0
            self.msgs[3].data= 100.0
            self.action='turn'
        else:
            self.msgs[0].data= 0.0
            self.msgs[1].data= 0.0 
            self.msgs[2].data= 0.0
            self.msgs[3].data= 0.0
            

        self.la_pub.publish(self.msgs[0]) 
        self.ra_pub.publish(self.msgs[1])
        self.lt_pub.publish(self.msgs[2])  
        self.rt_pub.publish(self.msgs[3]) 

    def publish_angle2(self):  
        self.get_logger().info(self.action2)
        if self.action2=='turn':
            self.msgs2[0].data= 1.0
            self.msgs2[1].data= 1.0 
            self.msgs2[2].data= 10.0
            self.msgs2[3].data= 10.0
            self.action2='go'
        elif self.action2=='go':
            self.msgs2[0].data= 0.0
            self.msgs2[1].data= 0.0 
            self.msgs2[2].data= 100.0
            self.msgs2[3].data= 100.0
            self.action2='turn'
        else:
            self.msgs2[0].data= 0.0
            self.msgs2[1].data= 0.0 
            self.msgs2[2].data= 0.0
            self.msgs2[3].data= 0.0
            

        self.la_pub2.publish(self.msgs2[0]) 
        self.ra_pub2.publish(self.msgs2[1])
        self.lt_pub2.publish(self.msgs2[2])  
        self.rt_pub2.publish(self.msgs2[3]) 
  


def main(args=None):  
    rclpy.init(args=args)  
    angle_publisher = BoatController()  
    rclpy.spin(angle_publisher)  
    angle_publisher.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':  
    main()