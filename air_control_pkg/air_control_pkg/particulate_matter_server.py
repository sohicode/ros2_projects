import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from interface_pkg.srv import ParticulateMatter

PM_10_DEFAULT = 150
PM_25_DEFAULT = 30

class PM_Server(Node):

    def __init__(self):
        super().__init__('particulate_matter_server')
        self.get_logger().info('Particulate Matter Server Running.')

        # parameter 선언
        self.declare_parameter("pm_10_parameter", PM_10_DEFAULT)
        self.declare_parameter("pm_25_parameter", PM_25_DEFAULT)

        # particulate_matter 서비스 서버 생성
        self.pm_srv = self.create_service(
            ParticulateMatter,                  # 서비스 타입
            'particulate_matter',               # 서비스 이름
            self.get_pmi_index                  # 콜백 함수
        )
        
        # new_pmp 토픽 서브스크립션 생성
        self.sub = self.create_subscription(Int32, "new_pmp", self.calibrate_air_condition, 10)

    def get_pmi_index(self, request, response):
        self.get_logger().info('Particulate Matter Request Received... ')
        pm_type = request.pm_type
        self.get_logger().info(f'pm_type = {pm_type}')

        pmp = PM_10_DEFAULT
        if pm_type == ParticulateMatter.Request.PM_10:
            pmp = self.get_parameter("pm_10_parameter").get_parameter_value().integer_value
        else:
            pmp = self.get_parameter("pm_25_parameter").get_parameter_value().integer_value
        
        response.particulate_matter = pmp
        return response

    def calibrate_air_condition(self, pmp_msg):
        new_pmp = int(pmp_msg.data)
        pmp = self.get_parameter("pm_10_parameter").get_parameter_value().integer_value
        print(f'calibrate_air_condition: pmp = {pmp}, new_pmp = {new_pmp}')

        my_new_param = rclpy.parameter.Parameter(
            'pm_10_parameter',
            rclpy.Parameter.Type.INTEGER,
            pmp + new_pmp
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)
        pmp2 = self.get_parameter("pm_10_parameter").get_parameter_value().integer_value
        print(f'calibrate_air_condition: pmp = {pmp}, pmp2 = {pmp2}')
            

def main(args=None):
    print('new server')
    rclpy.init(args=args)
    pm_server = PM_Server()

    try:
        rclpy.spin(pm_server)
    except KeyboardInterrupt:
        pm_server.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pm_server.destroy_node()
        rclpy.shutdown()     

if __name__ == '__main__':
    main()