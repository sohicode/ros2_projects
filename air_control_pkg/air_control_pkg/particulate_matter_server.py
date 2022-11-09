import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int32

from interface_pkg.srv import ParticulateMatter
from rclpy.executors import MultiThreadedExecutor

import traceback

PM_10_DEFAULT = 150
PM_25_DEFAULT = 30

class PM_Server(Node):

    def __init__(self):
        super().__init__('particulate_matter_server')
        self.get_logger().info('Particulate Matter Server Running.')

        self.declare_parameter("pm_10_parameter", PM_10_DEFAULT)
        self.declare_parameter("pm_25_parameter", PM_25_DEFAULT)

        self.callback_group = ReentrantCallbackGroup()

        self.pm_srv = self.create_service(
            ParticulateMatter
            ,'particulate_matter'
            ,self.get_pmi_index
            ,callback_group=self.callback_group
            )
        
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
        try:
            my_new_param = rclpy.parameter.Parameter(
                'pm_10_parameter',
                rclpy.Parameter.Type.INTEGER,
                pmp + new_pmp
            )
            all_new_parameters = [my_new_param]
            self.set_parameters(all_new_parameters)
            pmp2 = self.get_parameter("pm_10_parameter").get_parameter_value().integer_value
            print(f'calibrate_air_condition: pmp2 = {pmp}, pmp = {pmp + new_pmp}')
            
        except Exception:
            print("Exception !!!!")
            print(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    try:
        pm_server = PM_Server()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(pm_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pm_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            pm_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()