import rclpy
from rclpy.node import Node

from interface_pkg.srv import ParticulateMatter
from interface_pkg.msg import AirQualityIndex

class PM_Sensor(Node):

    def __init__(self):
        super().__init__('particulate_matter_sensor')
        self.pm_type = 1 # default : PM_10

        self.pm_client = self.create_client(ParticulateMatter, 'particulate_matter')
        self.pub = self.create_publisher(AirQualityIndex, "aqi", 10)
		
    def send_request(self):
        self.req = ParticulateMatter.Request()   
        self.req.pm_type = self.pm_type          
        self.pm_client.wait_for_service()
        self.future = self.pm_client.call_async(self.req)    
        rclpy.spin_until_future_complete(self, self.future)
        self.result = self.future.result()
        return self.result

        return futures  

    def publish_aqi(self, pmp):
        aqi_msg = AirQualityIndex()
        aqi_msg.air_quality_index = self.calculate_pmi(pmp)
        self.pub.publish(aqi_msg)   

    def calculate_pmi(self, pmp):
        self.get_logger().info(f'pm_type = {self.pm_type}, pm = {pmp}')
        if self.pm_type == ParticulateMatter.Request.PM_10:
            if pmp <= 30: pmi = 1
            elif pmp <= 80: pmi = 2
            elif pmp <= 150: pmi = 3
            else: pmi = 4
        elif self.pm_type == ParticulateMatter.Request.PM_25:
            if pmp <= 15: pmi = 1
            elif pmp <= 35: pmi = 2
            elif pmp <= 75: pmi = 3
            else: pmi = 4
        else:
            self.get_logger().error('Please make sure particulate matter type(PM_10, PM_25).')
        
        print(f'calculate_pmi : pmi = {pmi}, pmp = {pmp}')
        return pmi

def main(args=None):
    rclpy.init(args=args)
    sensor = PM_Sensor()
    user_trigger = True 
    try:
        while rclpy.ok():
            if user_trigger is True:
                res = sensor.send_request()
                sensor.get_logger().info(f'service_response = {res}')
                sensor.publish_aqi(res.particulate_matter) 
                user_trigger = False
            else:
                input('Press Enter for next service call.')
                user_trigger = True

    except KeyboardInterrupt:  
        sensor.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        sensor.destroy_node()  
        rclpy.shutdown()   


if __name__ == '__main__':
    main()