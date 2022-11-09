import rclpy
from rclpy.node import Node

from interface_pkg.msg import AirQualityIndex

class CentralControlUnit(Node):
	def __init__(self):
		super().__init__("central_control_unit_node")
		self.pub = self.create_publisher(AirQualityIndex, "aqi_ac", 10)
		self.sub = self.create_subscription(AirQualityIndex, "aqi", self.relay_to_air_conditioner, 10)
		
	def relay_to_air_conditioner(self, aqi_msg):
		aqi = aqi_msg.air_quality_index
		print(f'aqi = {aqi}')
		self.pub.publish(aqi_msg)

def main():
	rclpy.init()
	
	ccu = CentralControlUnit()
	
	print("Central Control Unit Node Running...")
	
	try:
		rclpy.spin(ccu)
	except KeyboardInterrupt:
		ccu.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()