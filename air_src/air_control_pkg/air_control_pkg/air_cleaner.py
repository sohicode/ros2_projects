import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from interface_pkg.msg import AirQualityIndex

class AirCleaner(Node):
	def __init__(self):
		super().__init__("air_conditioner_node")
		self.sub = self.create_subscription(AirQualityIndex, "aqi_ac", self.man_at_work, 10)
		self.pub = self.create_publisher(Int32, "new_pmp", 10)
		
	def man_at_work(self, aqi_msg):
		aqi = aqi_msg.air_quality_index
		# pmp = self.get_parameter("pm_10_parameter").get_parameter_value().integer_value
		pmp = 0
		print(f'aqi_ac = {aqi}')

		if aqi == 2:
			pmp = -5
		elif aqi == 3:
			pmp = -10
		elif aqi == 4:
			pmp = -20
		else:
			pmp = 0
		
		print(f'new pmp = {pmp}')
		msg = Int32()
		msg.data = int(pmp)
		self.pub.publish(msg)

def main():
	rclpy.init()
	
	ac = AirCleaner()
	
	print("Central Control Unit Node Running...")
	
	try:
		rclpy.spin(ac)
	except KeyboardInterrupt:
		ac.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()