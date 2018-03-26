#!/usr/bin/env python2
import socket



import rospy
from std_msgs.msg import String
    
def talker():
	serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	
	serversocket.bind(('10.8.48.217', 8080))
	
	serversocket.listen(5)
	pub = rospy.Publisher('unity_locations', String, queue_size=10)
	rospy.init_node('socket_server', anonymous=True)
	rate = rospy.Rate(40) # 60hz
	while not rospy.is_shutdown():
		c, addr = serversocket.accept()     # Establish connection with client.
		hello_str = c.recv(180)
		c.send('Cool')
		c.close()  
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
