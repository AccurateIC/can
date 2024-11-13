import rospy
from geometry_msgs.msg import Twist
import random
import time
 
def twist_publisher():
    rospy.init_node('twist_publisher', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(0.1)  # 1 Hz
 
    while not rospy.is_shutdown():
        # Create a dummy Twist message with random values
        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(-5.0, 5.0)  # Random value between -5.0 and 5.0 m/s
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = random.uniform(-1.0, 1.0)  # Random value between -1.0 and 1.0 rad/s
 
        # Publish the message
        pub.publish(twist_msg)
 
        # Wait for a short duration
        rate.sleep()
 
if __name__ == '__main__':
    try:
        twist_publisher()
    except rospy.ROSInterruptException:
        pass
 