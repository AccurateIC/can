import rospy
from geometry_msgs.msg import Twist

def callback(data):
    rospy.loginfo(f"I heard linear: x={data.linear.x}, y={data.linear.y}, z={data.linear.z} angular: x={data.angular.x}, y={data.angular.y}, z={data.angular.z}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()