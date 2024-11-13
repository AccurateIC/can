import rospy
from std_msgs.msg import Float64
import time

def publisher():
    rospy.init_node('steer_demo', anonymous=True)
    pub = rospy.Publisher('/steering_motor_cmd', Float64, queue_size=10)
    
    rate = rospy.Rate(100)  # Publish rate at 100 Hz
    
    current_value = -200.00
    start_time = time.time()

    def shutdown_hook():
        # Publish 0 RPM before exiting
        for i in range(10):
            pub.publish(0)
            rospy.loginfo("Published: 0 (Exiting)")
            # rospy.sleep(1)  # Wait for the message to be sent

    rospy.on_shutdown(shutdown_hook)

    while not rospy.is_shutdown():
        # Check if 3.5 seconds have passed
        if time.time() - start_time >= 12:
            # Reverse the value and reset the timer
            current_value = -current_value
            start_time = time.time()
        
        # Publish the current value at 100 Hz
        pub.publish(current_value)
        rospy.loginfo(f"Published: {current_value}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
