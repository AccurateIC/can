#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import random
import numpy as np

def generate_smooth_rpm_curve(min_rpm, max_rpm, step_size=10):
    """
    Generates RPM values from min_rpm to max_rpm and then back to min_rpm, forming a smooth curve.
    """
    # Generate ascending values from min_rpm to max_rpm
    ascending_rpms = np.arange(min_rpm, max_rpm + step_size, step_size)
    # Generate descending values from max_rpm to min_rpm
    descending_rpms = np.arange(max_rpm, min_rpm - step_size, -step_size)
    # Concatenate both to form a smooth curve
    rpm_curve = np.concatenate((ascending_rpms, descending_rpms))
    return rpm_curve

def get_random_min_max_rpm(min_rpm=0, max_rpm=1500, min_diff=500):
    """
    Returns a random minimum and maximum RPM with at least a minimum difference of 500.
    """
    random_min = random.randint(min_rpm, max_rpm - min_diff)
    random_max = random.randint(random_min + min_diff, max_rpm)
    rospy.loginfo(f"Current loop range {random_min}, {random_max}")
    return random_min, random_max

def publish_rpm_values():
    rospy.init_node('smooth_rpm_stress_test', anonymous=True)
    rpm_publisher = rospy.Publisher('/motor_rpm', Float32, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz, i.e., 100 messages per second

    duration = 3 * 60 * 60  # Run for 3 hours (if needed)
    elapsed_time = 0
    last_print_time = 0

    while not rospy.is_shutdown() and elapsed_time < duration:
        # Get random min and max RPM values with at least 500 RPM difference
        min_rpm, max_rpm = get_random_min_max_rpm()

        # Generate a smooth RPM curve (min to max and back to min)
        rpm_curve = generate_smooth_rpm_curve(min_rpm, max_rpm)

        # Publish each RPM value for 2 seconds (at 100 Hz)
        for rpm in rpm_curve:
            rospy.loginfo(f"Holding RPM: {rpm} for 2 seconds")
            for _ in range(2 * 100):  # 15 seconds at 100 Hz
                if rospy.is_shutdown() or elapsed_time >= duration:
                    break  # Stop if ROS is shutting down or time is up
                rpm_publisher.publish(rpm)
                rate.sleep()  # Sleep to maintain 100 Hz publishing rate
                elapsed_time += 0.01  # Each iteration is 1/100 seconds (0.01s)
                
                current_time = int(elapsed_time)
                if current_time > last_print_time:
                    hours = current_time // 3600
                    minutes = (current_time % 3600) // 60
                    seconds = current_time % 60
                    elapsed_time_str = f"{hours:02}:{minutes:02}:{seconds:02}"
                    rospy.loginfo(f"Elapsed time: {elapsed_time_str}")
                    last_print_time = current_time  # Update last printed time

            if elapsed_time >= duration:
                break  # Stop after the desired test duration

    rospy.loginfo("Stress test completed.")

if __name__ == '__main__':
    try:
        publish_rpm_values()
    except rospy.ROSInterruptException:
        pass
