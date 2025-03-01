#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
import time

class ContinuousPredefinedVelocityPublisher:
    def __init__(self):
        rospy.init_node('continuous_predefined_velocity_publisher', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # Adjust the rate based on your requirements

        # Predefined velocities list
        self.velocities = [
            Twist(linear=Vector3(-0.1, 0, 0), angular=Vector3(0, 0, 0.3)),
            Twist(linear=Vector3(0.2, 0, 0), angular=Vector3(0, 0, -0.4)),
            Twist(linear=Vector3(0.25, 0, 0), angular=Vector3(0, 0, 0.5)),
        ]

    def run(self, total_duration):
        start_time = time.time()
        velocity_index = 0

        while not rospy.is_shutdown() and time.time() - start_time < total_duration:
            if (time.time() - start_time) % 5.0 < self.rate.sleep_dur.to_sec():
                # Update velocities every 5 seconds
                twist_msg = self.velocities[velocity_index]
                velocity_index = (velocity_index + 1) % len(self.velocities)
                print(twist_msg)
                self.cmd_vel_pub.publish(twist_msg)

            self.rate.sleep()

        # Stop the robot after the specified duration
        self.stop_robot()

    def stop_robot(self):
        zero_twist_msg = Twist()  # Stop the robot by publishing zero velocities
        self.cmd_vel_pub.publish(zero_twist_msg)

if __name__ == '__main__':
    try:
        publisher = ContinuousPredefinedVelocityPublisher()
        publisher.run(total_duration=60.0)  # Run the robot for 1 minute
    except rospy.ROSInterruptException:
        pass