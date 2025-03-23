#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import TransformStamped

def broadcast_dynamic_tf():
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    angle = 0.0
    while not rospy.is_shutdown():
        # Simulate rotation around Z axis
        angle += 0.05
        x = 2.0
        y = 1.0
        z = 0.0

        quat = tf.transformations.quaternion_from_euler(0, 0, angle)  # Yaw rotation

        br.sendTransform(
            (x, y, z),
            quat,
            rospy.Time.now(),
            "robot_1",
            "world"
        )

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_dynamic_tf()
    except rospy.ROSInterruptException:
        pass
