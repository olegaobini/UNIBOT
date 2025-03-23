#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def publish_static_tf():
    rospy.init_node('static_tf_publisher')

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_tf = geometry_msgs.msg.TransformStamped()
    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = "world"
    static_tf.child_frame_id = "robot_1"

    static_tf.transform.translation.x = 2.0
    static_tf.transform.translation.y = 1.0
    static_tf.transform.translation.z = 0.0

    quat = tf_conversions.transformations.quaternion_from_euler(0, 0, 0.785)  # yaw = 45°
    static_tf.transform.rotation.x = quat[0]
    static_tf.transform.rotation.y = quat[1]
    static_tf.transform.rotation.z = quat[2]
    static_tf.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_tf)
    rospy.loginfo("Published static transform from world -> robot_1")

    rospy.spin()

if __name__ == '__main__':
    publish_static_tf()
