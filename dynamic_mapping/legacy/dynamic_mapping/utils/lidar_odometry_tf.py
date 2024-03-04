import rospy
import tf2_ros
import ros_numpy
import time
from geometry_msgs.msg import TransformStamped, Transform
from tf import transformations

rospy.init_node("tf_transformer")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
broadcaster = tf2_ros.TransformBroadcaster()

while rospy.Time().now() == 0:
    time.sleep(0.01)


has_base_tf = False
rate = rospy.Rate(10.0)

while not rospy.is_shutdown():
    try:
        if not has_base_tf:
            map3d_to_map = tfBuffer.lookup_transform("odom", "os_sensor", rospy.Time())
            T_3_M = ros_numpy.numpify(map3d_to_map.transform)
            has_base_tf = True
        else:
            range3_to_map3 = tfBuffer.lookup_transform("map_o3d", "range_sensor_o3d", rospy.Time())
            T_R_3 = ros_numpy.numpify(range3_to_map3.transform)
            T_R_M = T_R_3 @ T_3_M
            odom_to_lidar = tfBuffer.lookup_transform("os_sensor", "odom", rospy.Time())
            T_O_R = ros_numpy.numpify(odom_to_lidar.transform)
            T_O_M = T_O_R @ T_R_M
            T_M_O = transformations.inverse_matrix(T_O_M)
            tf = TransformStamped()
            tf.header.frame_id = "map"
            tf.child_frame_id = "odom"
            tf.header.stamp = rospy.Time().now()
            tfmsg = ros_numpy.msgify(Transform, T_M_O)
            tf.transform = tfmsg
            broadcaster.sendTransform(tf)
    except Exception as e:
        print(f"{type(e)}: {e}")

    rate.sleep()
