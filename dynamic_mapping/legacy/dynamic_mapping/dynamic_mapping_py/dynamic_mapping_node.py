import rospy
from dynamic_mapping.moving_objects_filter import MovingObjectsFilter
from dynamic_mapping.matcher import MessageMatcher


if __name__ == "__main__":
    node_name = "dynamic_mapping"
    rospy.init_node(node_name)
    sensor_frequency = rospy.get_param("~sensor_frequency", 10.0)
    use_world_frame = rospy.get_param("~use_world_frame", True)
    pointcloud_topic = rospy.get_param("~pointcloud_topic", "/ouster_points_self_filtered")
    movingObjectsFilter = MovingObjectsFilter(use_world_frame=use_world_frame)
    matcher = MessageMatcher(movingObjectsFilter.filter_moving_objects, sensor_frequency, pointcloud_topic=pointcloud_topic)
    rospy.loginfo(f"Use World Frame: {use_world_frame}; Sensor frequency: {sensor_frequency}Hz")
    rospy.loginfo("Started dynamic mapping")
    rospy.spin()
