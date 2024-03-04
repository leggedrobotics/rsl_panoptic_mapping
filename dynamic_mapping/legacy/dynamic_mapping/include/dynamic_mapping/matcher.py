import rospy
from message_filters import Subscriber
from dynamic_mapping.utils import LidarApproximateTimeSynchronizer
import sensor_msgs


class MessageMatcher:
    def __init__(
        self,
        callback,
        sensor_frequency_Hz=10.0,
        pointcloud_topic="/ouster_points_self_filtered",
        camera_topic="/camMainView/image_raw/compressed",
        raw_pointcloud_pub_topic="~raw_pointcloud",
        raw_image_topic="~raw_image",
    ):
        cloudSub = Subscriber(pointcloud_topic, sensor_msgs.msg.PointCloud2, queue_size=1)
        cameraSub = Subscriber(camera_topic, sensor_msgs.msg.CompressedImage, queue_size=1)
        self._time_synchronizer = LidarApproximateTimeSynchronizer([cloudSub, cameraSub], 50, 0.03, sensor_frequency_Hz=sensor_frequency_Hz)
        self._time_synchronizer.registerCallback(self._match_pointcloud_camera)

        self._raw_pointcloud_publisher = rospy.Publisher(raw_pointcloud_pub_topic, sensor_msgs.msg.PointCloud2, queue_size=1)
        self._raw_camera_image_publisher = rospy.Publisher(raw_image_topic, sensor_msgs.msg.CompressedImage, queue_size=1)

        self._no_ground_subscriber = rospy.Subscriber(
            "/ground_remover/no_ground_cloud",
            sensor_msgs.msg.PointCloud2,
            self._new_ground_removal,
            queue_size=1,
        )
        self._seg_mask_subscriber = rospy.Subscriber("/src/mask", sensor_msgs.msg.Image, self._new_seg_mask, queue_size=1)

        self._has_new_ground_removal = False
        self._has_new_mask = False
        self._waiting_for_data = False

        self._no_ground_cloud = None
        self._seg_mask = None
        self._raw_cloud = None
        self._raw_image = None

        self.callback = callback

    def _match_pointcloud_camera(self, pcl, img):
        if self._waiting_for_data:
            return
        self._raw_cloud = pcl
        self._raw_image = img
        self._waiting_for_data = True
        print(f"time delay img -> pcl: {(img.header.stamp.to_sec() - pcl.header.stamp.to_sec())* 1000:.3f}ms")
        self._raw_pointcloud_publisher.publish(pcl)
        self._raw_camera_image_publisher.publish(img)

    def _new_ground_removal(self, pcl):
        self._no_ground_cloud = pcl
        self._has_new_ground_removal = True
        self._publish_matched_data()

    def _new_seg_mask(self, img):
        self._seg_mask = img
        self._has_new_mask = True
        self._publish_matched_data()

    def _publish_matched_data(self):
        if self._has_new_ground_removal and self._has_new_mask:
            self._has_new_ground_removal = False
            self._has_new_mask = False
            self._waiting_for_data = False
            self.callback(self._raw_cloud, self._no_ground_cloud, self._seg_mask, self._raw_image)
