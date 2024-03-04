import rospy
import sensor_msgs
import cv_bridge
import cv2
import numpy as np
import quaternion
import ros_numpy
import tf2_ros
from math import sqrt
from jsk_recognition_msgs.msg import BoundingBoxArray
from dynamic_mapping import visualization, multitracking, projection_utils, utils
from dynamic_mapping.debug import ImgVis
from dynamic_mapping.utils import PointCloud
from sklearn.cluster import DBSCAN
from datetime import datetime
import time


class MovingObjectsFilter(object):
    def __init__(self, debug=True, vis_proj=False, use_world_frame=True):

        self.debug = debug
        self.vis_proj = vis_proj
        self.use_world_frame = use_world_frame

        self.bridge = cv_bridge.CvBridge()

        self.filter_labels = range(1, 8)

        self.cloudPub = rospy.Publisher("/new_cloud", sensor_msgs.msg.PointCloud2, queue_size=1)
        self.clusterPub = rospy.Publisher("/cluster_cloud", sensor_msgs.msg.PointCloud2, queue_size=1)

        self.bboxPub = rospy.Publisher("/bboxes", BoundingBoxArray, queue_size=1)
        self.trackerPub = rospy.Publisher("/trackerBboxes", BoundingBoxArray, queue_size=1)

        # image is scaled in rosbag
        self.camera_intrinsics = np.matrix([[1918.09863, 0, 2036.03749], [0, 1917.92043, 1494.94564]]) * 0.4
        self.camera_intrinsics = np.insert(self.camera_intrinsics, 2, values=[0, 0, 1], axis=0)

        self.camera_projection = np.matrix(
            [
                [739.8135742187501, 0.0, 821.5126881316421, 0.0],
                [0.0, 748.5543945312501, 598.1012262159028, 0.0],
                [0.0, 0.0, 1.0, 0.0],
            ]
        )
        self.distorsion_coeffs = np.array([[-0.044303, 0.006917, -0.000472, -0.000009, 0.000000]])

        self.camera_info_subscriber = rospy.Subscriber(
            "/camMainView/camera_info",
            sensor_msgs.msg.CameraInfo,
            self._camera_info_callback,
            queue_size=1,
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.clustering = DBSCAN(eps=0.6, min_samples=6, algorithm="ball_tree")

        self.tracker = multitracking.Tracker(initiable_labels=self.filter_labels)
        self.moved_tracks = set()

        self.detection_bboxes = []

        self.plotter = ImgVis()

    def _camera_info_callback(self, msg):
        self.camera_intrinsics = np.array(msg.K).reshape(3, 3)
        self.camera_projection = np.array(msg.P).reshape(3, 4)
        self.distorsion_coeffs = np.array([msg.D if len(msg.D) else [0.0] * 5])

    def _publish_pointcloud(self, points, headerstamp):
        x, y, z = points.cloud.astype(np.float32).T
        recarr = np.core.records.fromarrays(
            [x, y, z],
            names="x,y,z",
        )
        new_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(recarr, stamp=headerstamp, frame_id=points.frame_id)
        self.cloudPub.publish(new_cloud)

    def _publish_clusters(self, clusters, headerstamp):
        if not all(x.frame_id == clusters[0].frame_id for x in clusters):
            raise ValueError("All clusters must be in the same coordinate frame")
        x = np.empty((1,))
        y = np.empty((1,))
        z = np.empty((1,))
        label = np.zeros((1,))
        cluster = np.empty((1,))

        for l, c in enumerate(clusters):
            cx, cy, cz, lbl = c.cloud.T
            cl = np.ones(cx.shape) * l
            x = np.hstack([x, cx])
            y = np.hstack([y, cy])
            z = np.hstack([z, cz])
            label = np.hstack([label, lbl])
            cluster = np.hstack([cluster, cl])

        recarr = np.core.records.fromarrays(
            [
                x[1:].astype(np.float32),
                y[1:].astype(np.float32),
                z[1:].astype(np.float32),
                label[1:].astype(np.float32),
                cluster[1:].astype(np.int32),
            ],
            names="x,y,z,label,cluster",
        )
        new_cloud = ros_numpy.point_cloud2.array_to_pointcloud2(recarr, stamp=headerstamp, frame_id=clusters[0].frame_id)
        self.clusterPub.publish(new_cloud)

    def _get_clusters(self, pts, min_pts=10):
        """
        Parameters
        ----------
        pts: numpy.ndarray
            Nx3 ndarray of points
        min_pts: int, optional
            minimum amount of points a cluster should include

        Returns
        -------
        list
            list of PointCloud
        """
        cluster_labels = self.clustering.fit(pts.cloud).labels_.astype(np.int32)
        unique_labels = dict(zip(*np.unique(cluster_labels, return_counts=True)))
        clusters = []
        for label, count in unique_labels.items():
            if label == -1 or count < min_pts:  # -1 denotes outliers
                continue
            clusters.append(PointCloud(pts.frame_id, pts.cloud[cluster_labels == label]))
        return clusters

    def _add_detections(self, labelled_clusters, visualize=False):
        boxes = []
        if not all(x.frame_id == labelled_clusters[0].frame_id for x in labelled_clusters):
            raise ValueError("All clusters must be in the same coordinate frame")
        for cluster in labelled_clusters:
            labels = dict(zip(*np.unique(np.asarray(cluster.cloud[:, 3]), return_counts=True)))
            label = max(labels, key=labels.get)
            c, q, s = utils.generate_3d_bbox(cluster.cloud[:, :3])
            ts = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
            self.tracker.add_detection(np.hstack([c, s]), label, ts)
            if visualize:
                bbox = visualization.generate_bbox_marker(c, q, s, cluster.frame_id)
                boxes.append(bbox)
        return boxes

    def _label_camera_points(self, points, mask, img=None):
        try:
            cv_mask = self.bridge.imgmsg_to_cv2(mask, "bgr8")
            cv_mask = cv2.undistort(cv_mask, self.camera_intrinsics, self.distorsion_coeffs)
            cv_mask = cv2.rotate(cv_mask, cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv_mask = cv2.flip(cv_mask, 0)

            if img:
                cv_img = self.bridge.compressed_imgmsg_to_cv2(img, "bgr8")
                cv_img = cv2.undistort(cv_img, self.camera_intrinsics, self.distorsion_coeffs)

        except cv_bridge.CvBridgeError as e:
            print(e)

        tf = self.tfBuffer.lookup_transform("camMainView", points.frame_id, rospy.Time())
        homTf = ros_numpy.numpify(tf.transform)

        projected_points = projection_utils.project_pointcloud_to_cam(points.cloud, homTf, self.camera_projection)
        pixel_points, visible_points = utils.filter_visible_points(projected_points, points.cloud, cv_mask.shape)

        if img and self.vis_proj:
            print("save")
            self.plotter.save_projection(
                pixel_points,
                f"/home/pol/projection_debug/{rospy.Time.to_sec(img.header.stamp):.7f}.png",
                cv_img,
            )
        u, v, _ = pixel_points
        label = cv_mask[..., 2][u.astype(int), v.astype(int)].astype(int)
        labelled_points = np.vstack([visible_points, label]).T

        return PointCloud(points.frame_id, labelled_points)

    def _get_moving_tracks(self, speed_threshold=0.5, visualize=False):
        tracker_iter = self.tracker.get_tracker()
        track_dict = {ts: tr for ts, tr in tracker_iter}
        boxes = []
        tracks = []
        if track_dict:
            tr = max(track_dict, key=track_dict.get)
            current_tracks = track_dict[tr]
            if current_tracks:
                for track in current_tracks:
                    c = np.array([track.state_vector[0], track.state_vector[2], track.state_vector[4]])
                    s = np.array([track.state_vector[6], track.state_vector[7], track.state_vector[8]])
                    speed = sqrt(track.state_vector[1] ** 2 + track.state_vector[3] ** 2 + track.state_vector[5] ** 2)
                    print(f"{track.id} speed: {speed: 6.4f}")
                    if visualize:
                        bbox = visualization.generate_bbox_marker(c, quaternion.one, s, "map" if self.use_world_frame else "os_sensor")
                        bbox.label = 0
                    if speed > speed_threshold or track.id in self.moved_tracks:
                        self.moved_tracks.add(track.id)
                        tracks.append((c, s))
                        bbox.label = 1
                    boxes.append(bbox)
            return tracks, boxes

    def filter_moving_objects(self, full_cloud, no_ground_cloud, mask, img=None):
        start = time.time()
        no_ground_points = utils.numpify_pointcloud(no_ground_cloud)
        full_points = utils.numpify_pointcloud(full_cloud)
        if self.use_world_frame:
            tf = self.tfBuffer.lookup_transform("map", "os_sensor", rospy.Time())
            T_W_L = ros_numpy.numpify(tf.transform)
            no_ground_points = projection_utils.transform_pointcloud_lidar_to_world(no_ground_points, T_W_L)
            full_points = projection_utils.transform_pointcloud_lidar_to_world(full_points, T_W_L)

        labelled_points = self._label_camera_points(no_ground_points, mask, img)
        clusters = self._get_clusters(no_ground_points)
        labelled_clusters = utils.label_clusters(clusters, labelled_points)

        detection_boxes = self._add_detections(labelled_clusters, visualize=True)

        trackstart = time.time()
        tracks, tracker_boxes = self._get_moving_tracks(visualize=True)
        tracktime = time.time() - trackstart

        for center, size in tracks:
            full_points = utils.crop_pointcloud_boundingbox(full_points, center, size, 0.5)

        detectionBboxes = BoundingBoxArray(boxes=detection_boxes)
        trackerBboxes = BoundingBoxArray(boxes=tracker_boxes)
        if self.use_world_frame:
            detectionBboxes.header.frame_id = "map"
            trackerBboxes.header.frame_id = "map"
        else:
            detectionBboxes.header.frame_id = "os_sensor"
            trackerBboxes.header.frame_id = "os_sensor"
        self.bboxPub.publish(detectionBboxes)
        self.trackerPub.publish(trackerBboxes)

        if self.debug:
            self._publish_clusters(labelled_clusters, no_ground_cloud.header.stamp)
        self._publish_pointcloud(full_points, no_ground_cloud.header.stamp)
        print(f"total: {(time.time() - start)*1000: 7.2f}ms; tracking: {tracktime*1000: 7.2f}ms")
