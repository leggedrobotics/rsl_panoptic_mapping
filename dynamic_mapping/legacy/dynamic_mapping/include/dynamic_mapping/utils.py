import open3d as o3d
import quaternion
import numpy as np
from . import boundingbox
import ros_numpy
from dataclasses import dataclass
from message_filters import ApproximateTimeSynchronizer
import rospy


@dataclass
class PointCloud:
    frame_id: str
    cloud: np.ndarray


def generate_3d_bbox(cluster_pts, obb=False):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(cluster_pts)
    if obb:
        box = pc.get_oriented_bounding_box(robust=True)
        q = quaternion.from_rotation_matrix(box.R).normalized()
        size = box.extent
    else:
        box = pc.get_axis_aligned_bounding_box()
        q = quaternion.one
        size = box.get_extent()
    center = box.get_center()

    return center, q, size


def adjust_labels_in_cluster(cluster, split_on_label=False):
    labels = np.asarray(cluster[:, 3].flatten())
    label_counts = dict(zip(*np.unique(labels, return_counts=True)))
    clusters = []
    if label_counts:
        max_label = max(label_counts, key=label_counts.get)
        label_ratios = {k: v / labels.size for k, v in label_counts.items()}
        ratio = label_ratios[max_label]
        if ratio >= 0.8 and ratio < 1.0:
            cluster[:, 3] = max_label
            return [cluster]
        elif split_on_label:
            for label in label_ratios:
                idx = labels == label
                clusters.append(cluster[idx])
        else:
            clusters.append(cluster)
        return clusters


def filter_points_boundingbox(visible_points, bboxes):
    u, v, _ = visible_points
    boxes = np.zeros((1, visible_points.shape[1]), dtype=bool)
    if bboxes.detections:
        bbox_arr = boundingbox.bbox_to_array(bboxes)
        for bbox in bbox_arr:
            bbox_u = np.logical_and(u >= bbox[0], u <= bbox[2])
            bbox_v = np.logical_and(v >= bbox[1], v <= bbox[3])

            box = np.logical_and(bbox_u, bbox_v)
            boxes = np.logical_or(boxes, box)
    return visible_points[:, np.asarray(boxes).ravel()]


def numpify_pointcloud(pc):
    pc_points = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    height = pc_points.shape[0]
    if len(pc_points.shape) > 1:
        width = pc_points.shape[1]
    else:
        width = 1
    points = np.zeros((height * width, 3), dtype=np.float32)
    points[:, 0] = np.resize(pc_points["x"], height * width)
    points[:, 1] = np.resize(pc_points["y"], height * width)
    points[:, 2] = np.resize(pc_points["z"], height * width)

    # remove NaN
    return PointCloud(pc.header.frame_id, points[~np.isnan(points).any(axis=1)])


def label_clusters(clusters, labelled_points, threshold=10):
    labelled_clusters = []
    for cluster in clusters:
        if cluster.cloud.shape[0] < threshold:
            continue
        cluster_idx = np.argwhere(np.isin(labelled_points.cloud[:, :3], cluster.cloud).all(axis=1))
        if cluster_idx.size == cluster.cloud.shape[0]:
            labels = np.asarray(labelled_points.cloud[cluster_idx, 3])
        else:
            labels = np.ones((cluster.cloud.shape[0], 1)) * -1
        labelled_clusters.extend([PointCloud(cluster.frame_id, x) for x in adjust_labels_in_cluster(np.hstack([cluster.cloud, labels]))])

    return labelled_clusters


def crop_pointcloud_boundingbox(pc, center, size, margin=0.2):
    x1, y1, z1 = center - ((size / 2) * (1 + margin))
    x2, y2, z2 = center + ((size / 2) * (1 + margin))

    x, y, z = pc.cloud.T
    crop_x = np.logical_and(x >= x1, x <= x2)
    crop_y = np.logical_and(y >= y1, y <= y2)
    crop_z = np.logical_and(z >= z1, z <= z2)
    crop_xy = np.logical_and(crop_x, crop_y)
    crop = np.logical_and(crop_xy, crop_z)
    pts = pc.cloud[~crop]
    return PointCloud(pc.frame_id, pts)


def filter_visible_points(projected_points, pcl, img_shape):
    x, y, z = projected_points
    u = x / z
    v = y / z
    IMG_H, IMG_W, _ = img_shape
    u_out = np.logical_or(u < 0, u >= IMG_H)
    v_out = np.logical_or(v < 0, v >= IMG_W)
    z_out = np.logical_not(z > 0)
    outlier_uv = np.logical_or(u_out, v_out)
    outlier = np.logical_or(outlier_uv, z_out)
    pixel_points = np.delete(projected_points, np.where(outlier), axis=1)
    pixel_points[:2] /= pixel_points[2, :]
    visible_points = np.delete(pcl, np.where(outlier), axis=0)
    return pixel_points, visible_points.T


class LidarApproximateTimeSynchronizer(ApproximateTimeSynchronizer):
    def __init__(self, fs, queue_size, slop, allow_headerless=False, reset=False, sensor_frequency_Hz=10.0):
        self.sensor_frequency = sensor_frequency_Hz
        super().__init__(fs, queue_size, slop, allow_headerless, reset)

    def add(self, msg, my_queue, my_queue_index=None):
        if msg._type == "sensor_msgs/PointCloud2":
            msg.header.stamp += rospy.Duration().from_sec(0.75 * 1 / self.sensor_frequency)
        super().add(msg, my_queue, my_queue_index)
