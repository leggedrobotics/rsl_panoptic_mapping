import numpy as np
from .utils import PointCloud


def project_pointcloud_to_cam(points, transform, camera_intrinsics):

    points = np.insert(points, 3, 1, axis=1).T
    p = camera_intrinsics @ transform
    proj_points = p @ points

    return proj_points


def project_cam_to_pointcloud(pts, transform, camera_intrinsics):
    z = pts[2, :]
    uv = pts[:2]
    uv_hom = np.vstack((uv, np.ones((uv.shape[1],)), 1 / z))  # add disparity
    P_full = np.insert(camera_intrinsics, 3, values=[0, 0, 0, 1], axis=0)  # extend the projection matrix to full rank
    P_inv = np.linalg.inv(P_full)
    xyz = np.multiply(P_inv @ uv_hom, z)
    xyz_lidar = transform * xyz
    return xyz_lidar


def transform_pointcloud_lidar_to_world(pcl, transform):
    pts = np.insert(pcl.cloud, 3, 1, axis=1).T
    pts = transform @ pts

    return PointCloud("map", pts[:3, :].T)
