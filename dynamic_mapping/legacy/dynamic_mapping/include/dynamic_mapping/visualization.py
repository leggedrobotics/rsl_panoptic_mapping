from jsk_recognition_msgs.msg import BoundingBox


def generate_bbox_marker(center, q, size, frame):
    cx, cy, cz = center
    sx, sy, sz = size

    bbox = BoundingBox()
    bbox.header.frame_id = frame
    bbox.pose.position.x = cx
    bbox.pose.position.y = cy
    bbox.pose.position.z = cz

    bbox.pose.orientation.x = q.x
    bbox.pose.orientation.y = q.y
    bbox.pose.orientation.z = q.z
    bbox.pose.orientation.w = q.w

    bbox.dimensions.x = sx
    bbox.dimensions.y = sy
    bbox.dimensions.z = sz

    return bbox
