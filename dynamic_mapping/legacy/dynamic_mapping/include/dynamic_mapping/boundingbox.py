def bbox_to_array(bboxes):
    res = []
    for box in bboxes.detections:
        boxarr = [
            box.bbox.center.x - box.bbox.size_x / 2.0,
            box.bbox.center.y - box.bbox.size_y / 2.0,
            box.bbox.center.x + box.bbox.size_x / 2.0,
            box.bbox.center.y + box.bbox.size_y / 2.0,
            box.results[0].id,
        ]
        res.append(boxarr)
    return res
