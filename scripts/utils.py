import open3d as o3d
import numpy as np
import cv2
import json
import time
import random

def np_to_pcd(points, colors=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
    else:
        pcd.paint_uniform_color([0.5, 0.5, 0.5])
    return pcd


# depth image to pointcloud
def depth_to_pointcloud(depth, camera_matrix, color=None, mask=None):
    if mask is None:
        mask = np.ones(depth.shape, dtype=np.uint8)
    if color is None:
        color = np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

    pixels = np.array(np.where(mask > 0)).T
    z = depth[pixels[:, 0], pixels[:, 1]]
    x = (pixels[:, 1] - camera_matrix[0, 2]) * z / camera_matrix[0, 0]
    y = (pixels[:, 0] - camera_matrix[1, 2]) * z / camera_matrix[1, 1]
    points = np.stack([x, y, z], axis=1)

    colors = color[pixels[:, 0], pixels[:, 1]]

    return np_to_pcd(points, colors)

# depth image to pointcloud
def depth_to_points(depth, camera_matrix, upleft_xy, patch_size):
    h = w = patch_size
    z = depth[upleft_xy[1]:upleft_xy[1]+h, upleft_xy[0]:upleft_xy[0]+w].ravel()
    ind = np.where(z > 0)
    pix_x = np.tile(np.arange(w), h) + upleft_xy[0]
    pix_y = np.repeat(np.arange(h), w) + upleft_xy[1]
    x = (pix_x - camera_matrix[0, 2]) * z / camera_matrix[0, 0]
    y = (pix_y - camera_matrix[1, 2]) * z / camera_matrix[1, 1]
    points = np.stack([x, y, z], axis=1)[ind]
    return points