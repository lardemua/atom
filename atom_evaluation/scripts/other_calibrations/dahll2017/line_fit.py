#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import random

view = {
    "class_name": "ViewTrajectory", "interval": 29, "is_loop": False,
    "trajectory":
    [{"boundingbox_max": [1.4881374835968018, 1.0, 1.0],
      "boundingbox_min": [-0.059999999999999998, -0.83004301786422729, -0.40193906426429749],
      "field_of_view": 60.0,
      "front": [-0.95711769604701769, 0.0051956268381449571, 0.28965275999963758],
      "lookat": [0.72128191884594961, -0.50680060371821634, 0.18311132966112323],
      "up": [0.28882815786397104, -0.060368480020593036, 0.95547576727246641],
      "zoom": 0.61999999999999988}],
    "version_major": 1, "version_minor": 0}



# Generate some sample data
n = 100
xyz_coords = np.zeros((3,n))

for i in range(n):
    xyz_coords[0,i] = i
    xyz_coords[2,i] += 20*(random.random())


# Convert numpy array to Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz_coords.T)

# Convert point cloud to numpy array
points = np.asarray(pcd.points)

# Compute centroid of the point cloud
centroid = np.mean(points, axis=0)

# Perform principal component analysis (PCA) to get the direction of the line
covariance_matrix = np.cov(points.T)
eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)
line_direction = eigenvectors[:, np.argmax(eigenvalues)]

pc0 = o3d.geometry.PointCloud()
pc0.points.append(centroid)
pc1 = o3d.geometry.PointCloud()
pc1.points.append(centroid+300*line_direction)

# Define a line using centroid and direction
line = o3d.geometry.LineSet.create_from_point_cloud_correspondences(
    pc0,
    pc1,
    [(0,0)]
)

# Visualization
entities = []

entities.append(pcd)
entities.append(line)

o3d.visualization.draw_geometries(
    entities,
    zoom=view['trajectory'][0]['zoom'],
    front=view['trajectory'][0]['front'],
    lookat=view['trajectory'][0]['lookat'],
    up=view['trajectory'][0]['up']
    )