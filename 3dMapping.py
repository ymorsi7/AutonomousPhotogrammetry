"""
Visualize 3D point cloud of the environment in real-time, or playback your recordings and view their 3D point cloud.
Press 'H' to view Open3D point cloud viewer options.

Requirements: pip install -v open3d==0.16.0

Note: Most recent open3d version (0.17.0) has a bug with Visualizer::get_view_control() so version 0.16.0 is recommended.
"""
import depthai
import open3d as o3d
import numpy as np
import threading
import time
import os
from common.deserialize_output import input_stream_reader, MockVioOutput, MockMapperOutput
from enum import Enum

# Status for point clouds (for updating Open3D renderer).
class Status(Enum):
    VALID = 0
    NEW = 1
    UPDATED = 2
    REMOVED = 3

def invert_se3(a):
    b = np.eye(4)
    b[:3, :3] = a[:3, :3].transpose()
    b[:3, 3] = -np.dot(b[:3, :3], a[:3, 3])
    return b

def blend(a, b, m):
    return [a[0] * (1 - m) + b[0] * m, a[1] * (1 - m) + b[1] * m, a[2] * (1 - m) + b[2] * m]

class Trajectory:
    def __init__(self):
        self.maxPoints = 1000
        self.points = []
        self.colors = []
        self.cloud = o3d.geometry.PointCloud()
        for i in range(self.maxPoints):
            self.colors.append(blend([0.960, 0.0192, 0.270], [.217, .009, .9], i / (self.maxPoints - 1)))
            self.points.append([0,0,0])
        self.cloud.colors = o3d.utility.Vector3dVector(self.colors)

    def addPosition(self, pos):
        while (len(self.points) < self.maxPoints): self.points.append(pos)
        self.points.insert(0, pos)
        self.points = self.points[:self.maxPoints]
        self.cloud.points = o3d.utility.Vector3dVector(self.points)

# Wrapper around Open3D point cloud, which helps updating its world pose.
class PointCloud:
    def __init__(self, keyFrame, voxelSize, colorOnly, cameraPose):
        self.status = Status.NEW
        self.camToWorld = cameraPose
        self.cloud = self.__getKeyFramePointCloud(keyFrame, voxelSize, colorOnly)

    def __getKeyFramePointCloud(self, keyFrame, voxelSize, colorOnly):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(keyFrame.pointCloud.getPositionData())

        if keyFrame.pointCloud.hasColors():
            colors = keyFrame.pointCloud.getRGB24Data() * 1./255
            cloud.colors = o3d.utility.Vector3dVector(colors)

        if keyFrame.pointCloud.hasNormals():
            cloud.normals = o3d.utility.Vector3dVector(keyFrame.pointCloud.getNormalData())

        if cloud.has_colors() and colorOnly:
            # Filter points without color
            colors = np.asarray(cloud.colors)
            pointsWithColor = []
            for i in range(len(colors)):
                if colors[i, :].any():
                    pointsWithColor.append(i)
            cloud = cloud.select_by_index(pointsWithColor)

        if voxelSize > 0:
            cloud = cloud.voxel_down_sample(voxelSize)

        return cloud

    def updateWorldPose(self, camToWorld):
        prevWorldToCam = invert_se3(self.camToWorld)
        prevToCurrent = np.matmul(camToWorld, prevWorldToCam)
        self.cloud.transform(prevToCurrent)
        self.camToWorld = camToWorld
