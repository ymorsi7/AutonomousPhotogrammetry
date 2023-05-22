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

# Camera object
class CoordinateFrame:
    def __init__(self, scale=0.25):
        # self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(scale)
        corners = np.array([[0., 0., 0.], [0., 1., 0.], [1., 1., 0.], [1., 0., 0.], [0., 0., 1.], [0., 1., 1.], [1., 1., 1.], [1., 0., 1.]])
        vertices = np.array(list(map(lambda n: n - [.5, .5, 0] if n[2] == 1. else (n - [.5, .5, 0]) * .5, corners))) * .1
        colors = np.array(list(map(lambda n: np.array([0.960, 0.0192, 0.270]) * (.7 + .3 * n[0]) * (1. - .3 * n[1]) * (.7 + .3 * n[2]), corners)))
        quads = np.array([[0, 1, 2, 3], [0, 4, 5, 1], [1, 5, 6, 2], [2, 6, 7, 3], [0, 3, 7, 4], [4, 7, 6, 5]])
        triangles = []
        for quad in quads:
            triangles.append([quad[0], quad[1], quad[2]])
            triangles.append([quad[2], quad[3], quad[0]])
        triangles = np.array(triangles)
        self.frame = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangles))
        self.frame.vertex_colors = o3d.utility.Vector3dVector(colors)
        self.camToWorld = np.identity(4)

    def updateWorldPose(self, camToWorld):
        prevWorldToCam = invert_se3(self.camToWorld)
        prevToCurrent = np.matmul(camToWorld, prevWorldToCam)
        self.frame.transform(prevToCurrent)
        self.camToWorld = camToWorld

lass Open3DVisualization:
    def __init__(self, voxelSize, cameraManual, cameraSmooth, colorOnly, trajectory=False):
        self.shouldClose = False
        self.cameraFrame = CoordinateFrame()
        self.trajectory = Trajectory() if trajectory else None
        self.pointClouds = {}
        self.voxelSize = voxelSize
        self.cameraFollow = not cameraManual
        self.cameraSmooth = cameraSmooth
        self.colorOnly = colorOnly
        self.prevPos = None
        self.prevCamPos = None

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(self.cameraFrame.frame, reset_bounding_box=False)
        if self.trajectory: self.vis.add_geometry(self.trajectory.cloud)
        self.viewControl = self.vis.get_view_control()
        renderOption = self.vis.get_render_option()
        renderOption.point_size = 3
        renderOption.light_on = False
        self.viewControl.set_zoom(0.3)

    def run(self, cameraPoses):
        print("Close the window to stop mapping")

        while not self.shouldClose:
            self.shouldClose = not self.vis.poll_events()

            # Update camera coordinate axes
            self.vis.update_geometry(self.cameraFrame.frame)

            # Update trajectory
            if self.trajectory: self.vis.update_geometry(self.trajectory.cloud)

            # Update point clouds (add, move, remove)
            for pcId in list(self.pointClouds.keys()):
                pc = self.pointClouds[pcId]

                if pc.status == Status.VALID:
                    continue

                elif pc.status == Status.NEW:
                    reset = len(self.pointClouds) == 1
                    self.vis.add_geometry(pc.cloud, reset_bounding_box=reset)
                    pc.status = Status.VALID

                elif pc.status == Status.UPDATED:
                    self.vis.update_geometry(pc.cloud)
                    pc.status = Status.VALID

                elif pc.status == Status.REMOVED:
                    self.vis.remove_geometry(pc.cloud, reset_bounding_box=False)
                    del self.pointClouds[pcId]

            self.vis.update_renderer()
            time.sleep(0.01)

        self.vis.destroy_window()

    def updateCameraFrame(self, cameraPoses):
        for pose in cameraPoses:
            self.cameraFrame.updateWorldPose(pose)

            if self.trajectory: self.trajectory.addPosition(pose[0:3, 3])

            if self.cameraFollow:
                pos = pose[0:3, 3]
                forward = pose[0:3, 2]
                upVector = np.array([0, 0, 1])
                camPos = pos - forward * 0.1 + upVector * 0.05

                if self.cameraSmooth and self.prevPos is not None:
                    alpha = np.array([0.01, 0.01, 0.001])
                    camPos = camPos * alpha + self.prevCamPos * (np.array([1, 1, 1])  - alpha)
                    pos = pos * alpha + self.prevPos * (np.array([1, 1, 1]) - alpha)

                self.prevPos = pos
                self.prevCamPos = camPos

                viewDir = pos - camPos
                viewDir /= np.linalg.norm(viewDir)
                leftDir = np.cross(upVector, viewDir)
                upDir = np.cross(viewDir, leftDir)
                self.viewControl.set_lookat(pos)
                self.viewControl.set_front(-viewDir)
                self.viewControl.set_up(upDir)

    def containsKeyFrame(self, keyFrameId):
        return keyFrameId in self.pointClouds

    def addKeyFrame(self, keyFrameId, keyFrame, cameraPoses):
        camToWorld = cameraPoses[keyFrameId]
        pc = PointCloud(keyFrame, self.voxelSize, self.colorOnly, camToWorld)
        self.pointClouds[keyFrameId] = pc

    def updateKeyFrame(self, keyFrameId, keyFrame, cameraPoses):
        camToWorld = cameraPoses[keyFrameId]
        pc = self.pointClouds[keyFrameId]
        pc.updateWorldPose(camToWorld)
        pc.status = Status.UPDATED

    def removeKeyFrame(self, keyFrameId):
        pc = self.pointClouds[keyFrameId]
        pc.status = Status.REMOVED




