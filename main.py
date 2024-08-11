from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)

from matplotlib import colormaps as cm
import numpy as np
import scipy
from scipy import sparse
import time

import struct


WIDTH = 1000
HEIGHT = 800

class Project(Scene3D):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, "Project", output=True, n_sliders=2)
        self.reset_mesh()
        
    def reset_mesh(self):

        # Choose mesh
        # self.mesh = Mesh3D.create_bunny(color=Color.GRAY)
         
        # self.mesh = Mesh3D("resources/bun_zipper_res2.ply", color=Color.GRAY)
        self.mesh = Mesh3D("resources/uav1.obj", color=Color.GRAY)

        self.mesh.remove_duplicated_vertices()
        self.mesh.remove_unreferenced_vertices()
        vertices = self.mesh.vertices
        vertices -= np.mean(vertices, axis=0)
        distanceSq = (vertices ** 2).sum(axis=-1)
        max_dist = np.sqrt(np.max(distanceSq))
        self.mesh.vertices = vertices / max_dist
        self.removeShape("mesh")
        self.addShape(self.mesh, "mesh")

        self.wireframe = LineSet3D.create_from_mesh(self.mesh)
        self.removeShape("wireframe")
        self.addShape(self.wireframe, "wireframe")
        self.show_wireframe = True

        self.eigenvectors = None
        self.eigenvector_idx = 0

        
        
if __name__ == "__main__":
    app = Project()
    app.mainLoop()