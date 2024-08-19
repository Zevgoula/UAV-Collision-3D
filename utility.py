import open3d as o3d
import numpy as np
import random
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)
import random


def unit_sphere_normalization(mesh: Mesh3D):
        '''Normalize the mesh (make it smaller) to fit into the unit sphere 
        
        Args: mesh: Mesh3D: the mesh to be normalized
            
        Returns: Mesh3D: the normalized mesh
        
        '''
        mesh.vertices = np.array(mesh.vertices)
        max_len = np.max(np.linalg.norm(mesh.vertices, axis=1))
        mesh.vertices /= max_len
        return mesh
    
def randomize_mesh_position(mesh: Mesh3D):
        '''Randomize the position of the mesh
        
        Args: mesh: Mesh3D: the mesh to be randomized
            
        Returns: None
        
        '''
        # Randomize the position of the mesh within [-3, 3] in x, y, z
        new_position = np.array([(random.random() * 14), (random.random() * 10 +6),(random.random() * 14)])
        mesh.vertices += new_position
        return mesh