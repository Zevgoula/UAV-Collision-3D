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
    new_position = np.array([(random.random() * 3), (random.random() * 2 +2),(random.random() * 6)])
    mesh.vertices += new_position
    return mesh, new_position

def randomize_mesh_rotation(mesh: Mesh3D):
    '''Randomize the rotation of the mesh
    Args: mesh: Mesh3D: the mesh to be rotated
    Returns: None
    
    '''
    # Randomize the rotation of the mesh
    rotation = get_rotation_matrix(random.uniform(0, 360), np.array([random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1)]))
    print("Rotation matrix: ", rotation)
    mesh.vertices = np.dot(mesh.vertices, rotation)
    
    return mesh

def o3d_to_mesh(o3d_mesh:o3d.geometry.TriangleMesh) -> Mesh3D:
    '''Converts an Open3D mesh to a Mesh3D object.

    Args:
        o3d_mesh: The Open3D mesh

    Returns:
        mesh: The Mesh3D object
    '''
    mesh = Mesh3D()
    mesh.vertices = np.array(o3d_mesh.vertices)
    mesh.triangles = np.array(o3d_mesh.triangles)

    return mesh
    
def mesh_to_o3d(mesh:Mesh3D) -> o3d.geometry.TriangleMesh:
    '''Converts a Mesh3D object to an Open3D mesh.

    Args:
        mesh: The Mesh3D object

    Returns:
        o3d_mesh: The Open3D mesh
    '''
    o3d_mesh = o3d.geometry.TriangleMesh()
    o3d_mesh.vertices = o3d.utility.Vector3dVector(mesh.vertices)
    o3d_mesh.triangles = o3d.utility.Vector3iVector(mesh.triangles)

    return o3d_mesh

def get_aabb(mesh):
    """
    Computes the axis-aligned bounding box (AABB) of a mesh or point cloud.
    
    Parameters:
    - mesh: Open3D TriangleMesh or PointCloud object
    
    Returns:
    - aabb: Open3D AxisAlignedBoundingBox object
    """
    if isinstance(mesh, o3d.geometry.TriangleMesh):
        # For a mesh, compute the AABB using the vertices
        vertices = np.asarray(mesh.vertices)
    elif isinstance(mesh, o3d.geometry.PointCloud):
        # For a point cloud, compute the AABB using the points
        vertices = np.asarray(mesh.points)
    else:
        raise ValueError("Input must be either an Open3D TriangleMesh or PointCloud object.")
    
    # Compute the minimum and maximum coordinates along each axis
    min_coords = np.min(vertices, axis=0)
    max_coords = np.max(vertices, axis=0)
    
    # Create the AxisAlignedBoundingBox object
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_coords, max_coords)
    
    return aabb

from vvrpywork.shapes import LineSet3D

def aabb_to_cuboid(aabb, color=Color.RED):
    """
    Converts an axis-aligned bounding box (AABB) to a cuboid for visualization.
    
    Parameters:
    - aabb: Open3D AxisAlignedBoundingBox object
    - color: Color object (optional)
    
    Returns:
    - cuboid: Cuboid3D object representing the AABB
    """
    # Get the minimum and maximum coordinates of the AABB
    min_coords = np.array(aabb.get_min_bound())
    max_coords = np.array(aabb.get_max_bound())
    
    
    # Create the cuboid
    cuboid = Cuboid3D(min_coords, max_coords, color=color, width=2)
    
    return cuboid


def getAllCuboidCorners(self, cuboid: Cuboid3D):
        '''get all the points of the cuboid
        args: cuboid: Cuboid3D object
        returns: list of all the points of the cuboid
        '''
        
        #top max and bottom min points of the cuboid
        max_point = np.array([cuboid.x_max, cuboid.y_max, cuboid.z_max])
        min_point = np.array([cuboid.x_min, cuboid.y_min, cuboid.z_min])
        
        #get all the corners of the cuboid
        bottom_corners = [Point3D([min_point[0], min_point[1], min_point[2]], size= 5, color=Color.BLACK), Point3D([max_point[0], min_point[1], min_point[2]], size= 5, color=Color.GRAY), Point3D([max_point[0], min_point[1], max_point[2]], size= 5, color=Color.BLUE), Point3D([min_point[0], min_point[1], max_point[2]], size= 5, color=Color.MAGENTA)]
        top_corners = [Point3D([max_point[0], max_point[1], max_point[2]], size= 5, color=Color.RED), Point3D([min_point[0], max_point[1], max_point[2]], size= 5, color=Color.ORANGE), Point3D([min_point[0], max_point[1], min_point[2]], size= 5, color=Color.YELLOW), Point3D([max_point[0], max_point[1], min_point[2]],size= 5, color=Color.GREEN)]
        

        for i in range(4):
            randomint = np.random.randint(0, 10000)
            self.addShape(bottom_corners[i], f"bottom_corners_{randomint}")
            self.addShape(top_corners[i], f"top_corners_{randomint}")
        
        corners = bottom_corners + top_corners
        return corners, bottom_corners, top_corners