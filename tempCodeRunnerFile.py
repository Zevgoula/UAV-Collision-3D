import time
from vvrpywork.constants import Key, Mouse, Color
from vvrpywork.scene import Scene3D, get_rotation_matrix, world_space
from vvrpywork.shapes import (
    Point3D, Line3D, Arrow3D, Sphere3D, Cuboid3D, Cuboid3DGeneralized,
    PointSet3D, LineSet3D, Mesh3D
)

import heapq
import numpy as np
import utility as u
import open3d as o3d    

WIDTH = 1000
HEIGHT = 800

COLORS = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA, Color.YELLOWGREEN, Color.CYAN, Color.BLACK, Color.WHITE, Color.DARKGREEN]

class UAV(Scene3D):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, "UAV", output=True, n_sliders=4)
        self.reset()
    
    def reset(self):        
        self.n = 5  # number of UAVs
        if hasattr(self, 'geometries') and self.geometries != {}:
            self.removeAllGeometry()

        if hasattr(self, 'uavs') and self.uavs != {}:
            self.removeUAVs()

        self.uavs = {}
        self.geometries = {}
        self.aabb = {}
        self.kdop = {}
        self.plane = {}
        self.createPlane(self.n)
        
        
    def on_key_press(self, symbol, n):
        if symbol == Key.U:     #add uavs
            if self.uavs == {}:
                print("Adding UAVS...")
                self.addUAVs(self.n)
            else:
                print("Removing UAVS...")
                self.removeUAVs()
                if self.aabb != {}:
                    print("Removing AABB...")
                    self.remove_all_aabb()
            
        if symbol == Key.P:     #add planes
            if self.plane == {}:       
                print("Creating Plane...")
                self.createPlane(self.n)
            else:
                print("Removing Plane...")
                self.removePlane()
        
        if symbol == Key.A:     #add aabb
            if self.uavs == {}: 
                print("Please add UAVs first")
            else:
                if self.aabb == {}:
                    print("Creating AABB...")
                    self.create_all_AABBs()
                    self.show_aabbs(self.aabb)
                else:
                    print("Removing AABB...")
                    self.remove_all_aabb()
                    
        # add axis point
        if symbol == Key.X:
            point = Point3D([0, 0, 0], size=5, color=Color.RED)
            self.addShape(point, f"point")
            cuttingplane = self.createRotatedCuboid(point, 45, np.array([0, 1, 0]))
            self.addShape(cuttingplane, f"cuttingplane")
                        
            
        # if symbol == Key.K:     #add kdop
        #     if self.aabb == {}:
        if symbol == Key.K:     #add kdop
            for key in self.aabb.keys():
                corners = self.getAllCuboidCorners(self.aabb[key])
                for i in range(8):
                    cuttingplane = self.createRotatedCuboid(corners[i], 90, np.array([0, 0, 0]))
                    self.addShape(cuttingplane, f"cuttingplane_{key}_{i}")
            
        if symbol == Key.R:
            self.reset()

            
            
    def addUAVs(self, n: int):
        '''add all the UAVs to the scene
         
        Args: n : int: number of UAVs to be added
        Returns: None   
        '''
        print("addUAVs")
        # Limit the number of UAVs to the number of colors
        if n > len(COLORS):
            n = len(COLORS)
            print("Number of UAVs is limited to ", len(COLORS))
            
        # Add UAVs to the scene
        for i in range(n):
            print(i)
            mesh = Mesh3D("resources/uav3.obj", color=COLORS[i])
            
            mesh = u.unit_sphere_normalization(mesh)
            mesh = u.randomize_mesh_position(mesh)
            
            self.addShape(mesh, f"uav{i+1}")
            self.uavs[f"uav{i+1}"] = mesh
    
    def createPlane(self, n: int):
        '''Create a plane with n x n grids
            args: n: int: the number of grids in x and y direction
            returns: none
        '''     
        for i in range(n):
            for j in range(n):
                # Calculate the starting and ending points of the cuboid
                start_point = np.array([i * 2, 0, j * 2])
                end_point = np.array([(i + 1) * 2, 0.5, (j + 1) * 2])
                
                # Create a cuboid for each grid with alternating colors
                color = Color.WHITE if (i + j) % 2 == 0 else Color.RED
                cuboid = Cuboid3DGeneralized(
                    Cuboid3D(
                        start_point, 
                        end_point, 
                        width=1, 
                        color=color, 
                        filled=True
                    )
                )
                self.addShape(cuboid, f"pl_cuboid_{i}_{j}")
                self.plane[f"pl_cuboid_{i}_{j}"] = cuboid
        


    def removeUAVs(self):
        '''remove all the UAVs from the scene
         
        Args: None
        Returns: None   
        '''
        # Clear the UAVs from the dictionary AND the scene
        for key in self.uavs.keys():
            self.removeShape(key)
        self.uavs = {}
            
    def removePlane(self):
        '''remove the plane from the scene
         
        Args: None
        Returns: None   
        '''
        # Clear the plane from the scene
        for key in self.plane.keys():
            self.removeShape(key)
        self.plane = {}
        
    def removeAllGeometry(self):
        '''remove all the geometries from the scene'''
        for key in self.geometries.keys():
            self.removeShape(key)
        self.geometries = {}
      
    def create_all_AABBs(self): 
        '''add all the aabb to the scene
        args: None
        returns: None
        '''
        i = 0
                
        for mesh in self.uavs.values():
            self.aabb_from_mesh(mesh, i)
            i+=1
        
    def aabb_from_mesh(self, mesh, i):
        # find and display the bounding box aabb, for a mesh
        aabb = (u.mesh_to_o3d(mesh)).get_axis_aligned_bounding_box()
        aabbCuboid = u.aabb_to_cuboid(aabb, color=Color.RED)
        self.aabb[f"aabbCuboid{i+1}"] = aabbCuboid
        print("All cuboid corners: ", self.getAllCuboidCorners(aabbCuboid))
        
        
    def show_aabbs(self, aabb_dict):
        '''show all the aabbs in the scene
        args: aabb_dict: dict: a dictionary containing all the aabbs
        returns: None
        '''
        for key, aabb in aabb_dict.items():
            self.addShape(aabb, key)
    
    def remove_all_aabb(self):
        '''remove the aabb from the scene
         
        Args: None
        Returns: None   
        '''
        # Clear the aabb from the scene
        for key in self.aabb.keys():
            if key.startswith('aabbCuboid'):
                self.removeShape(key)
        self.aabb = {}
        
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
        return corners
        
    def createRotatedCuboid(self, point: Point3D, angle, axis: list):
        '''create a rotated cuboid
        args: point: Point3D object: the center point of the cuboid
              normal: np.array: the normal of the plane
        returns: Cuboid3DGeneralized object: the rotated cuboid    
        '''
        p1 = Point3D([point.x - 1, point.y, point.z - 1])
        p2 = Point3D([point.x + 1, point.y, point.z + 1])
        cuboid = Cuboid3D(p1, p2, color=Color.BLUE, filled=True)
        cuboid = Cuboid3DGeneralized(cuboid)
        cuboid.rotate(angle, axis)
    
        return cuboid
        
        
    def addcuttingPlanes(self, corners, mesh, aabb):
        '''add cutting planes to the scene
        args: corners: list: the corners of the aabb
              mesh: Mesh3D object: the mesh to be cut
              aabb: Cuboid3D object: the aabb to be cut
        returns: None
        '''
        # get the normal of the plane
        normal = np.array([1, 0, 0])
        
        # get the distance of the plane from the origin
        d = np.dot(normal, np.array([corners[0].x, corners[0].y, corners[0].z]))
        
        # get the projected points of the corners of the aabb on the plane
        projected_points = [self.project_point_onto_plane(corner, normal, d) for corner in corners]
        
        # get the center of the aabb
        center = np.mean(projected_points, axis=0)
        
        # get the normal of the plane
        normal = np.array([1, 0, 0])
        
        # get the distance of the plane from the origin
        d = np.dot(normal, center)
        
        # create the plane
        plane = self.createPlaneFromNormalAndDistance(normal, d)
        
        # get the rotation matrix
        rotation_matrix = get_rotation_matrix(np.array([1, 0, 0]), normal)
        
        # rotate the plane
        plane.rotate(rotation_matrix)
        
        # add the plane to the scene
        self.addShape(plane, "plane")
        
        # cut the mesh with the plane
        cut_mesh = self.cut_mesh_with_plane(mesh, plane)
        
        # add the cut mesh to the scene
        self.addShape(cut_mesh, "cut_mesh")
    
    
    def project_point_onto_plane(self, point: Point3D, normal: np.array, d: float):
        '''project a point onto a plane
        args: point: Point3D object: the point to be projected
                normal: np.array: the normal of the plane
                d: float: the distance of the plane from the origin
        returns: Point3D object: the projected point
        '''
        mypoint = np.array([point.x, point.y, point.z])     #3dpoint to array
        # plane equation: normal[0] * x + normal[1] * y + normal[2] * z = d
        distance_to_plane = np.dot(normal, mypoint) - d/np.linalg.norm(normal)
        projected_point = mypoint - distance_to_plane * normal
        
        return projected_point
    
        
if __name__ == "__main__":
    scene = UAV()
    scene.mainLoop()    