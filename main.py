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

import utility as u

WIDTH, HEIGHT = 800, 600


class Project(Scene3D):

    def __init__(self):
        super().__init__(WIDTH, HEIGHT, "Project")
        self.show_mesh()
        

    def show_mesh(self):
        '''add all the meshes to the scene
         
        Args: None
        Returns: None   
            '''
        self.createPlane(6)
        self.addUAVs()
        
        
        
    
    def keyControl(self, key):
        '''Control the scene using keyboard
        args: key: the key pressed
        returns: None
        '''
        if key == Key.U:
            self.addUAVs()
        elif key == Key.P:
            self.createPlane(6) 
        elif key == Key.C:
            self.clearShapes()
        elif key == Key.Q:
            self.close()
            
            
    def addUAVs(self):
        '''add all the UAVs to the scene
         
        Args: None
        Returns: None   
        '''
        color_list = [Color.BLACK, Color.BLUE, Color.GREEN, Color.YELLOW, Color.CYAN, Color.MAGENTA]
        for i in range(6):
            mesh = Mesh3D("resources/uav3.obj", color=color_list[i])
            mesh = u.unit_sphere_normalization(mesh)
            mesh = u.randomize_mesh_position(mesh)
            self.addShape(mesh, f"uav{i+1}")
        
    
    def createPlane(self, n: int):
        '''Create a plane with n x n grids
            args: n: int: the number of grids in x and y direction
            returns: none
        '''     
        for i in range(n):
            for j in range(n):
                # Calculate the starting and ending points of the cuboid
                start_point = np.array([i * 2, 0, j * 2])
                end_point = np.array([(i + 1) * 2, 2, (j + 1) * 2])
                
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
                self.addShape(cuboid, f"cuboid_{i}_{j}")
                
                
        
        
if __name__ == "__main__":
    scene = Project()
    scene.mainLoop()    