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
import trimesh
import random

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

        self.paused = True
        self.uavs = {}                      #all the uavs in the scene
        self.rotatedUavs = {}               #all the rotated uavs in the scene
        self.geometries = {}                #all the geometries in the scene
        self.aabb = {}                      #aabb for each uav
        self.kdop = {}                      #kdop for each uav
        self.plane = {}                     #plane with n x n grids
        # self.labels = {}                    #labels for the uavs
        self.intersectingPoints = {}        #points where the uavs intersect
        self.movingUavs = {}         # UAVs that are moving
        self.createPlatform(self.n)
        self.fake_aabbs = {}
        
    def on_key_press(self, symbol, n):
        if symbol == Key.U:                 #add uavs
            if self.uavs == {}:
                print("Adding UAVS...")
                self.addUAVs(self.n)
            else:
                print("Removing UAVS...")
                self.removeUAVs(uavs=self.uavs)
                self.remove_all_aabb()
                if self.aabb != {}:
                    print("Removing AABB...")
                    self.remove_all_aabb()
            
        if symbol == Key.P:                 #add planes
            if self.plane == {}:       
                print("Creating Plane...")
                self.createPlatform(self.n)
            else:
                print("Removing Plane...")
                self.removePlane()
        
        if symbol == Key.A:                 #add aabb
            if self.uavs == {} and self.rotatedUavs == {}: 
                print("Please add UAVs first")
            else:
                if self.aabb == {}:
                    print("Creating AABB...")
                    if self.uavs != {}:
                        self.create_all_AABBs(self.uavs)
                    elif self.rotatedUavs != {}:
                        self.create_all_AABBs(self.rotatedUavs)
                    self.show_aabbs(self.aabb)
                else:
                    print("Removing AABB...")
                    self.remove_all_aabb()         
        
        if symbol == Key.K:                 #add kdop
            print("i want to cry")
            
        if symbol == Key.C:                 #collision detection
            if self.uavs == {} and self.rotatedUavs == {}:
                print("Please add UAVs")
            elif self.uavs != {} and self.aabb == {}:
                self.create_all_AABBs(self.uavs)
                self.basicCollisionDetection(self.uavs, self.aabb, "accurate")
            elif self.rotatedUavs != {} and self.aabb == {}:
                self.create_all_AABBs(self.rotatedUavs)
                self.basicCollisionDetection(self.rotatedUavs, self.aabb, "accurate")
            elif self.uavs != {} and self.aabb != {}:
                self.basicCollisionDetection(self.uavs, self.aabb, "accurate")
            elif self.rotatedUavs != {} and self.aabb != {}:
                self.basicCollisionDetection(self.rotatedUavs, self.aabb, "accurate")
            else:
                print("Something went wrong")
                #self.basicCollisionDetection(self.uavs, self.aabb, "aabb")
                # self.basicCollisionDetection(self.uavs, self.aabb, "accurate")
                
        if symbol == Key.R:
            if self.rotatedUavs == {}:
                print("Adding Rotated UAVS...")
                self.addRotatedUavs(self.n)
            else:
                print("Removing Rotated UAVS...")
                self.removeUAVs(uavs=self.rotatedUavs)
                self.remove_all_aabb()
                if self.aabb != {}:
                    print("Removing AABB...")
                    self.remove_all_aabb()
                    
        if symbol == Key.SPACE:
            self.paused = not self.paused
            print("Paused: ", self.paused)  
                
                
    def on_idle(self):
        if not self.paused:
            if self.uavs != {}:
                if self.movingUavs != {}:
                    if self.aabb == {}:
                        try:
                            print("Creating AABBs for collision detection")
                            self.create_all_AABBs(self.uavs)
                        except Exception as e:
                            print("Problem with creating the AABBs. ", e)
                        
                    self.checkMovingCollision()
                    # self.moveUavs
                    # self.avoidCollision(self.uavs, self.aabb)
                else: 
                    print("All UAVs have stopped moving")
                    self.paused = True
                    
            else:
                print("No UAVs in the scene, Press U to add UAVs")

    def avoidCollision(self, uavs, aabbs):
        for key1, value1 in aabbs.items():
            for key2, value2 in aabbs.items():
                if key2>key1:
                    if self.aabbCollision(value1, value2, key1, key2, True):
                        self.movingUavs.pop('uav'+key1[-1])
                        self.movingUavs.pop('uav'+key2[-1])
                        print("moving uavs: ", self.movingUavs)
        
    
    def checkMovingCollision(self):
        '''
            when uavs are moving, check if they collide with each other using the convex hull of the aabb in time t+dt and time t
            input: uavs: dict: dictionary containing all the uavs, aabbs: dict: dictionary containing all the aabbs
            return None
        '''
        chulls = {} 
        velocities2 = self.get_velocities(self.uavs.keys())
        
        
        # "move" the aabbs to the next position 
        self.fake_aabbs = {}
        for i, (aabb_name, aabb_value) in enumerate(self.aabb.items()):
            if 'uav'+str(i+1) in self.movingUavs.keys():
                points_old = self.getAllCuboidCorners(aabb_value)[0]
                points = points_old
                vel = velocities2['uav'+str(i+1)]
                for i in range(len(points)):
                    points[i].x += vel[0]
                    points[i].y += vel[1]
                    points[i].z += vel[2]
                
                min_value = points[0]
                max_value = points[4]
                
                # create cuboid
                print("moving uavs: ", self.movingUavs.keys())
                self.fake_aabbs[aabb_name] = Cuboid3D(min_value, max_value)


            
        # find the convex hull of the aabbs
        for i, aabb1 in enumerate(self.aabb.values()):
            for j, aabb2 in enumerate(self.fake_aabbs.values()):
                if i==j:
                    if 'chull' + str(i) not in chulls:
                        chulls['chull' + str(i)] = [] 
                    chulls['chull'+str(i)].append(self.aabbs_to_chull(aabb1, aabb2))
        
            
        # check if the convex hulls are colliding
        for i, chullvalue1 in enumerate(chulls.values()):
            for j, chullvalue2 in enumerate(chulls.values()):
                
                if i<=j: 
                    continue
                elif i>j:
                    u_var = self.chull_collision(chullvalue1[0], chullvalue2[0])
                    if u_var[0]:
                        print("Collision detected")
                        for k in range(0, len(u_var[1]), 2):
                            
                            self.addShape(Point3D(u_var[1][k], size = 0.5, color=Color.BLACK), f"point{random.randint(0, 10000)}")
                            
                        if 'uav'+str(i+1) in self.movingUavs and 'uav'+str(j+1) in self.movingUavs:
                            self.movingUavs.pop('uav'+str(i+1))
                            self.movingUavs.pop('uav'+str(j+1))
                        print("moving uavs: ", self.movingUavs)
                        break
                    else: 
                        print("No collision detected")
                        
        
        for uav in self.movingUavs.keys():
            # i = int(uavs.split("uav")[1])
            self.moveUav(uav, velocities2[uav])
        return None
            
    

    # checks if 2 chulls are colliding
    def chull_collision(self, chull1, chull2):  
        '''detect collision using convex hull
            chull1 = movingCHull(aabb1 - > aabb1+velocities)
            chull2 = movingCHull(aabb2 - > aabb2+velocities)
            return true if collision is detected
        '''
        return self.mesh_intersection(chull1, chull2) 

        
    
    
    # finds the convex hull of 2 aabbs
    def aabbs_to_chull(self, aabb1, aabb2):
        '''
            input: aabb1, aabb2: Cuboid3D objects
            output: Convex hull of the two aabbs
        '''
        
        points1, _ = self.getAllCuboidCorners(aabb1)
        points2, _ = self.getAllCuboidCorners(aabb2)
        pointlist = []
        # Create a list of all the points
        for point in points1+points2:
            pointlist.append(np.array([point.x, point.y, point.z])) 
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointlist)
        print("chull0 : ", pointlist[0])
        print("point8: ", pointlist[8])
        # Compute the convex hull
        hull, _ = pcd.compute_convex_hull()
        hull = u.o3d_to_mesh(hull)
        
        return hull
        
        
            
    def draw_chull(self, chull, name):
        '''draw the convex hull in the scene
            input: chull: Mesh3D object
            return None
        '''
        try: self.updateShape(name)
        except: self.addShape(chull, name)
        
        
    # returns the velocities dictionary - > key: uav_name, value: [x, y, z]
    def get_velocities(self, uav_names):
        velocities = {}
        for uav in uav_names:
            # x, y, z = 0.08 - random.random() * 0.08, 0.1*(0.7- random.random()), 0.2 -random.random() * 0.2
            x, y, z = 0.1 - random.random() * 0.08, 0.1*(0.7- random.random()), 0.2 -random.random() * 0.2

            velocities[uav] = [x, y, z]
            # velocities['aabbCuboid'+uav[-1]] = [x, y, z]
        return velocities
            
    # moves the uav and the aabb
    def moveUav(self, uav_name, velocity):
        '''move the uav in the scene'''
        uav = self.uavs[uav_name]
        # x, y, z = self.get_velocities(self.uavs.keys())[uav_name]
        x, y, z = velocity
        uav.vertices += np.array([x, y, z])
        i = int(uav_name.split("uav")[1])
        if "aabbCuboid" + str(i) in self.aabb:
            aabb = self.aabb[f"aabbCuboid{i}"]
            aabb.translate([x, y, z])
            self.updateShape(f"aabbCuboid{i}", True)
        self.updateShape(uav_name, True)
                         
    # move only the aabb  
    def moveAabb(self, velocities, aabb_name):
        '''move the aabb in the scene'''
        aabb = self.aabb[aabb_name]
        aabb_generalized = Cuboid3DGeneralized(aabb)
        x, y, z = velocities['uav'+aabb_name[-1]]
        print("x, y, z: ", x, y, z)
        aabb_generalized.translate([x, y, z])
        self.updateShape(aabb_name, True)
        self.fake_aabbs[aabb_name] = aabb_generalized.cuboid
        print("before:", self.aabb[aabb_name].x_min)
        print("after: ", self.fake_aabbs[aabb_name].x_min)

        
    
        
    # add all the rotated UAVs to the scene
    def addRotatedUavs(self, n: int):
        '''add all the rotated UAVs to the scene
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
            mesh = u.randomize_mesh_rotation(mesh)
            mesh, position = u.randomize_mesh_position(mesh)
            
            # label = Label3D(position, f"uav{i+1}", color=Color.BLACK)
            # self.addShape(label, f"label{i+1}")
            self.addShape(mesh, f"uav{i+1}")
            self.rotatedUavs[f"uav{i+1}"] = mesh
            # self.labels[f"label{i+1}"] = label           
                           
    def basicCollisionDetection(self, uavs, aabbs, method):
        if method == "aabb":
            self.aabbCollisionDetection(aabbs, show_intersecting_cuboid=True)
        elif method == "kdop":
            self.kdopCollisionDetection(uavs)
        elif method == "accurate":
            self.accurateCollisionDetection(uavs, aabbs)
        else:
            print("Invalid method")

    def accurateCollisionDetection(self, uavs, aabbs):
        '''detect collision using accurate method
        args: uavs: dict: dictionary containing all the uavs
        returns: None
        '''
        for i, (key, value) in enumerate(list(uavs.items())):
            for j, (key2, value2) in enumerate(list(uavs.items())):
                if j>i:
                    self.accurateCollision(value, value2, aabbs[f"aabbCuboid{i+1}"], aabbs[f"aabbCuboid{j+1}"], key, key2)        
    
    def accurateCollision(self, uav1, uav2, aabb1, aabb2, i, j):
        '''detect collision between two uavs
        args: uav1: Mesh3D object: first uav
              uav2: Mesh3D object: second uav
        returns: None
        '''
        if self.aabbCollision(aabb1, aabb2, i, j, show_intersecting_cuboid=False):
            # if self.kdopCollision(uav1, uav2, i, j):
            collision, points = self.mesh_intersection(uav1, uav2)
            # Get the vertices of the two UAVs
            if collision:
                print(f"ACCURATE Collision detected between {i} and {j}")
                if f"uav{i}" in self.movingUavs:
                    self.movingUavs.pop(f"uav{i}")
                if f"uav{j}" in self.movingUavs:
                    self.movingUavs.pop(f"uav{j}")
                print("moving uavs: ", self.movingUavs) 
                for point in points[::2]:
                    index = random.random()
                    self.addShape(Point3D(point, size=2, color=Color.BLACK), f"point_{index}")
                    self.intersectingPoints[f"point_{index}"] = Point3D(point, size=2, color=Color.BLACK)
            
    def mesh_intersection(self, mesh1, mesh2):
        """Check if two meshes collide using trimesh, an external library
        Inputs:
        - mesh1: the first mesh
        - mesh2: the second mesh
        Outputs:
        - True if the meshes collide, False otherwise
        """
        
        trimesh1 = trimesh.Trimesh(vertices=mesh1.vertices, faces=mesh1.triangles)
        trimesh2 = trimesh.Trimesh(vertices=mesh2.vertices, faces=mesh2.triangles)
        collision_manager = trimesh.collision.CollisionManager()
        collision_manager.add_object("trimesh1", trimesh1)
        collision_manager.add_object("trimesh2", trimesh2)
        
        _, point_objs = collision_manager.in_collision_internal(return_data=True)
        points = []
        for point_obj in point_objs:
            points.append(point_obj.point)

        return collision_manager.in_collision_internal(), points
                                     
    def aabbCollisionDetection(self, aabbs, show_intersecting_cuboid):
        '''detect collision using aabb
        args: uavs: dict: dictionary containing all the uavs
        returns: None
        '''
        for i in aabbs:
            for j in aabbs:
                if j>i:
                    self.aabbCollision(aabbs[i], aabbs[j], i, j, show_intersecting_cuboid)
    
    def aabbCollision(self, aabb1, aabb2, i, j, show_intersecting_cuboid=False):
        '''detect collision between two aabbs
        args: aabb1: Cuboid3D object: aabb of the first uav
              aabb2: Cuboid3D object: aabb of the second uav
        returns: None
        '''
        xmin1 = aabb1.x_min
        xmax1 = aabb1.x_max
        ymin1 = aabb1.y_min
        ymax1 = aabb1.y_max
        zmin1 = aabb1.z_min
        zmax1 = aabb1.z_max
        
        xmin2 = aabb2.x_min
        xmax2 = aabb2.x_max
        ymin2 = aabb2.y_min
        ymax2 = aabb2.y_max
        zmin2 = aabb2.z_min
        zmax2 = aabb2.z_max
        
        if (xmin1 <= xmax2 and xmax1 >= xmin2) and (ymin1 <= ymax2 and ymax1 >= ymin2) and (zmin1 <= zmax2 and zmax1 >= zmin2):
            print(f"AABB Collision detected between {i} and {j}")
            if show_intersecting_cuboid: self.intersectingCuboid(aabb1, aabb2)
            return True
        return False
    
    def intersectingCuboid(self, aabb1, aabb2):
        '''get the intersecting cuboid
        args: aabb1: Cuboid3D object: aabb of the first uav
              aabb2: Cuboid3D object: aabb of the second uav
        returns: None
        '''
        x_min = max(aabb1.x_min, aabb2.x_min)
        x_max = min(aabb1.x_max, aabb2.x_max)
        y_min = max(aabb1.y_min, aabb2.y_min)
        y_max = min(aabb1.y_max, aabb2.y_max)
        z_min = max(aabb1.z_min, aabb2.z_min)
        z_max = min(aabb1.z_max, aabb2.z_max)
        
        intersecting_cuboid = Cuboid3DGeneralized(
            Cuboid3D(
                [x_min, y_min, z_min],
                [x_max, y_max, z_max],
                color=Color.BLUE,
                filled=False
            )
        )
        
        self.addShape(intersecting_cuboid, f"intersecting_cuboid_{aabb1}_{aabb2}")

    def rotatedCuboid(self, center, size, angle, axis):
        '''create a rotated cuboid
        args: center: np.array: center of the cuboid
              size: np.array: size of the cuboid
              angle: float: angle of rotation
              axis: np.array: axis of rotation
        returns: Cuboid3D object
        '''
        # Create a cuboid
        cuboid = Cuboid3DGeneralized(Cuboid3D(Point3D(center - size / 2), Point3D([center[0] + size / 2, center[1] + size / 2, center[2] - size / 2]), color=Color.RED, filled=True))
        
        # Rotate the cuboid
        rotation_matrix = get_rotation_matrix(angle, axis)
        cuboid.rotate(angle, axis)
        
        return cuboid
        
    def split_into_8_regions(vertices, center):
        regions = [[] for _ in range(8)]
        
        for vertex in vertices:
            x, y, z = vertex
            
            # Determine the region based on comparison with the center
            region_idx = (
                (x >= center[0]) << 2 |  # Most significant bit for X
                (y >= center[1]) << 1 |  # Middle bit for Y
                (z >= center[2])         # Least significant bit for Z
            )
            
            # Add the vertex to the corresponding region
            regions[region_idx].append(vertex)
        
        # Convert each region list to a numpy array for easier manipulation
        regions = [np.array(region) for region in regions]
        
        return regions
            
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
        
        if self.uavs != {}:
            self.removeUAVs
            
        # Add UAVs to the scene
        for i in range(n):
            print(i)
            mesh = Mesh3D("resources/uav3.obj", color=COLORS[i])
            
            mesh = u.unit_sphere_normalization(mesh)
            rotation = get_rotation_matrix(np.pi, np.array([0, 1, 0]))
            mesh.vertices = np.dot(mesh.vertices, rotation)
            mesh, position = u.randomize_mesh_position(mesh)
            
            
            # label = Label3D(position, f"uav{i+1}", color=Color.BLACK)
            # self.addShape(label, f"label{i+1}")
            self.addShape(mesh, f"uav{i+1}")
            self.uavs[f"uav{i+1}"] = mesh
            self.movingUavs[f"uav{i+1}"] = mesh
            # self.labels[f"label{i+1}"] = label
    
    def createPlatform(self, n: int):
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
        
    def removeUAVs(self, uavs):
        '''remove all the UAVs from the scene
         
        Args: None
        Returns: None   
        '''
        # Clear the UAVs from the dictionary AND the scene
        for key in uavs.keys():
            self.removeShape(key)
        self.uavs = {}
        self.movingUavs = {}
        self.rotatedUavs = {}
        # for label in self.labels.keys():
        #     self.removeShape(label)
        for point in self.intersectingPoints.keys():
            self.removeShape(point)
        
        # self.labels = {}
        self.intersectingPoints = {}
            
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
      
    def create_all_AABBs(self, uavs): 
        '''add all the aabb to the scene
        args: None
        returns: None
        '''
        i = 0
                
        for mesh in uavs.values():
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
        
        
        corners = bottom_corners + top_corners
        center = min_point + (max_point - min_point) / 2
        return corners, center
        
        
if __name__ == "__main__":
    scene = UAV()
    scene.mainLoop()    