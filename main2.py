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
import json

WIDTH = 1200
HEIGHT = 1000

COLORS = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.ORANGE, Color.MAGENTA, Color.YELLOWGREEN, Color.CYAN, Color.BLACK, Color.WHITE, Color.DARKGREEN]

class UAV(Scene3D):
    def __init__(self):
        super().__init__(WIDTH, HEIGHT, "UAV", output=True, n_sliders=0)
        self.reset()
    
    def reset(self):        
        self.n = 3  # number of UAVs
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
        self.intersectingPoints = {}        #points where the uavs intersect
        self.movingUavs = {}                # UAVs that are moving
        if self.n<=5: self.createPlatform(self.n)
        else: self.createPlatform(5)
        self.fake_aabbs = {}
        self.chulls = {}
        self.interCuboids = {}
        self.velocities = {}
        self.last_time = time.time()
        self.dt = 0.1
        self.chullFromMesh = {}
        self.stat_times ={}
        self.projections = {}
        
    def on_key_press(self, symbol, n):
        if symbol == Key.U:                 #add uavs
            if self.uavs == {}:
                print("Adding UAVS...")
                if 'addUAVs' not in self.stat_times.keys(): self.stat_times['addUAVs'] = []
                time0 = time.time() 
                self.addUAVs(self.n)
                self.stat_times['addUAVs'].append(time.time() - time0)
            else:
                print("Removing UAVS...")
                if 'removeUAVs' not in self.stat_times.keys(): self.stat_times['removeUAVs'] = []
                time0 = time.time()
                self.removeUAVs(uavs=self.uavs)
                self.stat_times['removeUAVs'].append(time.time() - time0)
                self.remove_all_aabb()
                if self.aabb != {}:
                    print("Removing AABB...")
                    self.remove_all_aabb()
            
        if symbol == Key.P:                 #add planes
            if self.plane == {}:       
                print("Creating Plane...")
                if self.n<=5:
                    if 'createPlatform' not in self.stat_times.keys(): self.stat_times['createPlatform'] = []
                    time0 = time.time()
                    self.createPlatform(self.n)
                    self.stat_times['createPlatform'].append(time.time() - time0)
                else: self.createPlatform(5)
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
                        if 'createAllAABBs' not in self.stat_times.keys(): self.stat_times['createAllAABBs'] = []
                        time0 = time.time()
                        self.create_all_AABBs(self.uavs)
                        self.stat_times['createAllAABBs'].append(time.time() - time0)
                    elif self.rotatedUavs != {}:
                        if 'createAllAABBs' not in self.stat_times.keys(): self.stat_times['createAllAABBs'] = []
                        time0 = time.time()
                        self.create_all_AABBs(self.rotatedUavs)
                        self.stat_times['createAllAABBs'].append(time.time() - time0)
                    self.show_aabbs(self.aabb)
                else:
                    print("Removing AABB...")
                    self.remove_all_aabb()         
        
        if symbol == Key.K:                 #add kdop
            print("not yet done")
            
        if symbol == Key.C:                 #collision detection
            method = "accurate" # "aabb", "chull", "accurate"
            if self.uavs == {} and self.rotatedUavs == {}:
                print("Please add UAVs")
            elif self.uavs != {} and self.aabb == {}:
                self.create_all_AABBs(self.uavs)
                self.chull_from_mesh(self.uavs.items(), show = False)
                if f'basicCollisionDetection{method}' not in self.stat_times.keys(): self.stat_times[f'basicCollisionDetection{method}'] = []
                time0 = time.time()
                self.basicCollisionDetection(self.uavs, self.aabb, self.chullFromMesh, method)
                self.stat_times[f'basicCollisionDetection{method}'].append(time.time() - time0)
            elif self.rotatedUavs != {} and self.aabb == {}:
                self.create_all_AABBs(self.rotatedUavs)
                self.chull_from_mesh(self.uavs.items(), show = False)
                if f'basicCollisionDetection{method}' not in self.stat_times.keys(): self.stat_times[f'basicCollisionDetection{method}'] = []
                time0 = time.time()
                self.basicCollisionDetection(self.rotatedUavs, self.aabb, self.chullFromMesh,method)
                self.stat_times[f'basicCollisionDetection{method}'].append(time.time() - time0)
            elif self.uavs != {} and self.aabb != {}:
                self.chull_from_mesh(self.uavs.items(), show = False)
                if f'basicCollisionDetection{method}' not in self.stat_times.keys(): self.stat_times[f'basicCollisionDetection{method}'] = []
                time0 = time.time()
                self.basicCollisionDetection(self.uavs, self.aabb, self.chullFromMesh,method)
                self.stat_times[f'basicCollisionDetection{method}'].append(time.time() - time0)
            elif self.rotatedUavs != {} and self.aabb != {}:
                self.chull_from_mesh(self.uavs.items(), show = False)
                if f'basicCollisionDetection{method}' not in self.stat_times.keys(): self.stat_times[f'basicCollisionDetection{method}'] = []
                time0 = time.time()
                self.basicCollisionDetection(self.rotatedUavs, self.aabb, self.chullFromMesh,method)
                self.stat_times[f'basicCollisionDetection{method}'].append(time.time() - time0)
            else:
                print("Something went wrong")

        if symbol == Key.R:
            if self.uavs == {}:
                print("Adding Rotated UAVS...")
                self.addRotatedUavs(self.n)
            else:
                print("Removing Rotated UAVS...")
                self.removeUAVs(uavs=self.uavs)
                self.remove_all_aabb()
                if self.aabb != {}:
                    print("Removing AABB...")
                    self.remove_all_aabb()
        
        if symbol == Key.X:
            if self.chullFromMesh == {}:
                if self.rotatedUavs != {}:
                    if 'chullFromMesh' not in self.stat_times.keys(): self.stat_times['chullFromMesh'] = []
                    time0 = time.time()
                    self.chull_from_mesh(self.rotatedUavs.items(), show = True)
                    self.stat_times['chullFromMesh'].append(time.time() - time0)
                elif self.uavs != {}:
                    if 'chullFromMesh' not in self.stat_times.keys(): self.stat_times['chullFromMesh'] = []
                    time0 = time.time()
                    self.chull_from_mesh(self.uavs.items(), show = True)
                    self.stat_times['chullFromMesh'].append(time.time() - time0)
                else:
                    print("PLEASE ADD UAVS TO THE SCENE")
            else:
                for chull in self.chullFromMesh.keys():
                    self.removeShape('chull'+chull)
                    self.chullFromMesh = {}
        
        if symbol == Key.L:
            for uav in self.uavs.keys():
                self.moveUav(uav, np.array([0, 0.5, 0]))
        

        if symbol == Key.SPACE:
            self.paused = not self.paused
            print("Paused: ", self.paused) 
        
        if symbol ==Key.W:
            for uav_name, uav in self.uavs.items():
                self.get_projections_of_mesh(uav, uav_name)
            self.projectionCollisionDetection()
            
        if symbol == Key.Q:
            print("STATS: ", self.stat_times)
            formatted_stats = {key: values for key, values in self.stat_times.items()}
            # Writing the formatted dictionary to a JSON file
            with open('stats.json', 'w') as json_file:
                json.dump(self.stat_times, json_file, indent=4)
                
    def on_idle(self):
        if time.time() - self.last_time< self.dt:
            return
        if not self.paused:
            if self.uavs != {}:
                if self.movingUavs != {}:
                    if self.aabb == {}:
                        try:
                            print("Creating AABBs for collision detection")
                            self.create_all_AABBs(self.uavs)
                        except Exception as e:
                            print("Problem with creating the AABBs. ", e)
                        
                    # if 'checkMovingCollision' not in self.stat_times.keys(): self.stat_times['checkMovingCollision'] = []
                    # time0 = time.time()
                    # self.checkMovingCollision()
                    # self.stat_times['checkMovingCollision'].append(time.time() - time0)
                    
                    # if 'avoidCollision' not in self.stat_times.keys(): self.stat_times['avoidCollision'] = []
                    # time0 = time.time()
                    # self.avoidCollision()
                    # self.stat_times['avoidCollision'].append(time.time() - time0)
                    
                    if 'launchUavs' not in self.stat_times.keys(): self.stat_times['launchUavs'] = []
                    time0 = time.time()
                    self.launchUavs()
                    self.stat_times['launchUavs'].append(time.time() - time0)
                
                    
                else: 
                    print("All UAVs have stopped moving")
                    self.paused = True
                    
            else:
                print("No UAVs in the scene, Press U to add UAVs")

                    
    def projectionCollisionDetection(self):
        for uav_name1, uav1 in self.uavs.items():
            for uav_name2, uav2 in self.uavs.items():
                if uav_name1 < uav_name2:

                    xy1 = self.projections[f'xy_{uav_name1}']
                    xy2 = self.projections[f'xy_{uav_name2}']
                    xz1 = self.projections[f'xz_{uav_name1}']
                    xz2 = self.projections[f'xz_{uav_name2}']
                    yz1 = self.projections[f'yz_{uav_name1}']
                    yz2 = self.projections[f'yz_{uav_name2}']
                    if self.mesh_intersection(xy1, xy2)[0]:
                        print("Collision detected in xy projection for ", uav_name1, " and ", uav_name2)
                        if self.mesh_intersection(xz1, xz2)[0]:
                            print("Collision detected in xy and xz projection for ", uav_name1, " and ", uav_name2)
                            if self.mesh_intersection(yz1, yz2)[0]:
                                print("Collision detected in xy, xz and yz projection for ", uav_name1, " and ", uav_name2)
                    else: 
                        print("No collision detected in xy projection for ", uav_name1, " and ", uav_name2)
        print("Projections checked")
                        
                                
                    
    def get_projections_of_mesh(self, mesh, mesh_name):
        vertices = np.array(mesh.vertices)
        vertices1 = vertices.copy()
        vertices2 = vertices.copy()
        vertices3 = vertices.copy()
        
        vertices1[:, 2] = 0
        vertices2[:, 1] = 0
        vertices3[:, 0] = 0
        xy_mesh = Mesh3D()
        xy_mesh.vertices = vertices1
        xy_mesh.triangles = mesh.triangles
        yz_mesh = Mesh3D()
        yz_mesh.vertices = vertices3
        yz_mesh.triangles = mesh.triangles
        xz_mesh = Mesh3D()
        xz_mesh.vertices = vertices2
        xz_mesh.triangles = mesh.triangles
        
        color_rgba = COLORS[int(mesh_name[-1])-1]
        xy_mesh.color = color_rgba
        xz_mesh.color = color_rgba
        yz_mesh.color = color_rgba
        self.addShape(xy_mesh, f'xy_mesh{mesh_name}') 
        self.addShape(xz_mesh, f'xz_mesh{mesh_name}') 
        self.addShape(yz_mesh, f'yz_mesh{mesh_name}') 

        self.projections[f'xy_{mesh_name}'] = xy_mesh
        self.projections[f'yz_{mesh_name}'] = yz_mesh
        self.projections[f'xz_{mesh_name}'] = xz_mesh
        
        return xy_mesh, xz_mesh, yz_mesh

        
        
    def chull_from_mesh(self, uavdict, show = False):
        for uavname, uav in uavdict:
            openMesh = u.mesh_to_o3d(uav)
            convex_hull, _ = openMesh.compute_convex_hull()
            chull = u.o3d_to_mesh(convex_hull)
            
            self.chullFromMesh[uavname] = chull
            
            if show:
                self.addShape(chull, 'chull'+uavname)
            
    def directed_movement(self, vector, uav_name):
        random_float = random.uniform(0.1, 0.4)
        velocity = random_float*vector
        self.moveUav(uav_name, velocity)
        
    def coinFlip(self):
        if random.random()>0.99: return True 
        return False
    
    def launchUavs(self):

        if len(self.uavs) != (self.n)**2:
            if self.coinFlip: 
                    self.spawnNewDrone()
                    print("New Drone Spawned")

            # for j, (uav_name, uav) in enumerate(self.uavs.items()):
        self.avoidCollision(False)
                # self.landUav(uav, uav_name, list(self.plane.values())[j])
        
    def landUav(self, uav, uav_name, cuboid):
        vector = self.getLandingVector(uav, cuboid)
        norm_vector = np.linalg.norm(vector)
        if norm_vector>0.1:
            self.directed_movement(vector, uav_name)
        
    def getLandingVector(self, uav, cuboid):
        cuboid_center = self.getAllCuboidCorners(cuboid)[1]
        uav_center = np.mean(uav.vertices,axis=0)
        cuboid_landing_point = cuboid_center + np.array([0, 0.25, 0])
        uav_landing_point = uav_center -  np.array([0, 0.2, 0])
        
        return cuboid_landing_point-uav_landing_point     
                     
    def spawnNewDrone(self):
        i = len(self.uavs)
        mesh = Mesh3D(f"resources/uav{i%3+1}.obj", color=COLORS[i%len(COLORS)])
        mesh = u.unit_sphere_normalization(mesh)
        rotation = get_rotation_matrix(np.pi, np.array([0, 1, 0]))
        mesh.vertices = np.dot(mesh.vertices, rotation)
        mesh, position = u.randomize_mesh_position(mesh)
        
        self.addShape(mesh, f"uav{i+1}")
        self.uavs[f"uav{i+1}"] = mesh
        self.movingUavs[f"uav{i+1}"] = mesh
             
    def get_directed_velocities(self):
        my_dict = {}
        for i, (uav_name, uav) in enumerate(self.uavs.items()):
            vector = self.getLandingVector(uav, list(self.plane.values())[i])
            norm_vector = np.linalg.norm(vector)
            random_float = random.uniform(0.1, 0.4)
            velocity = random_float*(vector/norm_vector)
            if norm_vector <= 0.5:
                velocity = np.array([0, 0, 0])    
            my_dict[uav_name] = velocity
        return my_dict
        
    def avoidCollision(self, flag = True):
        if flag:
            velocities2 = self.get_velocities(self.uavs.keys())
        else: velocities2 = self.get_directed_velocities()
        if all(np.array_equal(velocities2[uav], np.array([0, 0, 0])) for uav in velocities2.keys()):
            self.paused = True
        self.velocities = velocities2
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
                self.fake_aabbs[aabb_name] = Cuboid3D(min_value, max_value)

        
        for aabb1_name, aabb1 in self.aabb.items():
            i = aabb1_name[-1]
            # if 'chull'+i not in self.chulls:
            #     self.chulls['chull'+i] = []
            if aabb1_name in self.fake_aabbs.keys():
                self.chulls['chull'+i] = self.aabbs_to_chull(self.aabb['aabbCuboid'+i], aabb1)
            else:
                self.chulls['chull'+i] = u.o3d_to_mesh(self.aabbs_to_chull(aabb1, aabb1))

            
        # check if the convex hulls are colliding
        for i, chullvalue1 in enumerate(self.chulls.values()):
            for j, chullvalue2 in enumerate(self.chulls.values()):
                if i<=j: 
                    continue
                elif i>j:
                    u_var = self.chull_collision(chullvalue1, chullvalue2)
                    if u_var[0]:
                        print("Future Collision detected")
                        current_velocity1 = self.velocities[f'uav'+str(i+1)]
                        current_velocity2 = self.velocities[f'uav'+str(j+1)]
                        deflected_velocity1 = self.normal_to_deflected_velocity(u_var[2][0], current_velocity1)
                        deflected_velocity2 = self.normal_to_deflected_velocity(u_var[2][0], current_velocity2)
                        self.velocities['uav'+str(i)] = deflected_velocity1*3
                        self.velocities['uav'+str(j)] = deflected_velocity2*3

                    
        for i, (uav, uav_mesh) in enumerate(self.movingUavs.items()):
            if flag:
                self.moveUav(uav, self.velocities[uav])
            else: 
                if self.velocities[uav][0] == 0 and self.velocities[uav][1] == 0 and self.velocities[uav][2] == 0:
                    uav_center = np.mean(uav_mesh.vertices,axis=0)
                    cuboid = list(self.plane.values())[int(uav[-1])-1]
                    cuboid_center = self.getAllCuboidCorners(cuboid)[1] + np.array([0, 0.5, 0])
                    vector = cuboid_center - uav_center
                    uav_mesh.vertices += vector
                    self.updateShape(uav, True)
                else:
                    self.moveUav(uav, self.velocities[uav])
        return None
    
    def normal_to_deflected_velocity(self, normal: np.ndarray, current_velocity):
        try: collision_norm = normal #/np.linalg.norm(normal)
        except Exception as e: print("Error e: ", e)
        deflected_velocity =  np.array(current_velocity - 2*np.dot(current_velocity, collision_norm) * collision_norm)
        return deflected_velocity
    
    def checkMovingCollision(self):
        '''
            when uavs are moving, check if they collide with each other using the convex hull of the aabb in time t+dt and time t
            input: uavs: dict: dictionary containing all the uavs, aabbs: dict: dictionary containing all the aabbs
            return None
        '''

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
                self.fake_aabbs[aabb_name] = Cuboid3D(min_value, max_value)
        
        
        for aabb1_name, aabb1 in self.aabb.items():
            i = aabb1_name[-1]
            # if 'chull'+i not in self.chulls:
            #     self.chulls['chull'+i] = []
            if aabb1_name in self.fake_aabbs.keys():
                self.chulls['chull'+i] = self.aabbs_to_chull(self.aabb['aabbCuboid'+i], aabb1)
            else:
                self.chulls['chull'+i] = u.o3d_to_mesh(self.aabbs_to_chull(aabb1, aabb1))

            
        # check if the convex hulls are colliding
        for i, chullvalue1 in enumerate(self.chulls.values()):
            for j, chullvalue2 in enumerate(self.chulls.values()):
                
                if i<=j: 
                    continue
                elif i>j:
                    u_var = self.chull_collision(chullvalue1, chullvalue2)
                    if u_var[0]:
                        print("Collision detected")
                        self.aabbCollisionDetection(self.fake_aabbs, show_intersecting_cuboid=True)
                        for k in range(0, len(u_var[1])):
                        
                            self.addShape(Point3D(u_var[1][k], size = 2, color=Color.BLACK), f"point{random.randint(0, 10000)}")
                            
                        if 'uav'+str(i+1) in self.movingUavs:
                            self.movingUavs.pop('uav'+str(i+1))
                        if 'uav'+str(j+1) in self.movingUavs:
                            self.movingUavs.pop('uav'+str(j+1))
                        # break
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
        self.last_time=time.time()
        uav = self.uavs[uav_name]
        # x, y, z = self.get_velocities(self.uavs.keys())[uav_name]
        x, y, z = velocity
        uav.vertices += np.array([x, y, z])
        i = int(uav_name.split("uav")[1])
        if "aabbCuboid" + str(i) in self.aabb:
            aabb = self.aabb[f"aabbCuboid{i}"]
            aabb.translate([x, y, z])
            self.updateShape(f"aabbCuboid{i}", True)
        if uav_name in self.chullFromMesh:
            chull = self.chullFromMesh[uav_name]
            chull.vertices += np.array([x, y, z])
            self.updateShape('chull'+uav_name, True)
        self.updateShape(uav_name, True) 
        for key in self.projections.keys():
            if key[3:] == uav_name:
                print("Updating projections")
                if key[:3] == "xy_":
                    xy = self.projections["xy_"+uav_name]
                    xy.vertices += np.array([x, y, 0])
                    self.updateShape('xy_mesh'+uav_name, True)
                if key[:3] == "xz_":
                    xz = self.projections["xz_"+uav_name]
                    xz.vertices += np.array([x, 0, z])
                    self.updateShape('yz_mesh'+uav_name, True)
                if key[:3] == "yz_":
                    yz = self.projections["yz_"+uav_name]
                    yz.vertices += np.array([0, y, z])
                    self.updateShape('xz_mesh'+uav_name, True)
                
                           
    
    # move only the aabb  
    def moveAabb(self, velocities, aabb_name):
        '''move the aabb in the scene'''
        aabb = self.aabb[aabb_name]
        aabb_generalized = Cuboid3DGeneralized(aabb)
        x, y, z = velocities['uav'+aabb_name[-1]]
        aabb_generalized.translate([x, y, z])
        self.updateShape(aabb_name, True)
        self.fake_aabbs[aabb_name] = aabb_generalized.cuboid

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
            num = 2 if i % 2 == 0 else 1 if i % 3 == 0 else 3
            mesh = Mesh3D(f"resources/uav{num}.obj", color=COLORS[i])
            
            mesh = u.unit_sphere_normalization(mesh)
            mesh = u.randomize_mesh_rotation(mesh)
            mesh, position = u.randomize_mesh_position(mesh)
            
            # label = Label3D(position, f"uav{i+1}", color=Color.BLACK)
            # self.addShape(label, f"label{i+1}")
            self.addShape(mesh, f"uav{i+1}")
            self.uavs[f"uav{i+1}"] = mesh
            self.movingUavs[f"uav{i+1}"] = mesh
            # self.labels[f"label{i+1}"] = label           
                           
    def basicCollisionDetection(self, uavs, aabbs, chulls, method):
        if method == "aabb":
            self.aabbCollisionDetection(aabbs, show_intersecting_cuboid=True)
        elif method == "chull":
            self.chullCollisionDetection(chulls, True)
        elif method == "accurate":
            self.accurateCollisionDetection(uavs, aabbs, chulls)
        else:
            print("Invalid method")

    def accurateCollisionDetection(self, uavs, aabbs, chulls):
        '''detect collision using accurate method
        args: uavs: dict: dictionary containing all the uavs
        returns: None
        '''
        
        for i, (key, value) in enumerate(list(uavs.items())):
            for j, (key2, value2) in enumerate(list(uavs.items())):
                if j>i:
                    self.accurateCollision(value, value2, aabbs[f"aabbCuboid{i+1}"], aabbs[f"aabbCuboid{j+1}"], chulls[key], chulls[key2], key, key2)        
    
    def accurateCollision(self, uav1, uav2, aabb1, aabb2, chull1, chull2, i, j):
        '''detect collision between two uavs
        args: uav1: Mesh3D object: first uav
              uav2: Mesh3D object: second uav
        returns: None
        '''
        if self.aabbCollision(aabb1, aabb2, i, j, show_intersecting_cuboid=False):
            if self.chull_collision(chull1, chull2):
                collision, points, _ = self.mesh_intersection(uav1, uav2)
                # Get the vertices of the two UAVs
                if collision:
                    print(f"ACCURATE Collision detected between {i} and {j}")
                    if f"uav{i}" in self.movingUavs:
                        self.movingUavs.pop(f"uav{i}")
                    if f"uav{j}" in self.movingUavs:
                        self.movingUavs.pop(f"uav{j}")
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
        
        collision, points = collision_manager.in_collision_internal(return_data=True)
        pointlist = []
        norms = []
        for point in points:
            pointlist.append(point.point)
            norms.append(point.normal)
            
        return collision, pointlist, norms
                                     
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
            if show_intersecting_cuboid: self.intersectingCuboid(aabb1, aabb2)
            return True
        return False
 
    def chullCollisionDetection(self, chulls, show_intersecting_mesh):
        '''detect collision using convex hull
        args: chulls: dict: dictionary containing all the chulls
        returns: None
        '''
        for i in chulls:
            for j in chulls:
                if i<j:
                    coll = self.chull_collision(chulls[i], chulls[j])
                    if coll[0]:
                        print(f"Chull collision detected between {i} and {j}")
                    
                        if show_intersecting_mesh:
                            for point in coll[1]:
                                
                                index = random.uniform(10, 10000)
                                self.addShape(Point3D(point, size=10, color=Color.RED), f"point_{index}")
                                self.intersectingPoints[f"point_{index}"] = Point3D(point, size=2, color=Color.BLACK)
                            
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
        self.interCuboids[f"intersecting_cuboid_{aabb1}_{aabb2}"] = intersecting_cuboid

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
            num = 2 if i % 2 == 0 else 1 if i % 3 == 0 else 3
            mesh = Mesh3D(f"resources/uav{num}.obj", color=COLORS[i])
            
            mesh = u.unit_sphere_normalization(mesh)
            rotation = get_rotation_matrix(np.pi, np.array([0, 1, 0]))
            mesh.vertices = np.dot(mesh.vertices, rotation)
            mesh, position = u.randomize_mesh_position(mesh)
            
            self.addShape(mesh, f"uav{i+1}")
            self.uavs[f"uav{i+1}"] = mesh
            self.movingUavs[f"uav{i+1}"] = mesh
    
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
                cuboid = Cuboid3D(
                        start_point, 
                        end_point, 
                        width=1, 
                        color=color, 
                        filled=True
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
        
        for cuboid in self.interCuboids.keys():
            self.removeShape(cuboid)
        
        for chull in self.chullFromMesh.keys():
            self.removeShape('chull'+chull)
            
        
        # self.labels = {}
        self.intersectingPoints = {}
        self.chulls = {}
        self.interCuboids = {}
        self.velocities = {}
        self.chullFromMesh = {}
           
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