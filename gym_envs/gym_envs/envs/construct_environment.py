'''
Functionality to enable the loading of the environment including:
- Plane (to form ground)
- Left Track
- Right Track
'''

import pybullet as p
import pybullet_data
import os
import numpy as np
# for plotting graphcs
import matplotlib.pyplot as plt
# packages for generating hulls
import alphashape
from scipy.spatial import KDTree
from scipy.interpolate import interp1d
from descartes import PolygonPatch
import time

def compute_arc_length(points):
        '''
        compute the cumulative arc length along a polygonal path.
        '''

        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)) 
        # diff computes the differences between consecutive points along the track, this results in a vector of differences in both x and y directions. 
        # sqrt of the sum computes the euclidean distance between each consecutive pair of points. straight line distance between consecutive points.
        return np.insert(np.cumsum(distances), 0, 0)  # Insert 0 at the start
        # np.cumsum computes the cumulative sum of the distances between consecutive points, which gives the arc length at each point
        # np.insert() adds a 0 at the beginning of the cumulative sum, so that the arc length at the first point is 0. 

def interpolate_track(points, num_samples = 100):
    '''
    interpolates the track to generate a specified number of evenly spaced points.
    '''

    arc_length = compute_arc_length(points)
    # print("arc_length: ", arc_length)
    interp_x = interp1d(arc_length, points[:,0], kind='linear')
    interp_y = interp1d(arc_length, points[:,1], kind='linear')

    new_arc_lengths = np.linspace(0, arc_length[-1], num_samples)
    return np.column_stack((interp_x(new_arc_lengths), interp_y(new_arc_lengths)))

def find_centerline(outer_track_id, inner_track_id):
    '''
    This takes the outer and inner track objects, and produces the centerline
    This is not the optimal racing centerline, simply the midpoints
    It can be used for generating a reward signal for the reinforcement learning system.
    '''

    # get the vertices of the meshes. 
    # num_visual_shapes = p.getVisualShapeData(outer_track_id)
    # mesh_vertices = []

    outer_points = []
    inner_points = []

    # compute points
    mesh_data = p.getMeshData(outer_track_id)
    outer_points = np.array(mesh_data[1])  # Extract vertex positions
    # print("original outer points: ", outer_points)

    # range_x = max(outer_points[:,0]) - min(outer_points[:,0])
    # range_y = max(outer_points[:,1]) - min(outer_points[:,1])
    # range_z = max(outer_points[:,2]) - min(outer_points[:,2])
    # print("ranges: ", range_x, ", ", range_y , ", ", range_z)
    outer_points = outer_points[:, [0,2]]
    # so the important information is in the first and last items. I guess y is the up direction.

    mesh_data = p.getMeshData(inner_track_id)
    inner_points = np.array(mesh_data[1])  # Extract vertex positions
    inner_points = inner_points[:, [0,2]]

    # print("outer_points: ", outer_points)
    # print("inner_points: ", inner_points)

    # x1, y1 = outer_points[:,0], outer_points[:, 1]
    # x2, y2 = inner_points[:,0], inner_points[:, 1]

    # fig, ax = plt.subplots()

    # ax.scatter(x1, y1, color='blue', label='Points 1')
    # ax.scatter(x2, y2, color='red', label='Points 2')

    # Labels and legend
    # plt.xlabel('X-axis')
    # plt.ylabel('Y-axis')
    # plt.title('Scatter Plot of Two 2D Arrays')
    # plt.axis('equal')
    # plt.legend()
    # plt.grid(True)

    

    # use alphashape to generate the hulls.

    alpha = 0.00 # tunable parameter

    # compute for outer track: 
    alpha_shape = alphashape.alphashape(outer_points, alpha)

    # print("alpha_shape: ", alpha_shape)

     # Handle MultiPolygon cases
    if alpha_shape.geom_type == "MultiPolygon":
        print("multipolygon")
        # Select the LARGEST polygon by area
        alpha_shape = max(alpha_shape.geoms, key=lambda p: p.area)

    outer_points = alpha_shape.exterior.xy
    outer_points = list(zip(outer_points[0], outer_points[1]))
    outer_points = np.array(outer_points)
    # x, y = alpha_shape.exterior.xy
    # TODO determine how to compute the interior instead of exterior for the outer track.
        # outer_points = alpha_shape.interiors

        # # Plot the **interior**
        # for interior in alpha_shape.interiors:
        #     print("interior")
        #     x, y = interior.xy
        #     ax.plot(x, y, 'r-', linewidth=2, label="Interior")

    # print(outer_points)
    # ✅ Use plt.plot() to visualize the boundary

    # compute for the inner track
    alpha_shape = alphashape.alphashape(inner_points, alpha)
     # Handle MultiPolygon cases
    if alpha_shape.geom_type == "MultiPolygon":
        print("multipolygon")
        # Select the LARGEST polygon by area
        alpha_shape = max(alpha_shape.geoms, key=lambda p: p.area)

    inner_points = alpha_shape.exterior.xy
    inner_points = list(zip(inner_points[0], inner_points[1]))
    inner_points = np.array(inner_points)

    # print("inner_points: ", inner_points)
    # print("np inner points: ", np.array(inner_points))
    
    # x,y = outer_points
    x = outer_points[:,0]
    y = outer_points[:,1]

    
    # ax.plot(x, y, 'b-', linewidth=2)  # Outline of the polygon
    # ax.fill(x, y, color='lightblue', alpha=0.5)  # Fill with transparency

    # x,y = inner_points
    x = inner_points[:,0]
    y = inner_points[:,1]

    # ax.plot(x, y, 'g-', linewidth=2)  # Outline of the polygon
    # ax.fill(x, y, color='lightblue', alpha=0.5)  # Fill with transparency

    # if alpha_shape.geom_type == "Polygon":
    #     boundary_points = np.array(alpha_shape.exterior.coords)
    #     # x3, y3 = boundary_points[:,0], boundary_points[:,1]
    #     # plt.scatter(x3, y3, color="yellow", label="boundary")
    #     fig, ax = plt.subplots()
    #     ax.add_patch(PolygonPatch(alpha_shape, alpha=0.1))
    # else:
    #     raise ValueError("Alpha shape did not produce a valid polygon.")

    # now that we have the inner and outer track polygons, compute the midpoints
    
    # print("inner_points: ", inner_points)
    # print("np(inner_points): ", np.array(inner_points))

    inner_interp = interpolate_track(np.array(inner_points))
    outer_interp = interpolate_track(np.array(outer_points))

    # plot the interpolated paths: 

    centerline = (inner_interp + outer_interp) / 2

    # ax.plot(centerline[:, 0], centerline[:, 1], 'g-', label="Centerline (Computed)")

    # ax.plot(inner_interp[:,0], inner_interp[:,1], 'r-', linewidth=2)
    # ax.plot(outer_interp[:,0], outer_interp[:,1], 'b-', linewidth=2)

    # plt.ion()
    # plt.show()
    # input("press enter to exit...")

    return centerline

def construct_environment(outer_track_path, inner_track_path):
    '''
    both paths should point to separate obj files that represent the inner and outer tracks. 
    '''

    print("current pwd: ", os.getcwd())

    # input error checking
    if not os.path.exists(outer_track_path):
        raise FileNotFoundError(f"Outer track file not found: {outer_track_path}")
    if not os.path.exists(inner_track_path):
        raise FileNotFoundError(f"Inner track file not found: {inner_track_path}")


    # set gravity
    p.setGravity(0,0,-10)

    # build the floor plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # build track
    # outer track
    outer_track_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=outer_track_path)
    outer_track_collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=outer_track_path)
    outer_track_id = p.createMultiBody(baseMass=0,  # Static object
                             baseCollisionShapeIndex=outer_track_collision_shape,
                             baseVisualShapeIndex=outer_track_visual_shape,
                             basePosition=[0, 0, 0],
                             baseOrientation=p.getQuaternionFromEuler([ 1.5708, 0, 0])
                             )

    # inner track
    inner_track_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=inner_track_path)
    inner_track_collision_shape = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=inner_track_path)
    inner_track_id = p.createMultiBody(baseMass=0,  # Static object
                             baseCollisionShapeIndex=inner_track_collision_shape,
                             baseVisualShapeIndex=inner_track_visual_shape,
                             basePosition=[0, 0, 0],
                             baseOrientation=p.getQuaternionFromEuler([ 1.5708, 0, 0])
                             )
    
    return outer_track_id, inner_track_id