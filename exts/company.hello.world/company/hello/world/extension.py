from functools import partial
import random as r
import numpy as np
from numpy.random import choice

from pxr import Gf, UsdGeom
from pxr.Usd import Stage

import omni.ext
import omni.kit.commands
import omni.ui as ui
import omni.usd
import omni.replicator.core as rep
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import omni.isaac.core.utils.prims as prim_utils
import omni.isaac.ui.ui_utils as ui_utils
from omni.isaac.ui.ui_utils import dropdown_builder, float_builder, str_builder
from omni.isaac.core.prims import XFormPrim
from pxr import UsdPhysics, Sdf, PhysxSchema, Usd
from omni.physx import get_physx_scene_query_interface
import carb
from scipy import interpolate
import noise #this needs to be added to the isaac sim python installation


trees = []
saved_lights = []
rocks = []
vegitation = []
object_to_spawn_list = []
selectable_models = 0

#change these
Rock_usd_path = "D:/temp_downloads/Rock_obj/Rock.usd"
Blueberry_usd_path = "D:/temp_downloads/Blueberry_obj/Blueberry.usd"
Bush_usd_path = "D:/temp_downloads/Bush_obj/Bush.usd"
Birch_usd_path = "D:/temp_downloads/Birch_obj/Birch.usd"
Spruce_usd_path = "D:/temp_downloads/Spruce_obj/Spruce.usd"
Pine_usd_path = "D:/temp_downloads/Pine_obj/Pine.usd"
#https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/
def convert_rotation(roll, pitch, yaw):
    #converts degrees to radians first
    roll = roll * (np.pi/180)
    pitch = pitch * (np.pi/180)
    yaw = yaw * (np.pi/180)
    #radians to quarternions
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qw, qx, qy, qz

def pick_tree(density, birch_p, spruce_p, pine_p, area_x, area_y, v_age_min, v_age_max):
    total_trees = density * (area_x * area_y) / 100.0

    y = int(total_trees * birch_p/100)
    z = int(total_trees * spruce_p/100)
    c = int(total_trees * pine_p/100)

    for _ in range(y):
        growth_multiplier = calculate_growth_multiplier(v_age_min, v_age_max)
        create_tree("Birch", area_x, area_y, growth_multiplier)
    for _ in range(z):
        growth_multiplier = calculate_growth_multiplier(v_age_min, v_age_max)
        create_tree("Spruce", area_x, area_y, growth_multiplier)
    for _ in range(c):
        growth_multiplier = calculate_growth_multiplier(v_age_min, v_age_max)
        create_tree("Pine", area_x, area_y, growth_multiplier)


def create_tree(tree_type, area_x, area_y, tree_age):
    stage = get_current_stage()
    area_x = (area_x/2) - 0.4 #a small no spawn zone around the edge (doesnt affect the terrain only the trees)
    area_y = (area_y/2) - 0.4 #a small no spawn zone around the edge (doesnt affect the terrain only the trees)
    over_all_scale = 1
    hight_multiplier = 1.35
    #todo change the way trees are made, atm it makes a whole new copy with new materials etc for each tree, dont know if this is actually possible
    #todo looks like cols dont get regenerated? They are only calculated when sim is not running. cant do anything about it
    #todo automatically load the ground texture
    
    if tree_type == "Birch":
        hight_multiplier = 1.35
        #scale stuff
        tree_scale = r.uniform(0.9, 1.1)*tree_age*over_all_scale
        tree_hight = r.uniform(0.9, 1.1)*hight_multiplier*tree_age*over_all_scale

        #path to the tree
        tree_path = f'/World/Tree_parent/Tree_{str(len(trees)).rjust(4,"0")}'
        xform = prim_utils.create_prim(tree_path, "Xform")

        #load the model and place it in the world
        add_reference_to_stage(usd_path=Birch_usd_path, prim_path=tree_path)

        #randomize roatation
        qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

        #set rot, scale and pos
        prim_utils.set_prim_property(tree_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(r.uniform(-area_x, area_x),
                                    r.uniform(-area_y, area_y), 100))
        prim_utils.set_prim_property(tree_path, "xformOp:scale", Gf.Vec3d(tree_scale, tree_scale, tree_hight))

        #find where ground is and move the tree there
        x, y, z = str(prim_utils.get_prim_property(tree_path, "xformOp:translate")).strip('()').split(",")
        distance_to_ground = 100-check_raycast(stage, tree_path)
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(float(x), float(y), distance_to_ground))

        trees.append(tree_path)
    elif tree_type == "Spruce":
        hight_multiplier = 1.25
        #scale stuff
        tree_scale = r.uniform(0.9, 1.1)*tree_age*over_all_scale
        tree_hight = r.uniform(0.9, 1.1)*hight_multiplier*tree_age*over_all_scale

        #path to the tree
        tree_path = f'/World/Tree_parent/Tree_{str(len(trees)).rjust(4,"0")}'
        xform = prim_utils.create_prim(tree_path, "Xform")

        #load the model and place it in the world
        add_reference_to_stage(usd_path=Spruce_usd_path, prim_path=tree_path)

        #randomize roatation
        qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

        #set rot, scale and pos
        prim_utils.set_prim_property(tree_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(r.uniform(-area_x, area_x),
                                    r.uniform(-area_y, area_y), 100))
        prim_utils.set_prim_property(tree_path, "xformOp:scale", Gf.Vec3d(tree_scale, tree_scale, tree_hight))

        #find where ground is and move the tree there
        x, y, z = str(prim_utils.get_prim_property(tree_path, "xformOp:translate")).strip('()').split(",")
        distance_to_ground = 100-check_raycast(stage, tree_path)
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(float(x), float(y), distance_to_ground))

        trees.append(tree_path)
    else:
        hight_multiplier = 1.35
        #scale stuff
        tree_scale = r.uniform(0.9, 1.1)*tree_age*over_all_scale
        tree_hight = r.uniform(0.9, 1.1)*hight_multiplier*tree_age*over_all_scale
        
        #path to the tree
        tree_path = f'/World/Tree_parent/Tree_{str(len(trees)).rjust(4,"0")}'
        xform = prim_utils.create_prim(tree_path, "Xform")

        #load the model and place it in the world
        add_reference_to_stage(usd_path=Pine_usd_path, prim_path=tree_path)

        #randomize roatation
        qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

        #set rot, scale and pos
        prim_utils.set_prim_property(tree_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(r.uniform(-area_x, area_x),
                                    r.uniform(-area_y, area_y), 100))
        prim_utils.set_prim_property(tree_path, "xformOp:scale", Gf.Vec3d(tree_scale, tree_scale, tree_hight))

        #find where ground is and move the tree there
        x, y, z = str(prim_utils.get_prim_property(tree_path, "xformOp:translate")).strip('()').split(",")
        distance_to_ground = 100-check_raycast(stage, tree_path)
        prim_utils.set_prim_property(tree_path, "xformOp:translate", Gf.Vec3d(float(x), float(y), distance_to_ground))

        trees.append(tree_path)

def create_rocks(stage, rockiness, area_x, area_y):
    total_rocks = int(rockiness * (area_x * area_y) / 100.0)
    area_x = (area_x/2) - 0.3
    area_y = (area_y/2) - 0.3
    for _ in range(total_rocks):
        #path to the rocks
        rock_path = f'/World/Rock_parent/Rock_{str(len(rocks)).rjust(4,"0")}'
        xform = prim_utils.create_prim(rock_path, "Xform")

        #load the model and place it in the world
        add_reference_to_stage(usd_path=Rock_usd_path, prim_path=rock_path)

        #randomize roatation
        qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

        #set rot, scale and pos
        prim_utils.set_prim_property(rock_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
        prim_utils.set_prim_property(rock_path, "xformOp:translate", Gf.Vec3d(r.uniform(-area_x, area_x),
                                    r.uniform(-area_y, area_y), 100))
        prim_utils.set_prim_property(rock_path, "xformOp:scale", Gf.Vec3d(r.uniform(0.2, 0.3), r.uniform(0.2, 0.3),
                                    r.uniform(0.2, 0.3)))

        #find where ground is and move the rock there
        x, y, z = str(prim_utils.get_prim_property(rock_path, "xformOp:translate")).strip('()').split(",")
        distance_to_ground = 100-check_raycast(stage, rock_path)
        prim_utils.set_prim_property(rock_path, "xformOp:translate", Gf.Vec3d(float(x), float(y), distance_to_ground))

        rocks.append(rock_path)

def generate_vegitation(stage, density, area_x, area_y):
    total_bushes = int(density * (area_x * area_y) / 100.0)
    area_x = (area_x/2) - 0.5
    area_y = (area_y/2) - 0.5

    for _ in range(total_bushes):
        random_number = r.randrange(0, 2)
        if random_number == 1:
            center_x = r.uniform(-area_x, area_x)
            center_y = r.uniform(-area_y, area_y)
            for _ in range(10):
                ranges_x = [(-0.4, -0.2), (0.2, 0.4)]
                selected_range_x = r.choice(ranges_x)
                random_float_x = r.uniform(selected_range_x[0], selected_range_x[1])
                ranges_y = [(-0.4, -0.2), (0.2, 0.4)]
                selected_range_y = r.choice(ranges_y)
                random_float_y = r.uniform(selected_range_y[0], selected_range_y[1])

                # Use a Gaussian distribution to bias the positions towards the center
                sigma = 0.09  # Adjust the sigma value to control the clustering
                temp_x = center_x + r.gauss(0, sigma) + random_float_x
                temp_y = center_y + r.gauss(0, sigma) + random_float_y

                #path to the other plants
                plant_path = f'/World/Bush_parent/Plant_{str(len(vegitation)).rjust(4,"0")}'
                xform = prim_utils.create_prim(plant_path, "Xform")

                #load the model and place it in the world
                add_reference_to_stage(usd_path=Blueberry_usd_path, prim_path=plant_path)

                #randomize rotation
                qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

                #set rot, scale and pos
                prim_utils.set_prim_property(plant_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
                prim_utils.set_prim_property(plant_path, "xformOp:translate", Gf.Vec3d(temp_x, temp_y, 100))
                prim_utils.set_prim_property(plant_path, "xformOp:scale", Gf.Vec3d(r.uniform(0.8, 1.2),
                                            r.uniform(0.8, 1.2), r.uniform(0.8, 1)))

                #find where ground is and move the bush there
                x, y, z = str(prim_utils.get_prim_property(plant_path, "xformOp:translate")).strip('()').split(',')
                distance_to_ground = 100-check_raycast(stage, plant_path)
                prim_utils.set_prim_property(plant_path, "xformOp:translate", Gf.Vec3d(float(x), float(y),
                                            distance_to_ground))

                vegitation.append(plant_path)
        else:
            #path to the other plants
            plant_path = f'/World/Bush_parent/Plant_{str(len(vegitation)).rjust(4,"0")}'
            xform = prim_utils.create_prim(plant_path, "Xform")

            #load the model and place it in the world
            add_reference_to_stage(usd_path=Bush_usd_path, prim_path=plant_path)

            #randomize rotation
            qw, qx, qy, qz = convert_rotation(0, 0, r.randrange(-180, 180))

            #set rot, scale and pos
            prim_utils.set_prim_property(plant_path, "xformOp:orient", Gf.Quatd(qw, qx, qy, qz))
            prim_utils.set_prim_property(plant_path, "xformOp:translate", Gf.Vec3d(r.uniform(-area_x, area_x),
                                        r.uniform(-area_y, area_y), 100))
            prim_utils.set_prim_property(plant_path, "xformOp:scale", Gf.Vec3d(r.uniform(0.4, 1), r.uniform(0.4, 1),
                                        r.uniform(0.4, 1)))

            #find where ground is and move the bush there
            x, y, z = str(prim_utils.get_prim_property(plant_path, "xformOp:translate")).strip('()').split(',')
            distance_to_ground = 100-check_raycast(stage, plant_path)
            prim_utils.set_prim_property(plant_path, "xformOp:translate", Gf.Vec3d(float(x), float(y),
                                        distance_to_ground))

            vegitation.append(plant_path)


def calculate_growth_multiplier(age_min, age_max):
    age = r.randint(age_min, age_max)
    multiplier = age / 100.0  # Linear growth multiplier
    return multiplier

def create_input_float(axis:str, indent=0):
    with ui.HStack(height=20):
        spaces = " " * (4 * indent)
        axis = spaces + axis
        float_field = ui_utils.float_builder(label=axis, min=0, max=100)
    return float_field

def create_input_int(axis:str, indent=0, tip=""):
    with ui.HStack(height=20):
        spaces = " " * (4 * indent)
        axis = spaces + axis
        int_field = ui_utils.int_builder(label=axis, min=0, tooltip=tip)

    return int_field

def delete_forest(stage:Stage):
    for tree_path in trees:
        if stage.GetPrimAtPath(tree_path):
            stage.RemovePrim(tree_path)
    for plant_path in vegitation:
        if stage.GetPrimAtPath(plant_path):
            stage.RemovePrim(plant_path)
    trees.clear()
    vegitation.clear()

def delete_hdr(stage:Stage):
    stage.RemovePrim('/Replicator/DomeLight_Xform')
    saved_lights.clear()

def delete_ground(stage:Stage):
    stage.RemovePrim('/World/terrain')
    saved_lights.clear()

def delete_rocks(stage:Stage):
    for rock_path in rocks:
        if stage.GetPrimAtPath(rock_path):
            stage.RemovePrim(rock_path)
    rocks.clear()

def create_light(dome_texture):
    global saved_lights
    lights = rep.create.light(
        light_type="Dome",
        rotation= (0,0,0),
        texture=dome_texture.get_value_as_string()
        )
    saved_lights.append(lights)

def check_raycast(stage, path_to_object):
    # Projects a raycast from 'origin', in the direction of 'rayDir', for a length of 'distance' cm
    # Parameters can be replaced with real-time position and orientation data  (e.g. of a camera)
    rayDir = carb.Float3(0.0, 0.0, -1.0)
    origin = carb.Float3(prim_utils.get_prim_property(path_to_object, "xformOp:translate"))
    distance = 10000.0
    # physX query to detect closest hit
    print("ffs hit something")
    hit = get_physx_scene_query_interface().raycast_closest(origin, rayDir, distance)
    if(hit["hit"]):
        #record distance from origin
        usdGeom = UsdGeom.Mesh.Get(stage, hit["rigidBody"])
        distance = hit["distance"]
        return distance

#terrain stuff, from: https://github.com/boredengineering/awesome_terrains/tree/v0.0 and self
#todo some kinda bug is creating terrain, where z and y are randomized but then these two values are dublicated for every x
def random_uniform_terrain(terrain, min_height, max_height, step=1, downsampled_scale=None):
    """
    Generate a terrain using Perlin noise for smoother elevation changes.

    Parameters:
        terrain (SubTerrain): the terrain
        min_height (float): the minimum height of the terrain [meters]
        max_height (float): the maximum height of the terrain [meters]
        step (float): minimum height change between two points [meters]
        downsampled_scale (float): distance between two randomly sampled points (must be larger or equal to
        terrain.horizontal_scale)

    """
    if downsampled_scale is None:
        downsampled_scale = terrain.horizontal_scale

    # switch parameters to discrete units
    min_height = int(min_height / terrain.vertical_scale)
    max_height = int(max_height / terrain.vertical_scale)
    step = int(step / terrain.vertical_scale)

    r.seed()  # Initialize the random seed

    seed = r.randint(0, 1000)  # Generate a random seed

    # Generate a larger height field
    num_rows = int(terrain.width * terrain.horizontal_scale / downsampled_scale)
    num_cols = int(terrain.length * terrain.horizontal_scale / downsampled_scale)
    height_field_downsampled = np.zeros((num_rows, num_cols))

    for i in range(num_rows):
        for j in range(num_cols):
            x = i * downsampled_scale / (terrain.width * terrain.horizontal_scale)
            y = j * downsampled_scale / (terrain.length * terrain.horizontal_scale)
            height_field_downsampled[i, j] = noise.pnoise2(x, y, octaves=6, base=seed)

    min_val = np.min(height_field_downsampled)
    max_val = np.max(height_field_downsampled)
    height_field_downsampled = np.interp(height_field_downsampled, (min_val, max_val), (min_height, max_height))

    # Extract a portion of the height field based on terrain.width and terrain.length
    height_field_extracted = height_field_downsampled[:terrain.width, :terrain.length]

    x = np.linspace(0, terrain.width * terrain.horizontal_scale, height_field_extracted.shape[0])
    y = np.linspace(0, terrain.length * terrain.horizontal_scale, height_field_extracted.shape[1])

    f = interpolate.interp2d(y, x, height_field_extracted, kind='cubic')

    x_upsampled = np.linspace(0, terrain.width * terrain.horizontal_scale, terrain.width)
    y_upsampled = np.linspace(0, terrain.length * terrain.horizontal_scale, terrain.length)
    z_upsampled = np.rint(f(y_upsampled, x_upsampled))

    terrain.height_field_raw += z_upsampled.astype(np.int16)
    return terrain

def convert_heightfield_to_trimesh(height_field_raw, horizontal_scale, vertical_scale, slope_threshold=None):
    """
    Convert a heightfield array to a triangle mesh represented by vertices and triangles.
    Optionally, corrects vertical surfaces above the provide slope threshold:

        If (y2-y1)/(x2-x1) > slope_threshold -> Move A to A' (set x1 = x2). Do this for all directions.
                   B(x2,y2)
                  /|
                 / |
                /  |
        (x1,y1)A---A'(x2',y1)

    Parameters:
        height_field_raw (np.array): input heightfield
        horizontal_scale (float): horizontal scale of the heightfield [meters]
        vertical_scale (float): vertical scale of the heightfield [meters]
        slope_threshold (float): the slope threshold above which surfaces are made vertical. If None no correction is
        applied (default: None)
    Returns:
        vertices (np.array(float)): array of shape (num_vertices, 3). Each row represents the location of each vertex
        [meters]
        triangles (np.array(int)): array of shape (num_triangles, 3). Each row represents the indices of the 3 vertices
        connected by this triangle.
    """
    hf = height_field_raw
    num_rows = hf.shape[0]
    num_cols = hf.shape[1]

    y = np.linspace(0, (num_cols-1)*horizontal_scale, num_cols)
    x = np.linspace(0, (num_rows-1)*horizontal_scale, num_rows)
    yy, xx = np.meshgrid(y, x)

    if slope_threshold is not None:

        slope_threshold *= horizontal_scale / vertical_scale
        move_x = np.zeros((num_rows, num_cols))
        move_y = np.zeros((num_rows, num_cols))
        move_corners = np.zeros((num_rows, num_cols))
        move_x[:num_rows-1, :] += (hf[1:num_rows, :] - hf[:num_rows-1, :] > slope_threshold)
        move_x[1:num_rows, :] -= (hf[:num_rows-1, :] - hf[1:num_rows, :] > slope_threshold)
        move_y[:, :num_cols-1] += (hf[:, 1:num_cols] - hf[:, :num_cols-1] > slope_threshold)
        move_y[:, 1:num_cols] -= (hf[:, :num_cols-1] - hf[:, 1:num_cols] > slope_threshold)
        move_corners[:num_rows-1, :num_cols-1] += (hf[1:num_rows, 1:num_cols] - hf[:num_rows-1, :num_cols-1] > slope_threshold)
        move_corners[1:num_rows, 1:num_cols] -= (hf[:num_rows-1, :num_cols-1] - hf[1:num_rows, 1:num_cols] > slope_threshold)
        xx += (move_x + move_corners*(move_x == 0)) * horizontal_scale
        yy += (move_y + move_corners*(move_y == 0)) * horizontal_scale

    # create triangle mesh vertices and triangles from the heightfield grid
    vertices = np.zeros((num_rows*num_cols, 3), dtype=np.float32)
    vertices[:, 0] = xx.flatten()
    vertices[:, 1] = yy.flatten()
    vertices[:, 2] = hf.flatten() * vertical_scale
    triangles = -np.ones((2*(num_rows-1)*(num_cols-1), 3), dtype=np.uint32)
    for i in range(num_rows - 1):
        ind0 = np.arange(0, num_cols-1) + i*num_cols
        ind1 = ind0 + 1
        ind2 = ind0 + num_cols
        ind3 = ind2 + 1
        start = 2*i*(num_cols-1)
        stop = start + 2*(num_cols-1)
        triangles[start:stop:2, 0] = ind0
        triangles[start:stop:2, 1] = ind3
        triangles[start:stop:2, 2] = ind1
        triangles[start+1:stop:2, 0] = ind0
        triangles[start+1:stop:2, 1] = ind2
        triangles[start+1:stop:2, 2] = ind3

    return vertices, triangles

def add_terrain_to_stage(stage, vertices, triangles, position=None, orientation=None):
    global terrain, terrain_mesh
    num_faces = triangles.shape[0]
    terrain_mesh = stage.DefinePrim("/World/terrain", "Mesh")
    terrain_mesh.GetAttribute("points").Set(vertices)
    terrain_mesh.GetAttribute("faceVertexIndices").Set(triangles.flatten())
    terrain_mesh.GetAttribute("faceVertexCounts").Set(np.asarray([3]*num_faces))

    terrain = XFormPrim(prim_path="/World/terrain",
                        name="terrain",
                        position=position,
                        orientation=orientation)
    terrain_collision(terrain, terrain_mesh) 

def terrain_collision(terrain, terrain_mesh):
    UsdPhysics.CollisionAPI.Apply(terrain.prim)
    collision_api = UsdPhysics.MeshCollisionAPI.Apply(terrain.prim)
    collision_api.CreateApproximationAttr().Set("triagnleMesh")
    physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(terrain.prim)
    physx_collision_api.GetContactOffsetAttr().Set(0.001)
    physx_collision_api.GetRestOffsetAttr().Set(0.00)
    terrain_mesh.CreateAttribute("physxsdfcollision:resolution", Sdf.ValueTypeNames.Int, True).Set(256)

    #terrain stuff ends

def test():
    #print(aaaaaa)
    print("wow you clicked")

# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class CompanyHelloWorldExtension(omni.ext.IExt):
    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id):
        print("[Forest generator] Forest generator startup")

        self._window = ui.Window("Forest generator", width=500, height=740)
        with self._window.frame:
            with ui.VStack():
                stage = omni.usd.get_context().get_stage()
                # Add a physics scene prim to stage
                scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
                # Set gravity vector
                scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
                scene.CreateGravityMagnitudeAttr().Set(981.0)
                
                def on_forest():
                    stage:Stage = omni.usd.get_context().get_stage()
                    n_of_birch = float(proportion_birch.get_value_as_float())
                    n_of_spruce = float(proportion_spruce.get_value_as_float())
                    n_of_pine = float(proportion_pine.get_value_as_float())
                    area_x = float(forest_size_x.get_value_as_float())
                    area_y = float(forest_size_y.get_value_as_float())
                    a_of_trees = int(density.get_value_as_int())
                    a_of_vegitation = int(density_veg.get_value_as_int())
                    v_age_min = int(age_min.get_value_as_int())
                    v_age_max = int(age_max.get_value_as_int())
                    v_vegitation = vegitation.get_value_as_bool()
                    if v_vegitation is True:
                        generate_vegitation(stage, a_of_vegitation, area_x, area_y)

                    pick_tree(a_of_trees, n_of_birch, n_of_spruce, n_of_pine, area_x, area_y, v_age_min, v_age_max)
                    
                    #print(dome_texture.get_value_as_string()) #this is how you read the values as str
                    #print(var.get_value_as_bool()) #this is how you read the values as boolean
                    #print(proportion_spruce.get_value_as_float()) #this is how you read the values as float
                    #print(var.get_value_as_int()) #this is how you read the values as int

                def on_del_forest():
                    stage:Stage = omni.usd.get_context().get_stage()
                    delete_forest(stage)

                def on_ground():
                    stage:Stage = omni.usd.get_context().get_stage()
                    area_x = float(forest_size_x.get_value_as_float())
                    area_y = float(forest_size_y.get_value_as_float())
                    terrain_roughness = float(roughness.get_value_as_float())
                    self.get_terrain(forest_size_x=forest_size_x, forest_size_y=forest_size_y,
                                    roughness=terrain_roughness)

                def on_rocks():
                    area_x = float(forest_size_x.get_value_as_float())
                    area_y = float(forest_size_y.get_value_as_float())
                    v_rockiness = float(rockiness.get_value_as_float())
                    create_rocks(stage, v_rockiness, area_x, area_y)

                def on_del_rocks():
                    stage:Stage = omni.usd.get_context().get_stage()
                    delete_rocks(stage)

                def on_del_ground():
                    stage:Stage = omni.usd.get_context().get_stage()
                    delete_ground(stage)

                def on_hdr():
                    global saved_lights
                    if len(saved_lights) == 0 and dome_texture.get_value_as_string() != " ":
                        create_light(dome_texture)
                
                def on_del_hdr():
                    stage:Stage = omni.usd.get_context().get_stage()
                    delete_hdr(stage)
                
                def test_clicking():
                    print("You clicked the string box")

                #bunch of ui stuff
                with ui.HStack(height=20):
                    ui.Label("Tree parameters:")

                density = create_input_int("Density (hover for info)", 1, "Trees per 10x10m area")
                
                with ui.HStack(height=20):
                    ui.Label("    Age range:")
                
                age_min = create_input_int("Min age", 2)
                age_max = create_input_int("Max age", 2)
                    
                with ui.HStack(height=20):
                    ui.Label("    Proportions of trees:")

                proportion_birch = create_input_float("Birch %", 2)
                proportion_spruce = create_input_float("Spruce %", 2)
                proportion_pine = create_input_float("Pine %", 2)
                    
                with ui.HStack(height=20):
                    ui.Label("Size of forest (m)")

                forest_size_x = create_input_int("Lenght", 1, "Going above 1km might result in crashes")
                forest_size_y = create_input_int("Width", 1, "Going above 1km might result in crashes")
                
                with ui.HStack(height=20):
                    ui.Label("\nTerrain parameters:")

                roughness = create_input_float("Elevation difference (m):", 1)

                #ig this could be what you use to pic the ground texture with
                with ui.HStack(height=20):
                    terrain_type = ui_utils.dropdown_builder(label="    Terrain type(WIP):",
                                                            items=["Not sure what to put here yet",
                                                                    "Not sure what to put here yet"])

                rockiness = create_input_int("Amount of rocks (hover)", 1,
                                            "How many rocks you want to spawn per 10x10m area")

                with ui.HStack(height=20):
                    vegitation = ui_utils.cb_builder(label="Other vegitation:")
                
                density_veg = create_input_int("Density (hover for info)", 0, "Other vegitation per 10x10m area")
                
                with ui.HStack(height=20):
                    dome_texture = str_builder(
                    label="HDRI texture path",
                    tooltip="Directory where the HDRI texture is stored. The path must not end in a slash.",
                    use_folder_picker=True,
                    item_filter_fn=lambda item: item.is_folder or item.path.endswith('.hdr'),
                    folder_dialog_title="Select Path",
                    folder_button_title="Select",
                    on_clicked_fn=test
                )

                with ui.HStack(height=70):
                    ui.Button("Generate forest", clicked_fn=on_forest)
                    ui.Button("Remove forest", clicked_fn=on_del_forest)
                
                with ui.HStack(height=70):
                    ui.Button("Generate ground", clicked_fn=on_ground)
                    ui.Button("Remove ground", clicked_fn=on_del_ground)

                with ui.HStack(height=70):
                    ui.Button("Generate rocks", clicked_fn=on_rocks)
                    ui.Button("Remove rocks", clicked_fn=on_del_rocks)
                
                with ui.HStack(height=70):
                    ui.Button("Load HDR", clicked_fn=on_hdr)
                    ui.Button("Remove HDR", clicked_fn=on_del_hdr)
                
    #this too is from the awesom terrain gen extension
    def get_terrain(self, forest_size_x, forest_size_y, roughness):
        stage = get_current_stage()
        # create all available terrain types
        num_terains = 1
        terrain_width = forest_size_x.get_value_as_int()
        terrain_length = forest_size_y.get_value_as_int()
        horizontal_scale = 0.25  # [m]
        vertical_scale = 0.005  # [m]
        num_rows = int(terrain_width/horizontal_scale)
        num_cols = int(terrain_length/horizontal_scale)
        heightfield = np.zeros((num_terains*num_rows, num_cols), dtype=np.int16)
        roughness = roughness/2

        def new_sub_terrain(): 
            return SubTerrain(width=num_rows, length=num_cols, vertical_scale=vertical_scale,
                            horizontal_scale=horizontal_scale)

        heightfield[0:num_rows, :] = random_uniform_terrain(new_sub_terrain(), min_height=-roughness,
                                                            max_height=roughness, step=0.01,
                                                            downsampled_scale=0.5).height_field_raw
        vertices, triangles = convert_heightfield_to_trimesh(heightfield, horizontal_scale=horizontal_scale,
                                                            vertical_scale=vertical_scale, slope_threshold=1.5)

        position = np.array([-terrain_width/2, terrain_length/2, 0])
        orientation = np.array([0.70711, 0.0, 0.0, -0.70711])
        add_terrain_to_stage(stage=stage, vertices=vertices, triangles=triangles, position=position,
                            orientation=orientation)

    def on_shutdown(self):
        print("[Forest generator] Forest generator shutdown")

#this too is from the awesome terrain gen extension
class SubTerrain:
    def __init__(self, terrain_name="terrain", width=256, length=256, vertical_scale=1.0, horizontal_scale=1.0):
        self.terrain_name = terrain_name
        self.vertical_scale = vertical_scale
        self.horizontal_scale = horizontal_scale
        self.width = width
        self.length = length
        self.height_field_raw = np.zeros((self.width, self.length), dtype=np.int16)
