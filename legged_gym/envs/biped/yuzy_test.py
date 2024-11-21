import math
import numpy as np
from isaacgym import gymapi
from isaacgym import gymutil
import os
# Initialize gym
gym = gymapi.acquire_gym()

# create a simulation
# configure common parameters
sim_params = gymapi.SimParams()
sim_params.dt = 1.0 / 60.0
sim_params.substeps = 2
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
# set PhysX-specific parameters
sim_params.physx.use_gpu = True
sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 6
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.contact_offset = 0.01
sim_params.physx.rest_offset = 0.0
# Parse arguments
args = gymutil.parse_arguments(description="yuzy11 test")
sim = gym.create_sim(args.compute_device_id,
                     args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    print("*** Failed to create sim")
    quit()

# Creating a Ground Plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)  # z-up
plane_params.distance = 0  # 距离
plane_params.static_friction = 1  # 静摩擦系数
plane_params.dynamic_friction = 1  # 动摩擦系数
plane_params.restitution = 0
gym.add_ground(sim, plane_params)

# Loading Assets
asset_root = os.path.dirname(__file__)+"/../../../resources/robots/biped"
asset_file = "xml/scene.xml"
# asset_file = "urdf/sa01_rl.urdf"
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = True
# asset_options.flip_visual_attachments = True
asset_options.armature = 0.01
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# Environments and Actors
num_envs = 4
env_spacing = 1.0
envs_per_row = 8
env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0)
env_upper = gymapi.Vec3(env_spacing, env_spacing, 0)

envs = []
actor_handles = []
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, 1.0)
    pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), 0.0 * math.pi)
    actor_handle = gym.create_actor(env, asset, pose, "MyActor", i, 1)
    actor_handles.append(actor_handle)


# Create viewer
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
gym.subscribe_viewer_keyboard_event(viewer, gymapi.KEY_R, "reset")
if viewer is None:
    print("*** Failed to create viewer")
    quit()
# save initial state for reset
initial_state = np.copy(gym.get_sim_rigid_body_states(sim, gymapi.STATE_ALL))
while not gym.query_viewer_has_closed(viewer):
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    for evt in gym.query_viewer_action_events(viewer):
        if evt.action == "reset" and evt.value > 0:
            gym.set_sim_rigid_body_states(sim, initial_state, gymapi.STATE_ALL)
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
# Some common handles for later use
