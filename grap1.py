import genesis as gs
import numpy as np

########################## init ##########################
gs.init(backend=gs.gpu)

########################## create a scene ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(3, -1, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=30,
        max_FPS=60,
    ),
    sim_options=gs.options.SimOptions(
        dt=0.01,
    ),
    show_viewer=True,
)

########################## entities ##########################
# Plane
plane = scene.add_entity(gs.morphs.Plane())

# Cube with default size and position
cube_size = (0.04, 0.04, 0.04)
cube_pos = (0.65, 0.0, 0.02)
cube = scene.add_entity(
    gs.morphs.Box(
        size=cube_size,
        pos=cube_pos,
    )
)

# Franka Robot
franka = scene.add_entity(gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"))

########################## build ##########################
scene.build()

# Set initial safe position
initial_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785, 0.04, 0.04])  # Safe home position
franka.set_dofs_position(initial_qpos)

# Define DOF indices
motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

# Set control gains
franka.set_dofs_kp(np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]))
franka.set_dofs_kv(np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]))
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)
end_effector = franka.get_link("hand")

########################## Robot Actions ##########################
# Move to pre-grasp pose (above the cube)
pre_grasp_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.25]  # Slightly above the cube
pre_grasp_quat = [0, 1, 0, 0]

# Intermediate safe position
safe_middle_pos = [0.4, 0.0, 0.4]
middle_qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array(safe_middle_pos),
    quat=np.array(pre_grasp_quat)
)

# Move to intermediate position
path1 = franka.plan_path(
    qpos_goal=middle_qpos,
    num_waypoints=100  # Use default settings for smooth transitions
)
for waypoint in path1:
    franka.control_dofs_position(waypoint)
    scene.step()

# Move to pre-grasp position
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array(pre_grasp_pos),
    quat=np.array(pre_grasp_quat)
)
path2 = franka.plan_path(
    qpos_goal=qpos,
    num_waypoints=200
)
for waypoint in path2:
    franka.control_dofs_position(waypoint)
    scene.step()

# Hold the pose for a short duration
for i in range(100):
    scene.step()

# Reach and grasp cube
reach_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + cube_size[2] / 2]
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array(reach_pos),
    quat=np.array(pre_grasp_quat)
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()

# Grasp the cube
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)  # Close gripper
for i in range(100):
    scene.step()

# Lift the cube
lift_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.28]
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array(lift_pos),
    quat=np.array(pre_grasp_quat)
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()

