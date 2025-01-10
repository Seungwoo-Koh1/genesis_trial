import numpy as np
import genesis as gs

########################## init ##########################
gs.init(backend=gs.gpu)

########################## create a scene ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(0, -3.5, 2.5),
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
plane = scene.add_entity(
    gs.morphs.Plane(),
)

arx5 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/rail/Downloads/slidertable.urdf',
        fixed=True,
    ),
)

########################## build ##########################
scene.build()

# Keep the simulation running for a short duration
print("Starting simulation...")
for _ in range(5000):  # Adjust the range for desired simulation steps
    scene.step()

print("Simulation completed!")

