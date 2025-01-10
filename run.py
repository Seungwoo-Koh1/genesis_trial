import genesis as gs

if not gs.is_initialized():
    gs.init(backend=gs.cpu)

scene = gs.Scene()

plane = scene.add_entity(
    gs.morphs.Plane(),
)
franka = scene.add_entity(
     gs.morphs.URDF(
         file='/home/rail/aloha_sim/aloha_isaac_sim/urdf/arx5_description_isaac.urdf',
         fixed=True,
     ),

)

scene.build()
for i in range(1000):
    scene.step()
