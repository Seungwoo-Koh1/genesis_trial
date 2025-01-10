import genesis as gs

# 단순히 gs.init()만 호출
gs.init(backend=gs.cpu)

scene = gs.Scene()
plane = scene.add_entity(
    gs.morphs.Plane(),
)

# ARX5 로봇 로드
arx5 = scene.add_entity(
    gs.morphs.URDF(
        file='/home/rail/aloha_sim/aloha_isaac_sim/urdf/arx5_description_isaac.urdf',
        fixed=True,
    ),
)

scene.build()

# 시뮬레이션 실행
for i in range(10000):
    scene.step()
