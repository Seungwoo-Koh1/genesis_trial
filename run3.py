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
        file='/home/rail/aloha_sim/aloha_isaac_sim/urdf/arx5_description_isaac.urdf',
        fixed=True,
    ),
)
franka = scene.add_entity(
    # gs.morphs.URDF(
    #     file='urdf/panda_bullet/panda.urdf',
    #     fixed=True,
    # ),
    gs.morphs.MJCF(file="xml/thin_box.xml"),
)


########################## build ##########################
scene.build()

# 왼쪽(fl)과 오른쪽(fr) 관절 이름 정의
fl_jnt_names = [f"fl_joint{i}" for i in range(1, 9)]
fr_jnt_names = [f"fr_joint{i}" for i in range(1, 9)]
jnt_names = fl_jnt_names + fr_jnt_names

# 관절 인덱스 가져오기
dofs_idx = [arx5.get_joint(name).dof_idx_local for name in jnt_names]

########################## 제어 게인 설정 ##########################
# 16개 관절에 대한 게인값 설정
kp_values = np.array([4000] * 16)  # 모든 관절에 동일한 kp 값 적용
kv_values = np.array([400] * 16)   # 모든 관절에 동일한 kv 값 적용

# Position gains
arx5.set_dofs_kp(
    kp=kp_values,
    dofs_idx_local=dofs_idx,
)

# Velocity gains
arx5.set_dofs_kv(
    kv=kv_values,
    dofs_idx_local=dofs_idx,
)

# Force limits
force_limit = 80
arx5.set_dofs_force_range(
    lower=np.array([-force_limit] * 16),
    upper=np.array([force_limit] * 16),
    dofs_idx_local=dofs_idx,
)

########################## 동작 실행 ##########################
# 초기화 동작
print("Executing initialization movement...")
for i in range(150):
    if i < 50:
        # 첫 번째 포즈: 모든 관절 약간 회전
        position = np.array([0.2] * 16)
        arx5.set_dofs_position(position, dofs_idx)
    elif i < 100:
        # 두 번째 포즈: 다른 방향으로 회전
        position = np.array([-0.2] * 16)
        arx5.set_dofs_position(position, dofs_idx)
    else:
        # 홈 포지션으로 복귀
        position = np.array([0.0] * 16)
        arx5.set_dofs_position(position, dofs_idx)
    scene.step()

# PD 제어를 통한 동작 수행
print("Executing main movement sequence...")
for i in range(1000):
    if i == 0:
        # 시작 포즈
        position = np.array([0.3] * 16)
        arx5.control_dofs_position(position, dofs_idx)
    elif i == 250:
        # 왼쪽 관절만 움직이기
        position = np.zeros(16)
        position[:8] = 0.5  # fl_joints
        arx5.control_dofs_position(position, dofs_idx)
    elif i == 500:
        # 오른쪽 관절만 움직이기
        position = np.zeros(16)
        position[8:] = 0.5  # fr_joints
        arx5.control_dofs_position(position, dofs_idx)
    elif i == 750:
        # 모든 관절 원위치
        position = np.zeros(16)
        arx5.control_dofs_position(position, dofs_idx)
    
    # 100 스텝마다 제어력과 내부 힘 출력
    if i % 100 == 0:
        print(f"Step {i}")
        print("Control force:", arx5.get_dofs_control_force(dofs_idx))
        print("Internal force:", arx5.get_dofs_force(dofs_idx))
    
    scene.step()

print("Simulation completed!")
