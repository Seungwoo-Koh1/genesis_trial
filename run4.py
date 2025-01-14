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
        file='/home/rail/aloha_sim/aloha_isaac_sim/urdf/arx5_description_isaac_wheeloper.urdf',
        fixed=True,
    ),
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
# Revolute 및 Prismatic 관절의 구동 범위를 구분
revolute_joints = dofs_idx[:12]  # revolute: joint1~6 (fl, fr)
prismatic_joints = dofs_idx[12:]  # prismatic: joint7~8 (fl, fr)

# Position gains
arx5.set_dofs_kp(
    kp=np.array([4000] * len(revolute_joints) + [2000] * len(prismatic_joints)),
    dofs_idx_local=dofs_idx,
)

# Velocity gains
arx5.set_dofs_kv(
    kv=np.array([400] * len(revolute_joints) + [200] * len(prismatic_joints)),
    dofs_idx_local=dofs_idx,
)

# Force limits
force_limit_revolute = 80
force_limit_prismatic = 40
arx5.set_dofs_force_range(
    lower=np.array([-force_limit_revolute] * len(revolute_joints) +
                   [-force_limit_prismatic] * len(prismatic_joints)),
    upper=np.array([force_limit_revolute] * len(revolute_joints) +
                   [force_limit_prismatic] * len(prismatic_joints)),
    dofs_idx_local=dofs_idx,
)

########################## 동작 실행 ##########################
# 초기화 동작
print("Executing initialization movement...")
for i in range(150):
    if i < 50:
        # 첫 번째 포즈: 모든 관절 약간 회전/이동
        position = np.concatenate([np.full(12, 0.2), np.full(4, 0.02)])
        arx5.set_dofs_position(position, dofs_idx)
    elif i < 100:
        # 두 번째 포즈: 반대 방향
        position = np.concatenate([np.full(12, -0.2), np.full(4, 0.03)])
        arx5.set_dofs_position(position, dofs_idx)
    else:
        # 홈 포지션
        position = np.zeros(16)
        arx5.set_dofs_position(position, dofs_idx)
    scene.step()

# PD 제어를 통한 동작 수행
print("Executing main movement sequence...")
for i in range(1000):
    if i == 0:
        # 시작 포즈
        position = np.concatenate([np.full(12, 0.3), np.full(4, 0.04)])
        arx5.control_dofs_position(position, dofs_idx)
    elif i == 250:
        # 왼쪽 관절만 움직이기
        position = np.zeros(16)
        position[:8] = np.concatenate([np.full(6, 0.5), np.full(2, 0.045)])
        arx5.control_dofs_position(position, dofs_idx)
    elif i == 500:
        # 오른쪽 관절만 움직이기
        position = np.zeros(16)
        position[8:] = np.concatenate([np.full(6, 0.5), np.full(2, 0.045)])
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
