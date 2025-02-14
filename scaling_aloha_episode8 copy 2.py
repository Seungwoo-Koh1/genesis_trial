import h5py
import numpy as np

def modify_hdf5_qpos(input_path, output_path, dataset_name, old_min, old_max, new_min, new_max):
    """
    HDF5 파일의 기존 데이터를 유지하면서 특정 데이터셋(qpos)만 변경하여 저장
    """
    with h5py.File(input_path, "r") as input_file:
        with h5py.File(output_path, "w") as output_file:
            # 기존 데이터 복사
            for name in input_file:
                input_file.copy(name, output_file)

            # qpos 변환 후 덮어쓰기
            if dataset_name in output_file:
                qpos_old = output_file[dataset_name][()]

                qpos_old = qpos_old * (180/np.pi)

                # 4,5번째 컬럼(인덱스 3,4) 서로 바꾸기
                #qpos_old[:, [3, 4]] = qpos_old[:, [4, 3]]

                # 11,12번째 컬럼(인덱스 10,11) 서로 바꾸기
                #qpos_old[:, [10, 11]] = qpos_old[:, [11, 10]]

                
                
                # 기존 로봇 qpos를 내 로봇 범위에 맞게 변환
                qpos_new = new_min + (qpos_old - old_min) * (new_max - new_min) / (old_max - old_min)

                qpos_new = qpos_new.copy()  # 변경 가능한 배열로 변환

                # 2번째 컬럼(인덱스 1, 8)에 대해 음수처리
                #qpos_new[:, 1] *= -1
                #qpos_new[:, 1] = (-qpos_new[:, 1]).astype(np.float64)
                #qpos_new[:, 8] = -qpos_new[:, 8]
                
                # 1,8번째 컬럼(인덱스 0, 7)에 대해 반대방향
                qpos_new[:, 0] = -qpos_new[:, 0]
                qpos_new[:, 7] = -qpos_new[:, 7]
                #2,9번째 컬럼(인덱스 1, 8)에 대해 반대방향
                qpos_new[:, 6] = 0
                qpos_new[:, 13] = 0

                #qpos_new[:, 6] = -qpos_new[:, 6]
                #qpos_new[:, 13] = -qpos_new[:, 13]

                # (인덱스 2, 9)에 대해 반대방향
                qpos_new[:, 2] = -qpos_new[:, 2]
                qpos_new[:, 9] = -qpos_new[:, 9]

                # (인덱스 3, 10)에 대해 반대방향
                qpos_new[:, 3] = -qpos_new[:, 3]
                qpos_new[:, 10] = -qpos_new[:, 10]

                # 4,5번째 컬럼(인덱스 3,4) 서로 바꾸기
                #qpos_new[:, [3, 4]] = qpos_new[:, [4, 3]]

                # 11,12번째 컬럼(인덱스 10,11) 서로 바꾸기
                #qpos_new[:, [10, 11]] = qpos_new[:, [11, 10]]


                # 변환된 qpos 값이 내 로봇의 허용 범위를 초과하지 않도록 제한
                #qpos_new = np.clip(qpos_new, new_min, new_max)

                # 기존 qpos 데이터 삭제 후 새로운 데이터 저장
                del output_file[dataset_name]
                output_file.create_dataset(dataset_name, data=qpos_new)

            print(f"✅ 변환된 데이터가 {output_path}에 저장되었습니다.")

# 기존 HDF5 파일 경로 및 새로운 파일 경로
input_hdf5_path = "/home/aloha/catkin_ws/src/mc_embodied_kit_ros/myarm_m/scripts/recorded_data/episode_0.hdf5"
output_hdf5_path = "/home/aloha/catkin_ws/src/mc_embodied_kit_ros/myarm_m/scripts/recorded_data/hdf5/episode_9.hdf5"
qpos_dataset_name = "observations/qpos"

# 기존 로봇 (데이터를 불러와 직접 확인 필요)
old_min_angles = np.array([-180, -101, -101, -107, -180, -180, 42, -180, -101, -101, -107, -180, -180, 42])  # 예제
old_max_angles = np.array([180, 101, 92, 130, 180, 180, 116, 180, 101, 92, 130, 180, 180, 116])

# 기존 로봇, 인덱스 3 <-> 4, 10 <-> 11 (데이터를 불러와 직접 확인 필요)
#old_min_angles = np.array([-180, -101, -101, -180, -107, -180, 42, -180, -101, -101, -180, -107, -180, 42])  # 예제
#old_max_angles = np.array([180, 101, 92, 180, 130, 180, 116, 180, 101, 92, 180, 130, 180, 116])

# 내 로봇 (MyArm MC-Embodied)
# myarm_min_angles = np.array([-168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0, -168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0])  # 직접 확인 필요
# myarm_max_angles = np.array([172.0, 90.0, 91.0, 148.0, 84.0, 146.0, 2.0, 172.0, 90.0, 91.0, 148.0, 84.0, 146.0, 2.0])
myarm_min_angles = np.array([-168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0, -168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0])  # 직접 확인 필요
myarm_max_angles = np.array([172.0, 83.0, 91.0, 148.0, 84.0, 146.0, 2.0, 172.0, 83.0, 91.0, 148.0, 84.0, 146.0, 2.0])



#clip_min_angles = np.array([-168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0, -168.0, -77.0, -86.0, -159.0, -95.0, -161.0, -118.0])  # 직접 확인 필요
#clip_max_angles = np.array([172.0, 83.0, 83.0, 148.0, 84.0, 146.0, 2.0, 172.0, 83.0, 83.0, 148.0, 84.0, 146.0, 2.0])

# 실행
modify_hdf5_qpos(input_hdf5_path, output_hdf5_path, qpos_dataset_name, 
                  old_min_angles, old_max_angles, myarm_min_angles, myarm_max_angles)

