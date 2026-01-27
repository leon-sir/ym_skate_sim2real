import argparse
import os 

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="Visualize G1 robot by directly setting joint positions from motion data (with joint order matching)."
)
AppLauncher.add_app_launcher_args(parser)
parser.add_argument(
    "--file_path",
    type=str,
    default="default.npz",
    help="Path to the motion data file (.npz). Can be absolute or relative."
)
parser.add_argument(
    "--slow_down_factor",
    type=float,
    default=1.0,
    help="Playback speed multiplier (<1 for slower, >1 for faster)."
)
parser.add_argument(
    "--start_frame",
    type=int,
    default=None,
    help="Start frame for visualization (default: 0)"
)
parser.add_argument(
    "--end_frame",
    type=int,
    default=None,
    help="End frame for visualization (default: last frame of dataset)"
)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.actuators import ImplicitActuatorCfg
from legged_lab.assets.ymbot_e.ymbot import Ymboy_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR, ISAACLAB_NUCLEUS_DIR

# Fixed settings
NUM_ENVS = 1


class G1SceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", 
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )
    # Disable gravity and actuation so the robot is purely kinematic
    Robot = Ymboy_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=Ymboy_CFG.spawn.usd_path,
            activate_contact_sensors=Ymboy_CFG.spawn.activate_contact_sensors,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=3.0,
                max_angular_velocity=3.0,
                max_depenetration_velocity=10.0,
            ),
            articulation_props=Ymboy_CFG.spawn.articulation_props 
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["legs"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "feet": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["feet"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "shoulders": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["shoulders"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            ),
            "arms": ImplicitActuatorCfg(
                joint_names_expr=Ymboy_CFG.actuators["arms"].joint_names_expr,
                stiffness=0.0,
                damping=0.0,
                effort_limit=0.0
            )
        },
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.8),  # G1 default height
            joint_pos={".*": 0.0}
        )
    )


def run_simulator(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    data_tensor: torch.Tensor,
    motion_dof_indexes: list[int],
    start_frame: int,  # 新增参数
    end_frame: int,    # 新增参数
    frame_dt: float,
    slow_down_factor: float
):
    sim_dt = sim.get_physics_dt()
    frame_time = 0.0
    frame_range = end_frame - start_frame + 1  # 计算帧范围长度

    while simulation_app.is_running():
        frame_time += sim_dt * slow_down_factor
        # 计算当前帧在范围内的偏移量，再转换为绝对帧数
        current_offset = int(frame_time / frame_dt) % frame_range
        current_frame = start_frame + current_offset  # 绝对帧数

        # 打印绝对帧数
        print(f"当前帧数: {current_frame}/{end_frame}")

        # 获取当前帧配置数据
        configuration = data_tensor[current_frame]
        
        # 解析根状态（XYZ位置和四元数姿态）
        root_pos = configuration[:3]
        root_rot_xyzw = configuration[3:7]  # QXQYQZQW (XYZW)
        root_rot_wxyz = root_rot_xyzw[[3, 0, 1, 2]]  # 转换为WXYZ

        # 解析关节位置（根据索引映射调整顺序）
        raw_joint_pos = configuration[7:]
        joint_pos = raw_joint_pos[motion_dof_indexes].clone()
        joint_vel = torch.zeros_like(joint_pos)

        # 更新根状态
        root_state = scene["Robot"].data.default_root_state.clone()
        root_state[:, :3] = root_pos + scene.env_origins  # 添加环境原点偏移
        root_state[:, 3:7] = root_rot_wxyz
        root_state[:, 7:] = 0.0  # 零速度

        # 写入仿真状态
        scene["Robot"].write_root_pose_to_sim(root_state[:, :7])
        scene["Robot"].write_root_velocity_to_sim(root_state[:, 7:])
        scene["Robot"].write_joint_state_to_sim(joint_pos, joint_vel)
        
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view((2.5, 0.0, 1.5), (0.0, 0.0, 0.8))

    # 加载CSV运动数据
    data = np.genfromtxt(args_cli.file_path, delimiter=',', dtype=np.float32)
    data_tensor = torch.tensor(data, device=args_cli.device)
    total_frames = data_tensor.shape[0]
    frame_dt = 1.0 / 30.0  # 根据README，G1数据集帧率为30 FPS

    # 设置默认帧范围
    start_frame = args_cli.start_frame if args_cli.start_frame is not None else 0
    end_frame = args_cli.end_frame if args_cli.end_frame is not None else (total_frames - 1)

    # 验证帧范围有效性
    if total_frames <= 0:
        raise ValueError("数据集中没有有效的帧数据")
    if start_frame < 0:
        raise ValueError(f"起始帧不能为负数: {start_frame}")
    if end_frame >= total_frames:
        raise ValueError(f"结束帧({end_frame})不能超过总帧数({total_frames - 1})")
    if start_frame >= end_frame:
        raise ValueError(f"起始帧({start_frame})必须小于结束帧({end_frame})")
    frame_range = end_frame - start_frame + 1
    print(f"可视化帧范围: {start_frame}~{end_frame} (共{frame_range}帧)")
    

    # 创建场景
    scene_cfg = G1SceneCfg(NUM_ENVS, env_spacing=10.0)
    scene = InteractiveScene(scene_cfg)

    sim.reset()
    print("[INFO] G1关节位置可视化设置完成。开始仿真...")

    # 获取机器人数据
    robot = scene["Robot"]
    robot_joint_names = robot.data.joint_names

    # 根据README定义CSV关节顺序
    csv_joint_order = [
        "left_hip_pitch_joint",               # 1
        "left_hip_roll_joint",                # 2
        "left_hip_yaw_joint",                 # 3
        "left_knee_joint",                    # 4
        "left_ankle_pitch_joint",             # 5
        "left_ankle_roll_joint",              # 6
        "right_hip_pitch_joint",              # 7
        "right_hip_roll_joint",               # 8  
        "right_hip_yaw_joint",                # 9
        "right_knee_joint",                   # 10
        "right_ankle_pitch_joint",            # 11
        "right_ankle_roll_joint",             # 12
        "waist_yaw_joint",                    # 13
        "left_shoulder_pitch_joint",          # 14
        "left_shoulder_roll_joint",           # 15
        "left_shoulder_yaw_joint",            # 16
        "left_elbow_joint",                   # 17
        "left_wrist_yaw_joint",               # 18
        "right_shoulder_pitch_joint",         # 19
        "right_shoulder_roll_joint",          # 20
        "right_shoulder_yaw_joint",           # 21
        "right_elbow_joint",                  # 22
        "right_wrist_yaw_joint",              # 23
    ]

    # 创建关节索引映射（CSV关节顺序 -> 机器人关节顺序）
    motion_dof_indexes = [csv_joint_order.index(name) for name in robot_joint_names]

    # 运行仿真
    run_simulator(
        sim, scene, data_tensor, motion_dof_indexes, 
        start_frame, end_frame, frame_dt, args_cli.slow_down_factor  # 传递处理后的帧范围参数
    )


if __name__ == "__main__":
    main()
    simulation_app.close()