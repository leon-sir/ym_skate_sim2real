# Copyright (c) 2021-2024, The RSL-RL Project Developers.
# All rights reserved.
# Original code is licensed under the BSD-3-Clause license.
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The Legged Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# This file contains code derived from the RSL-RL, Isaac Lab, and Legged Lab Projects,
# with additional modifications by the TienKung-Lab Project,
# and is distributed under the BSD-3-Clause license.

import math

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (  # noqa:F401
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
    RslRlRndCfg,
    RslRlSymmetryCfg,
)

import legged_lab.mdp as mdp
# from legged_lab.assets.tienkung2_lite import TIENKUNG2LITE_CFG
from legged_lab.assets.ymbot_e.ymbot import Ymboy_CFG
from legged_lab.envs.base.base_config import (
    ActionDelayCfg,
    BaseSceneCfg,
    CommandRangesCfg,
    CommandsCfg,
    DomainRandCfg,
    EventCfg,
    HeightScannerCfg,
    NoiseCfg,
    NoiseScalesCfg,
    NormalizationCfg,
    ObsScalesCfg,
    PhysxCfg,
    RobotCfg,
    SimCfg,
)
from legged_lab.terrains import GRAVEL_TERRAINS_CFG, ROUGH_TERRAINS_CFG  # noqa:F401


@configclass
class GaitCfg:
    # gait_air_ratio_l: float = 0.38
    # gait_air_ratio_r: float = 0.38
    # gait_phase_offset_l: float = 0.38
    # gait_phase_offset_r: float = 0.88
    # gait_cycle: float = 0.85

    # # 快速行走参数配置
    # gait_air_ratio_l: float = 0.3     # 增加滞空比例 (38% -> 50%)
    # gait_air_ratio_r: float = 0.3     # 增加滞空比例 (38% -> 50%)
    # gait_phase_offset_l: float = 0.25  # 调整相位偏移
    # gait_phase_offset_r: float = 0.75  # 调整相位偏移
    # gait_cycle: float = 0.8           # 缩短步态周期 (0.85s -> 0.6s)


    # 快速行走参数配置
    gait_air_ratio_l: float = 0.48     # 滞空比例 
    gait_air_ratio_r: float = 0.48     # 滞空比例
    gait_phase_offset_l: float = 0.0  # 相位偏移
    gait_phase_offset_r: float = 0.5  # 相位偏移
    gait_cycle: float = 0.7           # 步态周期


@configclass
class LiteRewardCfg:
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp, 
        weight=2.0, 
        params={"std": 0.5}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp, 
        weight=2.0, 
        params={"std": 0.5}
    )
    lin_vel_z_l2 = RewTerm(
        func=mdp.lin_vel_z_l2, 
        weight=-1.0
    )
    ang_vel_xy_l2 = RewTerm(
        func=mdp.ang_vel_xy_l2, 
        weight=-0.05
    )
    energy = RewTerm(
        func=mdp.energy, 
        weight=-1e-3
    )
    dof_acc_l2 = RewTerm(
        func=mdp.joint_acc_l2, 
        weight=-2.5e-7
    )
    action_rate_l2 = RewTerm(
        func=mdp.action_rate_l2, 
        weight=-0.01
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={
            "sensor_cfg": SceneEntityCfg(
                "contact_sensor", body_names=[".*knee_link", ".*shoulder_roll_link", ".*elbow_link", "pelvis"]
            ),
            "threshold": 1.0,
        },
    )
    # 奖励函数功能重复，只保留一个
    # body_orientation_l2 = RewTerm(
    #     func=mdp.body_orientation_l2, 
    #     weight=-2.0,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names="pelvis")}, 
    # )
    flat_orientation_l2 = RewTerm(
        func=mdp.flat_orientation_l2, 
        # weight=-1.0,
        weight=-0.01,
    )
    termination_penalty = RewTerm(
        func=mdp.is_terminated, 
        weight=-200.0
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        # weight=-0.25,
        weight=-0.01,
        params={
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll_link"),
            "asset_cfg": SceneEntityCfg("robot", body_names=".*ankle_roll_link"),
        },
    )
    feet_force = RewTerm(
        func=mdp.body_force,
        weight=-3e-3,
        params={
            "sensor_cfg": SceneEntityCfg("contact_sensor", body_names=".*ankle_roll_link"),
            "threshold": 500,
            "max_reward": 400,
        },
    )
    feet_too_near = RewTerm(
        func=mdp.feet_too_near_humanoid,
        weight=-2.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=[".*ankle_roll_link"]), "threshold": 0.2},
    )
    feet_stumble = RewTerm(
        func=mdp.feet_stumble,
        weight=-2.0,
        params={"sensor_cfg": SceneEntityCfg("contact_sensor", body_names=[".*ankle_roll_link"])},
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits, 
        weight=-2.0
    )
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.15,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*hip_yaw_joint",
                    ".*hip_roll_joint",
                    ".*shoulder_pitch_joint",
                    ".*elbow_joint",
                ],
            )
        },
    )
    joint_deviation_arms = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*shoulder_roll_joint", ".*shoulder_yaw_joint"])},
    )
    joint_deviation_legs = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.02,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*hip_pitch_joint",
                    ".*knee_joint",
                    ".*ankle_pitch_joint",
                    ".*ankle_roll_joint",
                ],
            )
        },
    )

    gait_feet_frc_perio = RewTerm(func=mdp.gait_feet_frc_perio, weight=1.0, params={"delta_t": 0.02})
    gait_feet_spd_perio = RewTerm(func=mdp.gait_feet_spd_perio, weight=1.0, params={"delta_t": 0.02})
    gait_feet_frc_support_perio = RewTerm(func=mdp.gait_feet_frc_support_perio, weight=0.6, params={"delta_t": 0.02})

    ankle_torque = RewTerm(
        func=mdp.ankle_torque, 
        weight=-0.0005
    )
    ankle_action = RewTerm(
        func=mdp.ankle_action, 
        weight=-0.001
    )
    # hip_roll_action = RewTerm(func=mdp.hip_roll_action, weight=-1.0)
    # hip_yaw_action = RewTerm(func=mdp.hip_yaw_action, weight=-1.0)
    feet_y_distance = RewTerm(
        func=mdp.feet_y_distance, 
        weight=-2.0
    )


@configclass
class TienKungWalkFlatEnvCfg:
    amp_motion_files_display = ["legged_lab/envs/tienkung/datasets/motion_visualization/walk.txt"]
    device: str = "cuda:0"
    scene: BaseSceneCfg = BaseSceneCfg(
        max_episode_length_s=20.0,
        num_envs=4096,
        env_spacing=2.5,
        # robot=TIENKUNG2LITE_CFG,
        robot=Ymboy_CFG,
        # terrain_type="generator",
        # terrain_generator=GRAVEL_TERRAINS_CFG,
        terrain_type="plane",
        terrain_generator= None,
        max_init_terrain_level=5,
        height_scanner=HeightScannerCfg(
            enable_height_scan=False,
            prim_body_name="pelvis",
            resolution=0.1,
            size=(1.6, 1.0),
            debug_vis=False,
            drift_range=(0.0, 0.0),  # (0.3, 0.3)
        ),
    )
    robot: RobotCfg = RobotCfg(
        actor_obs_history_length=10,
        critic_obs_history_length=10,
        action_scale=0.25,
        terminate_contacts_body_names=[".*knee_link", ".*shoulder_roll_link", ".*elbow_link", "pelvis"],
        feet_body_names=[".*ankle_roll_link"],
        # 新增：质心高度阈值，当机器人质心低于此高度时触发重置
        # 0.5米是一个合理的阈值，低于此高度通常意味着机器人已经摔倒
        min_height_threshold=0.5,
    )
    reward = LiteRewardCfg()
    gait = GaitCfg()
    normalization: NormalizationCfg = NormalizationCfg(
        obs_scales=ObsScalesCfg(
            lin_vel=1.0,
            ang_vel=1.0,
            projected_gravity=1.0,
            commands=1.0,
            joint_pos=1.0,
            joint_vel=1.0,
            actions=1.0,
            height_scan=1.0,
        ),
        clip_observations=100.0,
        clip_actions=100.0,
        height_scan_offset=0.5,
    )
    commands: CommandsCfg = CommandsCfg(
        resampling_time_range=(10.0, 10.0),
        rel_standing_envs=0.2,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=CommandRangesCfg(
            lin_vel_x=(-0.6, 1.0), 
            lin_vel_y=(-0.5, 0.5), 
            ang_vel_z=(-1.57, 1.57), 
            heading=(-math.pi, math.pi)
        ),
    )
    noise: NoiseCfg = NoiseCfg(
        add_noise=False,
        noise_scales=NoiseScalesCfg(
            lin_vel=0.2,
            ang_vel=0.2,
            projected_gravity=0.05,
            joint_pos=0.01,
            joint_vel=1.5,
            height_scan=0.1,
        ),
    )
    domain_rand: DomainRandCfg = DomainRandCfg(
        events=EventCfg(
            physics_material=EventTerm(
                func=mdp.randomize_rigid_body_material,
                mode="startup",
                params={
                    "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
                    # 设置摩擦力范围为固定值，关闭物理材料随机化
                    "static_friction_range": (0.8, 0.8),  # 固定为0.8
                    "dynamic_friction_range": (0.6, 0.6),  # 固定为0.6
                    "restitution_range": (0.0, 0.0),      # 固定为0.0
                    "num_buckets": 1,  # 减少为1个bucket
                },
            ),
            add_base_mass=EventTerm(
                func=mdp.randomize_rigid_body_mass,
                mode="startup",
                params={
                    "asset_cfg": SceneEntityCfg("robot", body_names="pelvis"),
                    # 质量随机化范围设为0，关闭质量随机化
                    "mass_distribution_params": (0.0, 0.0),
                    "operation": "add",
                },
            ),
            reset_base=EventTerm(
                func=mdp.reset_root_state_uniform,
                mode="reset",
                params={
                    # 大幅缩小初始位置和姿态的随机范围
                    "pose_range": {"x": (-0.01, 0.01), "y": (-0.01, 0.01), "yaw": (-0.01, 0.01)},
                    "velocity_range": {
                        "x": (-0.01, 0.01),
                        "y": (-0.01, 0.01),
                        "z": (-0.01, 0.01),
                        "roll": (-0.01, 0.01),
                        "pitch": (-0.01, 0.01),
                        "yaw": (-0.01, 0.01),
                    },
                },
            ),
            reset_robot_joints=EventTerm(
                func=mdp.reset_joints_by_scale,
                mode="reset",
                params={
                    # 关节位置范围设为固定值1.0，关闭关节位置随机化
                    "position_range": (1.0, 1.0),
                    "velocity_range": (0.0, 0.0),
                },
            ),
            push_robot=EventTerm(
                func=mdp.push_by_setting_velocity,
                mode="interval",
                interval_range_s=(1000.0, 1000.0),  # 设置极长的间隔，基本不会触发
                # 推力范围已经很小，保持现状
                params={"velocity_range": {"x": (-0.001, 0.001), "y": (-0.001, 0.001)}},
            ),
        ),
        action_delay=ActionDelayCfg(enable=False, params={"max_delay": 5, "min_delay": 0}),
    )
    sim: SimCfg = SimCfg(dt=0.005, decimation=4, physx=PhysxCfg(gpu_max_rigid_patch_count=10 * 2**15))


@configclass
class TienKungWalkAgentCfg(RslRlOnPolicyRunnerCfg):
    seed = 42
    device = "cuda:0"
    num_steps_per_env = 24
    max_iterations = 50000
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        class_name="ActorCritic",
        init_noise_std=1.0,
        noise_std_type="scalar",
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        class_name="AMPPPO",
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.005,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
        normalize_advantage_per_mini_batch=False,
        symmetry_cfg=None,  # RslRlSymmetryCfg()
        rnd_cfg=None,  # RslRlRndCfg()
    )
    clip_actions = None
    save_interval = 200
    runner_class_name = "AmpOnPolicyRunner"
    experiment_name = "walk"
    run_name = ""
    logger = "tensorboard"
    neptune_project = "walk"
    wandb_project = "walk"
    resume = False
    load_run = ".*"
    load_checkpoint = "model_.*.pt"

    # amp parameter
    amp_reward_coef = 0.5  # 算法内部
    amp_motion_files = [
        "/home/ps/sh_robot/TienKung-Lab/TienKung-Lab/legged_lab/assets/ymbot_e/motions/txt/walk1_subject1_clipped_2700_3340.txt",
        "/home/ps/sh_robot/TienKung-Lab/TienKung-Lab/legged_lab/assets/ymbot_e/motions/txt/walk1_subject1_clipped_90_200.txt",
        "/home/ps/sh_robot/TienKung-Lab/TienKung-Lab/legged_lab/assets/ymbot_e/motions/txt/backward_clipped_0_150.txt",
    ]
    amp_num_preload_transitions = 200000
    amp_task_reward_lerp = 0.7 # 任务和风格
    amp_discr_hidden_dims = [1024, 512, 256]
    min_normalized_std = [0.05] * 20
