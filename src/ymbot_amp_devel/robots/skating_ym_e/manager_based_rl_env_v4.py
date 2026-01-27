# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# needed to import for allowing type-hinting: np.ndarray | None
from __future__ import annotations

import gymnasium as gym
import math
import numpy as np
import torch

from isaaclab.envs.common import VecEnvStepReturn
# from isaaclab.envs.manager_based_rl_env import ManagerBasedRLEnv
from humanoid_isaac_dash.third_party.isaaclab_rl.envs.manager_based_rl_env import ManagerBasedRLEnv
from isaaclab.envs.manager_based_rl_env_cfg import ManagerBasedRLEnvCfg


class ManagerBasedRLYMEnv(ManagerBasedRLEnv, gym.Env):
    """The superclass for the manager-based workflow reinforcement learning-based environments.

    This class inherits from :class:`ManagerBasedEnv` and implements the core functionality for
    reinforcement learning-based environments. It is designed to be used with any RL
    library. The class is designed to be used with vectorized environments, i.e., the
    environment is expected to be run in parallel with multiple sub-environments. The
    number of sub-environments is specified using the ``num_envs``.

    Each observation from the environment is a batch of observations for each sub-
    environments. The method :meth:`step` is also expected to receive a batch of actions
    for each sub-environment.

    While the environment itself is implemented as a vectorized environment, we do not
    inherit from :class:`gym.vector.VectorEnv`. This is mainly because the class adds
    various methods (for wait and asynchronous updates) which are not required.
    Additionally, each RL library typically has its own definition for a vectorized
    environment. Thus, to reduce complexity, we directly use the :class:`gym.Env` over
    here and leave it up to library-defined wrappers to take care of wrapping this
    environment for their agents.

    Note:
        For vectorized environments, it is recommended to **only** call the :meth:`reset`
        method once before the first call to :meth:`step`, i.e. after the environment is created.
        After that, the :meth:`step` function handles the reset of terminated sub-environments.
        This is because the simulator does not support resetting individual sub-environments
        in a vectorized environment.

    """

    cfg: ManagerBasedRLEnvCfg
    """Configuration for the environment."""

    def __init__(self, cfg: ManagerBasedRLEnvCfg, render_mode: str | None = None, **kwargs):
        """Initialize the environment.

        Args:
            cfg: The configuration for the environment.
            render_mode: The render mode for the environment. Defaults to None, which
                is similar to ``"human"``.
        """
        # -- counter for curriculum
        self.common_step_counter = 0
        self.total_episodes = 0

        # initialize the base class to setup the scene.
        super().__init__(cfg=cfg)
        # store the render mode
        self.render_mode = render_mode

        # initialize data and constants
        # -- init buffers
        self.episode_length_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        # -- set the framerate of the gym video recorder wrapper so that the playback speed of the produced video matches the simulation
        self.metadata["render_fps"] = 1 / self.step_dt

        print("[INFO]: Completed setting up the environment...")

    # """
    # Properties.
    # """

    # @property
    # def max_episode_length_s(self) -> float:
    #     """Maximum episode length in seconds."""
    #     return self.cfg.episode_length_s

    # @property
    # def max_episode_length(self) -> int:
    #     """Maximum episode length in environment steps."""
    #     return math.ceil(self.max_episode_length_s / self.step_dt)

    def _para_for_self_using_init(self):
        # self.duty = 0.7  # 保留但可能不再直接使用
        self.phase_offset = torch.tensor([0.5, 0.0], device=self.device, requires_grad=False).repeat(self.num_envs, 1)

        self.each_envs_total_mass = torch.sum(
            self.scene.articulations["robot"].root_physx_view.get_masses(), dim=1
        ).to(self.device)   # 先不用域随机的数据
        # 新增三相参数：duty1定义相位1结束，duty2定义相位2结束（示例值，可调整）

        """ a perfered parameter: freq = 0.65, duty=[0.5, 0.25, 0.25]"""
        self.base_frequencies = 0.65
        frequency_bias_range = 0.3
        self.base_duty1 = 0.5  # 相位1占空比（建议 0 < duty1 < duty2 < 1）

        duty_bias_range = 0.1
        self.duty_bias = duty_bias_range * (torch.rand(self.num_envs, device=self.device, requires_grad=False) - 0.5)
        self.duty1 = self.base_duty1 + self.duty_bias

        self.base_duty1_left = self.base_duty1
        self.base_duty1_right = self.base_duty1
        self.duty1_left = self.base_duty1_left + self.duty_bias
        self.duty1_right = self.base_duty1_right + self.duty_bias

        # self.duty2 = 0.5 + self.base_duty1 / 2  # 相位2占空比(我这里定义是结束位置) self.duty1+(1-self.duty1)/2

        self.gait_frequencies_bias = frequency_bias_range * (torch.rand(self.num_envs, device=self.device, requires_grad=False) - 0.5)
        self.gait_frequencies = self.base_frequencies + self.gait_frequencies_bias
        self.gait_indices = torch.rand(self.num_envs, device=self.device, requires_grad=False)  # 这一项有可能影响蒸馏(结论：换成0效果也没变好)
        # self.gait_indices = torch.zeros(self.num_envs, device=self.device, requires_grad=False)

        self.gait_indices = torch.remainder(self.gait_indices + self.gait_frequencies * self.step_dt, 1.0)

        """ parameters for adaptive gait (魔法数字，巨难调)"""
        self._min_duty = 0.4
        self._max_duty = 0.95    # 0.6
        self._min_freq = 0.65
        self._max_freq = 1.0
        self._max_tracking_error = 0.75   # 1 m/s 虽然最大速度指令是5m/s，但是误差到达_max_tracking_error m/s，就改用最大频率和最小占空比了
        self._max_ang_cmd = 1.0
        self._max_duty_diff = 0.3

        self.lin_vel_torso_error = torch.zeros(self.num_envs, device=self.device)
        self.ang_vel_error = torch.zeros(self.num_envs, device=self.device)

        # time constants for smoothing duty/frequency updates (seconds)
        # self._duty_ramp_tau = 0.15
        # self._freq_ramp_tau = 0.15

        self._duty_ramp_tau = 0.02
        self._freq_ramp_tau = 0.02

        self.alpha_d = 1.0 - math.exp(-float(self.step_dt) / max(1e-6, float(self._duty_ramp_tau)))
        self.alpha_f = 1.0 - math.exp(-float(self.step_dt) / max(1e-6, float(self._freq_ramp_tau)))

    def _map_vx_tracking_error_to_gait(self, x_tracking_error: torch.Tensor):
        """
            Linear mapping from tracking error -> (duty, frequency). 
            duty-> duty1: stance phase
            x_tracking_error: tensor (num_envs,)
            returns (duty_tensor, freq_tensor) each shape (num_envs,)
        """

        norm = torch.clamp(x_tracking_error / float(self._max_tracking_error), min=0.0, max=1.0)

        # duty: error small -> close to _max_duty (1.0), error large -> close to _min_duty (0.5)
        duty = self._min_duty + (1.0 - norm) * (self._max_duty - self._min_duty)

        # frequency: error small -> close to _min_freq (0.5), error large -> close to _max_freq (2.0)
        freq = self._min_freq + norm * (self._max_freq - self._min_freq)

        return duty.to(self.device), freq.to(self.device)

    def _map_angvel_cmd_to_duty_diff(self, ang_vel_cmd: torch.Tensor) -> torch.Tensor:
        """Linear mapping from angular velocity error -> duty_diff per env.

        - `ang_vel_error` shape: (num_envs,)
        - returns duty_diff shape: (num_envs,), clipped to [-max_duty_diff, max_duty_diff]
        """
        # configurable defaults (you can set these in _para_for_self_using_init)
        max_cmd = getattr(self, "_max_ang_cmd", 1.0)     # when |err| >= max_err -> saturate
        max_dd  = getattr(self, "_max_duty_diff", 0.2)         # max absolute duty diff

        # normalize (preserve sign), clamp to [-1,1]
        norm = torch.clamp(ang_vel_cmd / float(max_cmd), min=-1.0, max=1.0)

        # linear mapping -> duty_diff in [-max_dd, +max_dd]
        duty_diff = norm * float(max_dd)

        return duty_diff.to(self.device)

    def _update_gait_phase(self):
        """
        根据速度误差更新步态索引
        """
        """
        To do list:
        x_tracking_error应该通过 torso_link的vel计算
        """

        x_tracking_error = self.lin_vel_torso_error     # updating in RewTerm: track_torso_lin_vel_xy_yaw_frame_exp
        # x_tracking_error = self.command_manager.get_command("base_velocity")[:, 0] - self.scene.articulations["robot"].data.root_lin_vel_b[:, 0]
        # print(x_tracking_error)
        desired_duty, desired_freq = self._map_vx_tracking_error_to_gait(x_tracking_error)     # new duty and freq for each env according to tracking error
        # 比如左转，相当于左腿duty1增加，右腿duty1减少
        self.duty_diff = self._map_angvel_cmd_to_duty_diff(self.command_manager.get_command("base_velocity")[:, 2])

        # self.duty_diff = -0.4

        self.base_duty1_left = (
            (1.0 - self.alpha_d) * self.base_duty1_left
            + self.alpha_d * torch.clamp(
                (desired_duty + self.duty_diff),
                min=self._min_duty, max=self._max_duty)
        )
        self.base_duty1_right = (
            (1.0 - self.alpha_d) * self.base_duty1_right
            + self.alpha_d * torch.clamp(
                (desired_duty - self.duty_diff),
                min=self._min_duty, max=self._max_duty)
        )

        self.duty1_left = self.base_duty1_left + self.duty_bias  # duty_bias \in (-0.05, 0.05) max base_duty1_left/right = 0.95
        self.duty1_right = self.base_duty1_right + self.duty_bias

        self.duty2_left = 0.5 + self.duty1_left / 2   # self.duty1 + (1-self.duty1)/2
        self.duty2_right = 0.5 + self.duty1_right / 2   # self.duty1 + (1-self.duty1)/2

        # 每次只修正base_frequencies，但是保留每个env的 frequencies_bias
        self.base_frequencies = (1.0 - self.alpha_f) * self.base_frequencies + self.alpha_f * desired_freq
        self.gait_frequencies = self.base_frequencies + self.gait_frequencies_bias

        self.gait_indices = torch.remainder(self.gait_indices + self.gait_frequencies * self.step_dt, 1.0)

        # 计算每条腿的相位（考虑相位偏移）
        foot_indices = [
            self.phase_offset[:, 0] + self.gait_indices,  # 左腿
            self.phase_offset[:, 1] + self.gait_indices   # 右腿
        ]

        # 平滑参数（与原始代码一致）
        kappa_gait_probs = 0.02
        smoothing_cdf_start = torch.distributions.normal.Normal(0, kappa_gait_probs).cdf

        # stack left/right duty values -> shape (num_envs, 2)
        duty1_stack = torch.stack([self.duty1_left, self.duty1_right], dim=1)
        duty2_stack = torch.stack([self.duty2_left, self.duty2_right], dim=1)

        # 为每条腿和每个相位计算平滑权重（使用 per-leg duty1/duty2）
        smoothing_multiplier_list = []
        for i, foot_index in enumerate(foot_indices):
            f = torch.remainder(foot_index, 1.0)  # (num_envs,)
            d1 = duty1_stack[:, i]               # (num_envs,)
            d2 = duty2_stack[:, i]               # (num_envs,)

            # phase 1: stance
            w1 = (
                smoothing_cdf_start(f) * (1 - smoothing_cdf_start(f - d1))
                + smoothing_cdf_start(f - 1) * (1 - smoothing_cdf_start(f - d1 - 1))
            )

            # phase 2: skate
            w2 = (
                smoothing_cdf_start(f - d1) * (1 - smoothing_cdf_start(f - d2))
                + smoothing_cdf_start(f - d1 - 1) * (1 - smoothing_cdf_start(f - d2 - 1))
            )

            # phase 3: flight
            w3 = (
                smoothing_cdf_start(f - d2) * (1 - smoothing_cdf_start(f - 1))
                + smoothing_cdf_start(f - d2 - 1) * (1 - smoothing_cdf_start(f - 2))
            )

            phase_weights = torch.stack([w1, w2, w3], dim=1)  # (num_envs, 3)
            smoothing_multiplier_list.append(phase_weights)

        # 组合左右腿数据 shape [num_envs, 2, 3]
        self.smoothing_multiplier = torch.stack(smoothing_multiplier_list, dim=1)

    def load_managers(self):
        # note: this order is important since observation manager needs to know the command and action managers
        # and the reward manager needs to know the termination manager

        # 要准确整机重量的话把这句话放super().load_managers()后面，一般控接触力可能会用到
        self._para_for_self_using_init()
        print("[INFO]: Successfully initialize self-using parameters for manager-based RL environment...")

        # call the parent class to load the managers for observations and actions.
        super().load_managers()

        # already perform events at the start of the simulation in super().load_managers()
        # self._para_for_self_using_init()
        # print("[INFO]: Successfully initialize self-using parameters for manager-based RL environment...")


    """
    Operations - MDP
    """

    def step(self, action: torch.Tensor) -> VecEnvStepReturn:
        """Execute one time-step of the environment's dynamics and reset terminated environments.

        Unlike the :class:`ManagerBasedEnv.step` class, the function performs the following operations:

        1. Process the actions.
        2. Perform physics stepping.
        3. Perform rendering if gui is enabled.
        4. Update the environment counters and compute the rewards and terminations.
        5. Reset the environments that terminated.
        6. Compute the observations.
        7. Return the observations, rewards, resets and extras.

        Args:
            action: The actions to apply on the environment. Shape is (num_envs, action_dim).

        Returns:
            A tuple containing the observations, rewards, resets (terminated and truncated) and extras.
        """
        # process actions
        self.action_manager.process_action(action.to(self.device))

        self.recorder_manager.record_pre_step()

        # check if we need to do rendering within the physics loop
        # note: checked here once to avoid multiple checks within the loop
        is_rendering = self.sim.has_gui() or self.sim.has_rtx_sensors()

        # perform physics stepping
        for _ in range(self.cfg.decimation):
            self._sim_step_counter += 1
            # set actions into buffers
            self.action_manager.apply_action()
            # set actions into simulator
            self.scene.write_data_to_sim()
            # simulate
            self.sim.step(render=False)
            # render between steps only if the GUI or an RTX sensor needs it
            # note: we assume the render interval to be the shortest accepted rendering interval.
            #    If a camera needs rendering at a faster frequency, this will lead to unexpected behavior.
            if self._sim_step_counter % self.cfg.sim.render_interval == 0 and is_rendering:
                self.sim.render()
            # update buffers at sim dt
            self.scene.update(dt=self.physics_dt)

        self._update_gait_phase()
        # post-step:
        # -- update env counters (used for curriculum generation)
        self.episode_length_buf += 1  # step in current episode (per env)
        self.common_step_counter += 1  # total step (common for all envs)
        # -- check terminations
        self.reset_buf = self.termination_manager.compute()
        self.reset_terminated = self.termination_manager.terminated
        self.reset_time_outs = self.termination_manager.time_outs
        # -- reward computation
        self.reward_buf = self.reward_manager.compute(dt=self.step_dt)

        if len(self.recorder_manager.active_terms) > 0:
            # update observations for recording if needed
            self.obs_buf = self.observation_manager.compute()
            self.recorder_manager.record_post_step()

        # -- reset envs that terminated/timed-out and log the episode information
        reset_env_ids = self.reset_buf.nonzero(as_tuple=False).squeeze(-1)
        if len(reset_env_ids) > 0:
            # trigger recorder terms for pre-reset calls
            self.recorder_manager.record_pre_reset(reset_env_ids)

            self._reset_idx(reset_env_ids)
            # update articulation kinematics
            self.scene.write_data_to_sim()
            self.sim.forward()

            # if sensors are added to the scene, make sure we render to reflect changes in reset
            if self.sim.has_rtx_sensors() and self.cfg.rerender_on_reset:
                self.sim.render()

            # trigger recorder terms for post-reset calls
            self.recorder_manager.record_post_reset(reset_env_ids)

            # record total episodes
            self.total_episodes += len(reset_env_ids)

        # -- update command
        self.command_manager.compute(dt=self.step_dt)
        # -- step interval events
        if "interval" in self.event_manager.available_modes:
            self.event_manager.apply(mode="interval", dt=self.step_dt)
        # -- compute observations
        # note: done after reset to get the correct observations for reset envs
        self.obs_buf = self.observation_manager.compute()

        # return observations, rewards, resets and extras
        return self.obs_buf, self.reward_buf, self.reset_terminated, self.reset_time_outs, self.extras

