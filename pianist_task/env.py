from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.envs import DirectRLEnv
from isaaclab.sim.spawners.from_files import GroundPlaneCfg, spawn_from_usd, spawn_ground_plane
from isaaclab.utils.math import combine_frame_transforms, quat_from_euler_xyz

from mdp import curriculums as pianist_curriculums
from pianist_env_cfg import (
    ARM_CENTER_DEG,
    HAND_MOUNT_ROTATE_XYZ_DEG,
    HAND_MOUNT_TRANSLATION_M,
    KEYBOARD_ROT_QUAT_WXYZ,
    KEYBOARD_TRANSLATION_M,
    PianistEnvCfg,
)

ARM_CENTER_RAD = torch.tensor([math.radians(v) for v in ARM_CENTER_DEG], dtype=torch.float32)
HAND_MOUNT_POS_B = torch.tensor(HAND_MOUNT_TRANSLATION_M, dtype=torch.float32)
KEYBOARD_TRANSLATION = torch.tensor(KEYBOARD_TRANSLATION_M, dtype=torch.float32)
KEYBOARD_ROT_QUAT = torch.tensor(KEYBOARD_ROT_QUAT_WXYZ, dtype=torch.float32)
C4_LOCAL_POSITION = torch.tensor([0.0, 0.05875, 0.02250], dtype=torch.float32)


class PianistRLEnv(DirectRLEnv):
    cfg: PianistEnvCfg

    def __init__(self, cfg: PianistEnvCfg, render_mode: str | None = None, **kwargs):
        self.current_midi_goal = torch.zeros((cfg.scene.num_envs, len(cfg.midi_notes)), device=cfg.sim.device)
        self.current_target_pos = torch.zeros((cfg.scene.num_envs, 3), device=cfg.sim.device)
        self.current_note_indices = torch.zeros((cfg.scene.num_envs,), device=cfg.sim.device, dtype=torch.long)
        self._keyboard_translation = KEYBOARD_TRANSLATION.to(cfg.sim.device)
        self._keyboard_rot_quat = KEYBOARD_ROT_QUAT.to(cfg.sim.device)
        super().__init__(cfg, render_mode, **kwargs)

        self._arm_joint_ids, _ = self.arm.find_joints([f"joint{i}" for i in range(1, 7)], preserve_order=True)
        self._eef_body_id = self.arm.find_bodies("link_eef")[0][0]

        self._finger_joint_ids, _ = self.hand.find_joints(
            ["index_proximal_joint", "middle_proximal_joint", "ring_proximal_joint"], preserve_order=True
        )
        self._thumb_joint_ids, _ = self.hand.find_joints(
            ["thumb_proximal_yaw_joint", "thumb_proximal_pitch_joint"], preserve_order=True
        )
        self._pinky_joint_ids, _ = self.hand.find_joints(["pinky_proximal_joint"], preserve_order=True)
        self._tip_body_ids, _ = self.hand.find_bodies(["index_tip", "middle_tip", "ring_tip"], preserve_order=True)

        self._arm_center_joint_pos = self.arm.data.default_joint_pos.clone()
        self._arm_center_joint_pos[:, self._arm_joint_ids] = ARM_CENTER_RAD.to(self.device)
        self._arm_joint_vel_zeros = torch.zeros_like(self.arm.data.default_joint_vel)

        self._hand_joint_center = self.hand.data.default_joint_pos.clone()
        self._hand_joint_vel_zeros = torch.zeros_like(self.hand.data.default_joint_vel)
        self._hand_root_velocity_zeros = torch.zeros((self.num_envs, 6), device=self.device, dtype=torch.float32)
        self._mount_pos_b = HAND_MOUNT_POS_B.to(self.device).repeat(self.num_envs, 1)

        roll = torch.full((self.num_envs,), math.radians(HAND_MOUNT_ROTATE_XYZ_DEG[0]), device=self.device)
        pitch = torch.full((self.num_envs,), math.radians(HAND_MOUNT_ROTATE_XYZ_DEG[1]), device=self.device)
        yaw = torch.full((self.num_envs,), math.radians(HAND_MOUNT_ROTATE_XYZ_DEG[2]), device=self.device)
        self._mount_quat_b = quat_from_euler_xyz(roll, pitch, yaw)

        self.base_c4_pos = self.scene.env_origins + self._keyboard_translation + C4_LOCAL_POSITION.to(self.device)

        self.actions = torch.zeros((self.num_envs, self.cfg.action_space), device=self.device, dtype=torch.float32)
        all_env_ids = torch.arange(self.num_envs, device=self.device, dtype=torch.long)
        self._sample_targets(all_env_ids)
        self._reset_robot_state(all_env_ids)

    def _setup_scene(self):
        self.arm = Articulation(self.cfg.arm_cfg)
        self.hand = Articulation(self.cfg.hand_cfg)

        spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
        spawn_from_usd(
            "/World/envs/env_0/keyboard",
            self.cfg.keyboard_cfg,
            translation=KEYBOARD_TRANSLATION_M,
            orientation=KEYBOARD_ROT_QUAT_WXYZ,
        )

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

        self.scene.articulations["arm"] = self.arm
        self.scene.articulations["hand"] = self.hand

        light_cfg = sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=2500.0)
        light_cfg.func("/World/light", light_cfg)

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        self.actions = actions.clone().clamp_(-1.0, 1.0)

    def _apply_action(self) -> None:
        arm_targets = self._arm_center_joint_pos[:, self._arm_joint_ids] + self.cfg.arm_action_scale_rad * self.actions[:, :6]
        arm_limits = self.arm.data.soft_joint_pos_limits[:, self._arm_joint_ids]
        arm_targets = torch.clamp(arm_targets, arm_limits[..., 0], arm_limits[..., 1])
        self.arm.set_joint_position_target(arm_targets, joint_ids=self._arm_joint_ids)

        self.arm.write_data_to_sim()
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(dt=0.0)

        eef_pose = self.arm.data.body_link_pose_w[:, self._eef_body_id, :7]
        hand_pos_w, hand_quat_w = combine_frame_transforms(
            eef_pose[:, :3], eef_pose[:, 3:7], self._mount_pos_b, self._mount_quat_b
        )
        hand_root_pose = torch.cat((hand_pos_w, hand_quat_w), dim=-1)
        self.hand.write_root_pose_to_sim(hand_root_pose)
        self.hand.write_root_velocity_to_sim(self._hand_root_velocity_zeros)

        hand_targets = self._hand_joint_center[:, self._finger_joint_ids] + self.cfg.finger_action_scale_rad * self.actions[:, 6:9]
        hand_limits = self.hand.data.soft_joint_pos_limits[:, self._finger_joint_ids]
        hand_targets = torch.clamp(hand_targets, hand_limits[..., 0], hand_limits[..., 1])
        self.hand.set_joint_position_target(hand_targets, joint_ids=self._finger_joint_ids)

        tucked_targets = torch.zeros((self.num_envs, len(self._thumb_joint_ids) + len(self._pinky_joint_ids)), device=self.device)
        self.hand.set_joint_position_target(tucked_targets[:, : len(self._thumb_joint_ids)], joint_ids=self._thumb_joint_ids)
        self.hand.set_joint_position_target(tucked_targets[:, len(self._thumb_joint_ids) :], joint_ids=self._pinky_joint_ids)

    def _get_observations(self) -> dict:
        arm_obs = self.arm.data.joint_pos[:, self._arm_joint_ids]
        finger_obs = self.hand.data.joint_pos[:, self._finger_joint_ids]
        obs = torch.cat((arm_obs, finger_obs, self.current_midi_goal), dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        tip_positions = self.hand.data.body_state_w[:, self._tip_body_ids, :3]
        distances = torch.norm(tip_positions - self.current_target_pos.unsqueeze(1), dim=-1)
        alignment_reward = 1.0 / (1.0 + torch.min(distances, dim=1).values)

        discipline_ids = self._thumb_joint_ids + self._pinky_joint_ids
        discipline_penalty = torch.norm(self.hand.data.joint_pos[:, discipline_ids], dim=1)

        key_height = self.current_target_pos[:, 2].unsqueeze(1)
        press_depth = key_height - tip_positions[:, :, 2]
        force_reward = torch.where((press_depth > 0.0) & (press_depth < 0.010), 1.0, 0.0).sum(dim=1)

        return 20.0 * alignment_reward - 15.0 * discipline_penalty + 5.0 * force_reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        terminated = torch.zeros_like(time_out)
        return terminated, time_out

    def _reset_idx(self, env_ids: Sequence[int] | None):
        if env_ids is None:
            env_ids = self.arm._ALL_INDICES
        env_ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        super()._reset_idx(env_ids)
        if env_ids.numel() == 0:
            return
        self._sample_targets(env_ids)
        self._reset_robot_state(env_ids)

    def _reset_robot_state(self, env_ids: torch.Tensor):
        arm_root_state = self.arm.data.default_root_state[env_ids].clone()
        arm_root_state[:, :3] += self.scene.env_origins[env_ids]
        self.arm.write_root_pose_to_sim(arm_root_state[:, :7], env_ids=env_ids)
        self.arm.write_root_velocity_to_sim(arm_root_state[:, 7:], env_ids=env_ids)
        self.arm.write_joint_state_to_sim(
            self._arm_center_joint_pos[env_ids], self._arm_joint_vel_zeros[env_ids], env_ids=env_ids
        )
        self.arm.set_joint_position_target(self._arm_center_joint_pos[env_ids], env_ids=env_ids)
        self.arm.write_data_to_sim()
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(dt=0.0)

        eef_pose = self.arm.data.body_link_pose_w[env_ids, self._eef_body_id, :7]
        hand_pos_w, hand_quat_w = combine_frame_transforms(
            eef_pose[:, :3], eef_pose[:, 3:7], self._mount_pos_b[env_ids], self._mount_quat_b[env_ids]
        )
        hand_root_pose = torch.cat((hand_pos_w, hand_quat_w), dim=-1)
        self.hand.write_root_pose_to_sim(hand_root_pose, env_ids=env_ids)
        self.hand.write_root_velocity_to_sim(self._hand_root_velocity_zeros[env_ids], env_ids=env_ids)
        self.hand.write_joint_state_to_sim(
            self._hand_joint_center[env_ids], self._hand_joint_vel_zeros[env_ids], env_ids=env_ids
        )
        self.hand.set_joint_position_target(self._hand_joint_center[env_ids], env_ids=env_ids)
        self.scene.write_data_to_sim()
        self.sim.forward()
        self.scene.update(self.physics_dt)

    def _sample_targets(self, env_ids: torch.Tensor):
        curriculum_term = getattr(getattr(self.cfg, "curriculum", None), "note_selection", None)
        if curriculum_term is not None:
            curriculum_term.func(self, env_ids, **curriculum_term.params)
        else:
            pianist_curriculums.sequenced_note_curriculum(self, env_ids)
