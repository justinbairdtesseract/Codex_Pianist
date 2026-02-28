import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import Articulation

def track_midi_goal(env, finger_cfg: SceneEntityCfg, target_pos: torch.Tensor | None = None) -> torch.Tensor:
    # Reward for Index (7), Middle (8), and Ring (9) fingers being near target_pos
    asset: Articulation = env.scene[finger_cfg.name]
    if target_pos is None:
        target_pos = env.current_target_pos
    tips_w = asset.data.body_state_w[:, finger_cfg.body_ids, :3]
    dist = torch.norm(tips_w - target_pos.unsqueeze(1), dim=-1)
    return 1.0 / (1.0 + torch.min(dist, dim=1)[0])

def finger_discipline_penalty(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    # Penalize Thumb (6) and Pinky (10) if they deviate from 0.0 (tucked)
    asset: Articulation = env.scene[asset_cfg.name]
    return torch.norm(asset.data.joint_pos[:, [6, 10]], dim=1)

def piano_force_reward(env, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    # Reward for 2N-5N force on the key
    force = env.scene.sensors[sensor_cfg.name].data.net_forces_w[:, sensor_cfg.body_ids, 2].abs()
    return torch.where((force > 2.0) & (force < 5.0), 1.0, 0.0).sum(dim=1)
