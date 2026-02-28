from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg, ViewerCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

import mdp.curriculums as pianist_curriculums


ROOT_DIR = Path(__file__).resolve().parent
SIM_DIR = ROOT_DIR / "pianist_robot_v1" / "simulation"
ARM_USD = str(SIM_DIR / "uf850.usd")
HAND_USD = str(SIM_DIR / "inspire_rh56dfx_2r_hand.usd")
KEYBOARD_USD = str(SIM_DIR / "assets" / "keyboard" / "robopianist_keyboard.usda")

ARM_CENTER_DEG = (-12.9, -18.1, -25.5, -15.1, 73.2, -45.0)
KEYBOARD_TRANSLATION_M = (0.676093, -0.087234, 0.988099)
KEYBOARD_ROT_QUAT_WXYZ = (0.0, 0.0, 0.0, 1.0)
HAND_MOUNT_TRANSLATION_M = (0.004, 0.0, 0.026)
HAND_MOUNT_ROTATE_XYZ_DEG = (0.0, 0.0, 225.0)
OBSERVATION_DIM = 16
ACTION_DIM = 9
C_MAJOR_NOTES = (60, 62, 64, 65, 67, 69, 71)


@configclass
class PianistEnvCfg(DirectRLEnvCfg):
    decimation = 2
    episode_length_s = 4.0
    action_space = ACTION_DIM
    observation_space = OBSERVATION_DIM
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,
        env_spacing=2.5,
        replicate_physics=True,
        clone_in_fabric=True,
    )

    viewer: ViewerCfg = ViewerCfg(eye=(2.5, 2.5, 2.0), lookat=(0.5, 0.0, 1.0))

    arm_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/arm",
        spawn=sim_utils.UsdFileCfg(usd_path=ARM_USD),
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.74)),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                stiffness=180.0,
                damping=24.0,
            )
        },
    )

    hand_cfg: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/hand",
        spawn=sim_utils.UsdFileCfg(usd_path=HAND_USD, activate_contact_sensors=True),
        actuators={
            "hand": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                stiffness=80.0,
                damping=8.0,
            )
        },
    )

    keyboard_cfg = sim_utils.UsdFileCfg(usd_path=KEYBOARD_USD)

    arm_action_scale_rad = 0.35
    finger_action_scale_rad = 0.60
    midi_notes = C_MAJOR_NOTES
    curriculum_update_steps = 500

    def __post_init__(self):
        super().__post_init__()
        self.curriculum = SimpleNamespace()
        self.curriculum.note_selection = CurrTerm(
            func=pianist_curriculums.sequenced_note_curriculum,
            params={},
        )
