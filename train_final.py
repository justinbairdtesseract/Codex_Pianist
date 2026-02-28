import argparse
import copy
import sys
import time
from pathlib import Path

if "--headless" not in sys.argv:
    sys.argv.append("--headless")

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Final C-major curriculum PPO training for the pianist task.")
parser.add_argument("--num_envs", type=int, default=4096)
parser.add_argument("--max_iterations", type=int, default=2048)
parser.add_argument("--save_interval_seconds", type=int, default=300)
parser.add_argument("--chunk_iterations", type=int, default=64)
AppLauncher.add_app_launcher_args(parser)
args_cli, _ = parser.parse_known_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
import torch.nn as nn

from skrl import config as skrl_config
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
from skrl.memories.torch import RandomMemory
from skrl.models.torch import DeterministicMixin, GaussianMixin, Model
from skrl.resources.preprocessors.torch import RunningStandardScaler
from skrl.trainers.torch import SequentialTrainer

import pianist_task  # noqa: F401
from isaaclab_rl.skrl import SkrlVecEnvWrapper
from pianist_env_cfg import PianistEnvCfg


class Policy(GaussianMixin, Model):
    def __init__(self, observation_space, action_space, device):
        Model.__init__(self, observation_space, action_space, device)
        GaussianMixin.__init__(
            self,
            clip_actions=True,
            clip_log_std=True,
            min_log_std=-20.0,
            max_log_std=2.0,
            reduction="sum",
        )
        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, self.num_actions),
        )
        self.log_std_parameter = nn.Parameter(torch.zeros(self.num_actions))

    def compute(self, inputs, role):
        return self.net(inputs["states"]), self.log_std_parameter, {}


class Value(DeterministicMixin, Model):
    def __init__(self, observation_space, action_space, device):
        Model.__init__(self, observation_space, action_space, device)
        DeterministicMixin.__init__(self, clip_actions=False)
        self.net = nn.Sequential(
            nn.Linear(self.num_observations, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 1),
        )

    def compute(self, inputs, role):
        return self.net(inputs["states"]), {}


def install_skrl_device_override(resolved_device: str) -> None:
    original_parse_device = skrl_config.torch.parse_device

    def _parse_device(device=None, validate: bool = True):
        if device is None:
            device = resolved_device
        return original_parse_device(device, validate)

    skrl_config.torch.device = resolved_device
    skrl_config.torch.parse_device = staticmethod(_parse_device)


def main():
    rl_device = args_cli.device
    install_skrl_device_override(rl_device)

    env_cfg = PianistEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = rl_device
    env = gym.make("Isaac-Pianist-C4C5-v0", cfg=env_cfg)
    env = SkrlVecEnvWrapper(env, ml_framework="torch")

    device = torch.device(rl_device)
    memory = RandomMemory(memory_size=16, num_envs=args_cli.num_envs, device=device)

    models = {
        "policy": Policy(env.observation_space, env.action_space, device),
        "value": Value(env.observation_space, env.action_space, device),
    }

    cfg = copy.deepcopy(PPO_DEFAULT_CONFIG)
    cfg["rollouts"] = 16
    cfg["learning_epochs"] = 8
    cfg["mini_batches"] = 32
    cfg["discount_factor"] = 0.99
    cfg["lambda"] = 0.95
    cfg["learning_rate"] = 3e-4
    cfg["random_timesteps"] = 0
    cfg["learning_starts"] = 0
    cfg["grad_norm_clip"] = 1.0
    cfg["ratio_clip"] = 0.2
    cfg["value_clip"] = 0.2
    cfg["clip_predicted_values"] = True
    cfg["entropy_loss_scale"] = 0.0
    cfg["value_loss_scale"] = 2.0
    cfg["state_preprocessor"] = RunningStandardScaler
    cfg["state_preprocessor_kwargs"] = {"size": env.observation_space, "device": device}
    cfg["value_preprocessor"] = RunningStandardScaler
    cfg["value_preprocessor_kwargs"] = {"size": 1, "device": device}
    cfg["experiment"]["write_interval"] = 1000
    cfg["experiment"]["checkpoint_interval"] = 10000
    cfg["experiment"]["directory"] = "logs/skrl/pianist"
    cfg["experiment"]["experiment_name"] = "ppo_c_major_curriculum"

    agent = PPO(
        models=models,
        memory=memory,
        cfg=cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=device,
    )

    run_dir = Path(cfg["experiment"]["directory"]) / cfg["experiment"]["experiment_name"]
    run_dir.mkdir(parents=True, exist_ok=True)

    completed_iterations = 0
    last_save_time = time.time()

    while completed_iterations < args_cli.max_iterations:
        chunk = min(args_cli.chunk_iterations, args_cli.max_iterations - completed_iterations)
        trainer = SequentialTrainer(cfg={"timesteps": chunk, "headless": True}, env=env, agents=agent)
        trainer.train()
        completed_iterations += chunk

        now = time.time()
        if now - last_save_time >= args_cli.save_interval_seconds or completed_iterations >= args_cli.max_iterations:
            checkpoint_path = run_dir / f"checkpoint_iter_{completed_iterations}.pt"
            agent.save(str(checkpoint_path))
            last_save_time = now
            print(f"[INFO] Saved checkpoint: {checkpoint_path}")

    final_path = run_dir / "final_policy.pt"
    agent.save(str(final_path))
    print(f"[INFO] Saved final policy: {final_path}")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
