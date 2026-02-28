import gymnasium as gym


gym.register(
    id="Isaac-Pianist-C4C5-v0",
    entry_point="pianist_task.env:PianistRLEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": "pianist_env_cfg:PianistEnvCfg"},
)
