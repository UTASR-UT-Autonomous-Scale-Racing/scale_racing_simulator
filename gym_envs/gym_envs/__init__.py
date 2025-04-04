import gymnasium as gym
from gymnasium.envs.registration import register

register(
    id="gym_envs/RacingEnv-v0",
    entry_point="gym_envs.envs:RacingEnv",
)