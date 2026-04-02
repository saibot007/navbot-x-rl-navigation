import os

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from rl.env.navbot_env import NavBotEnv


def main():
    os.makedirs("rl/models", exist_ok=True)
    os.makedirs("rl/logs", exist_ok=True)

    env = NavBotEnv()

    check_env(env, warn=True)

    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        tensorboard_log="./rl/logs/ppo_navbot_tensorboard/",
        device="cpu",
    )

    model.learn(total_timesteps=50000)
    model.save("rl/models/ppo_navbot_curriculum_stage1")

    env.close()


if __name__ == "__main__":
    main()