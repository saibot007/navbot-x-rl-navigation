from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

from rl.env.navbot_env import NavBotEnv


def main():
    env = NavBotEnv()

    check_env(env, warn=True)

    model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=3e-4,
    n_steps=1024,
    batch_size=64,
    gamma=0.99,
    gae_lambda=0.95,
    ent_coef=0.01,
    tensorboard_log="./rl/logs/ppo_navbot_tensorboard/",
    device="cpu"
)

    model.learn(total_timesteps=5000)
    model.save("rl/models/ppo_navbot_first")

    env.close()


if __name__ == "__main__":
    main()