import numpy as np
from stable_baselines3 import PPO

from rl.env.navbot_env import NavBotEnv


def main():
    model_path = "rl/models/ppo_navbot_reward_v1"
    n_eval_episodes = 10

    env = NavBotEnv()
    model = PPO.load(model_path)

    success_count = 0
    collision_count = 0
    truncated_count = 0

    episode_rewards = []
    episode_lengths = []

    for ep in range(n_eval_episodes):
        reset_ok = False
        for _ in range(3):
            try:
                obs, info = env.reset()
                reset_ok = True
                break
            except RuntimeError:
                pass

        if not reset_ok:
            print(f"Episode {ep + 1}: reset failed, skipping episode")
            continue
        done = False
        truncated = False
        ep_reward = 0.0
        ep_len = 0

        while not (done or truncated):
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)

            ep_reward += reward
            ep_len += 1

        episode_rewards.append(ep_reward)
        episode_lengths.append(ep_len)

        if info.get("goal_reached", False):
            success_count += 1
        elif info.get("collision", False):
            collision_count += 1
        elif truncated:
            truncated_count += 1

        print(
            f"Episode {ep + 1}: reward={ep_reward:.2f}, "
            f"length={ep_len}, success={info.get('goal_reached', False)}, "
            f"collision={info.get('collision', False)}, truncated={truncated}"
        )

    print("\n===== EVALUATION SUMMARY =====")
    print(f"Episodes: {n_eval_episodes}")
    print(f"Success rate: {success_count / n_eval_episodes:.2f}")
    print(f"Collision rate: {collision_count / n_eval_episodes:.2f}")
    print(f"Truncation rate: {truncated_count / n_eval_episodes:.2f}")
    print(f"Average reward: {np.mean(episode_rewards):.2f}")
    print(f"Average episode length: {np.mean(episode_lengths):.2f}")

    env.close()


if __name__ == "__main__":
    main()