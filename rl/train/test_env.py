import time
import numpy as np

from rl.env.navbot_env import NavBotEnv


def main():
    env = NavBotEnv()

    obs, info = env.reset()
    print("Initial observation shape:", obs.shape)
    print("Initial observation:", obs)

    for step in range(20):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        print(f"Step {step}")
        print("Action:", action)
        print("Reward:", reward)
        print("Min lidar:", np.min(obs[:12]))
        print("Goal distance:", obs[12])
        print("Goal angle abs:", obs[13])

        if terminated or truncated:
            print("Episode ended.")
            break

        time.sleep(0.1)

    env.close()


if __name__ == "__main__":
    main()
