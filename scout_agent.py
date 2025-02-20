import os
import torch
import argparse
import numpy as np
from datetime import date

from mlagents_envs.base_env import ActionTuple
from mlagents_envs.environment import UnityEnvironment
from torch.utils.tensorboard import SummaryWriter

from PPO import PPO

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

state_dim = 11
action_dim = 2
max_episodes = 100_000
reward_stat_freq = 100
init_episode = 0
action_std = 1.
lr = 0.0003
# Linearly decay learning rate
lr_decay_freq = 250
batch_size = 1024
buffer_size = batch_size * 40

today = date.today().strftime("%Y-%m-%d")
level = 'differential_walls'


def train(env, scout_model, args, writer):
    print('Training...')
    print(f'Starting learning rate: {lr}')
    print(f'Starting action_std: {action_std}')

    behavior_name = list(env.behavior_specs.keys())[0]
    decision_steps, terminal_steps = env.get_steps(behavior_name)
    agent_ids = list(decision_steps)

    episode = init_episode

    rewards = []
    agent_reward = {agent_id: 0.0 for agent_id in agent_ids}
    agent_episode = {agent_id: episode for agent_id in agent_ids}

    weight_folder = os.path.join('weights', f'weights_{today}_{level}')
    os.makedirs(weight_folder, exist_ok=True)

    while episode < max_episodes:
        for agent_id in agent_ids:
            current_obs = decision_steps[agent_id].obs[0]
            action = ActionTuple(scout_model.select_action(agent_id, torch.tensor(current_obs).to(device)).reshape(1, action_dim))
            env.set_action_for_agent(behavior_name, agent_id, action)

        env.step()

        decision_steps, terminal_steps = env.get_steps(behavior_name)

        for agent_id in agent_ids:
            done = False

            if agent_id in terminal_steps:
                terminal = terminal_steps[agent_id]
                done = True
            else:
                terminal = decision_steps[agent_id]

            agent_reward[agent_id] += terminal.reward
            scout_model.store_obs(agent_id, terminal.reward, done)

            if done:
                last_obs = terminal.obs[0]
                print(f'{agent_id=}, ep: {agent_episode[agent_id]} reward: {agent_reward[agent_id]}')
                agent_episode[agent_id] += 1

                if scout_model.num_of_stored_obs(agent_id) >= buffer_size:
                    print('Updating weights...')
                    scout_model.update(agent_id, torch.tensor(last_obs, dtype=torch.float32).to(device))

                rewards.append(agent_reward[agent_id])
                agent_reward[agent_id] = 0.0
                episode += 1

                # Linear decay of learning rate
                if episode % lr_decay_freq == 0:
                    learning_rate = max((1 - episode / max_episodes) * lr, 1e-8)
                    scout_model.update_learning_rate(learning_rate)

                # Display stats in command line and tensorboard
                if episode % reward_stat_freq == 0:
                    mean_reward = np.mean(rewards[-reward_stat_freq:])
                    rewards = []
                    scout_model.save_weights(f'{level}_e{episode}_{today}', weight_folder)

                    print(f'Mean reward between {episode - reward_stat_freq}-{episode}: {mean_reward}')
                    print(f'Current log_action_std: {scout_model.model.log_action_std}')

                    writer.add_scalar('Mean reward', mean_reward, episode)
                writer.flush()

    env.close()


def inference(env, scout_model):
    print('Inference...')

    while True:
        behavior_name = list(env.behavior_specs.keys())[0]
        decision_steps, _ = env.get_steps(behavior_name)
        agent_ids = list(decision_steps)

        for agent_id in agent_ids:
            current_obs = decision_steps[agent_id].obs[0]
            action = ActionTuple(scout_model.select_action(agent_id, torch.tensor(current_obs).to(device)).reshape(1, action_dim))
            env.set_action_for_agent(behavior_name, agent_id, action)
        env.step()


def main(args):
    print('Loading Unity Environment...')
    env = UnityEnvironment()
    env.reset()

    behavior_name = list(env.behavior_specs.keys())[0]
    decision_steps, _ = env.get_steps(behavior_name)
    agent_ids = list(decision_steps)

    load_weights_folder, load_weights_file = os.path.split(args.weights)

    scout_model = PPO(state_dim, action_dim, lr, action_std, agent_ids, hidden_size=128, mini_batch_size=batch_size, inference=(args.mode == 'inference'))
    scout_model.load(load_weights_file, load_weights_folder)

    if args.mode == 'train':
        writer = SummaryWriter(log_dir=os.path.join('train_logs', f'{today}_{level}'))
        train(env, scout_model, args, writer)
        writer.flush()
        writer.close()
    elif args.mode == 'inference':
        inference(env, scout_model)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', dest='mode', action='store_const', const='train', help='training mode')
    parser.add_argument('--inference', dest='mode', action='store_const', const='inference', help='inference mode')
    parser.set_defaults(mode='train')
    parser.add_argument('--weights', type=str, default='weights/navigation.pth', help='Weights to be loaded for the agent')
    args = parser.parse_args()
    main(args)
