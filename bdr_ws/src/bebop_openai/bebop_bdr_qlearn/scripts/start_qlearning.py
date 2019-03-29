#!/usr/bin/env python

from gym import wrappers, spaces
from qlearner import QLearner
import gym
import bebop_bdr

import rospy
import rospkg

import time
import numpy as np
import pandas as pd
import pickle

def build_state(features):
    return int("".join(map(lambda feature: str(int(feature)), features)))

def to_bin(value, bins):
    return np.digitize(x=[value], bins=bins)[0]

if __name__ == '__main__':
    rospy.init_node('bebop_bdr_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('BebopBdr-v0')
    rospy.logwarn("Gym environment launched")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('bebop_bdr_qlearn')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)

    # Q-learning input parameters
    alpha = 0.5
    gamma = 0.7
    epsilon = 0.9
    
    epsilon_discount = 0.999
    num_episodes = 1000

    # Discretize the action space
    num_actions = 889

    # Size of observation space
    num_states = 56

    # Initialize the algorithm
    learner = QLearner(num_states=num_states,
                       num_actions=5,
                       alpha=0.9,
                       gamma=0.2,
                       random_action_rate=0.9,
                       random_action_decay_rate=0.999,
                       load_last=False)

    start_time = time.time()
    last_time_steps = np.ndarray(0)
    highest_reward = 0


    for episode in range(num_episodes):
        rospy.logwarn("############### START EPISODE " + str(episode) + " ###############")
        
        observation = env.reset()
        action = learner.set_initial_state(observation)

        cumulated_reward = 0
        step = 0
        
        while True:
            step += 1            
             # Pick an action based on the current state
            rospy.logwarn("Episode " + str(episode) + " Step " + str(step))
            observation, reward, done, info = env.step(action)

            if done:
                cumulated_reward += reward
                last_time_steps = np.append(last_time_steps, [int(step + 1)])
                break

            cumulated_reward += reward
            action = learner.move(observation, reward)
            print("main", str(action))

        rospy.logwarn("Episode " + str(episode) + " complete. " + "Episode Reward: " + str(cumulated_reward))
        learner.save()
        raw_input("Press enter to begin the next episode")

    l = last_time_steps.tolist()
    rospy.logwarn("Overall score: {0.4f}".format(last_time_steps.mean()))

    env.close()

