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
    num_steps = 1000
    running_step = 0.06

    # Discretize the action space
    num_bins_angular_z = 10
    num_bins_linear_x  = 10
    num_bins_linear_y  = 10

    angular_z_bins = pd.cut([-1, 1], bins=num_bins_angular_z, retbins=True)[1][1:-1]
    linear_x_bins  = pd.cut([0, 1],  bins=num_bins_linear_x,  retbins=True)[1][1:-1]
    linear_y_bins  = pd.cut([-1, 1], bins=num_bins_linear_y,  retbins=True)[1][1:-1]
    
    num_states = 10 ** env.action_space.shape[0]
    num_actions = 10 ** env.action_space.shape[0]

    # Initialize the algorithm
    learner = QLearner(num_states=num_states,
                       num_actions=num_actions,
                       alpha=0.2,
                       gamma=1,
                       random_action_rate=0.9,
                       random_action_decay_rate=0.99)

    start_time = time.time()
    last_time_steps = np.ndarray(0)
    highest_reward = 0

    for episode in range(num_episodes):
        rospy.logwarn("############### START EPISODE " + str(episode) + " ###############")
        
        observation = env.reset()
        yaw, speed, lateral = observation
        rospy.logwarn(str(observation) + " " + str(to_bin(yaw, angular_z_bins)) + " " + str(to_bin(speed, linear_x_bins)) + " " + str(to_bin(lateral, linear_y_bins)))

        state = build_state([to_bin(yaw, angular_z_bins),
                             to_bin(speed, linear_x_bins),
                             to_bin(lateral, linear_y_bins)])
        
        action = learner.set_initial_state(state)
        rospy.logwarn(action)

        cumulated_reward = 0

        step = 0
        while True:
            step += 1            
             # Pick an action based on the current state
            rospy.logwarn("Episode " + str(episode) + " Step " + str(step) + ": " + str(action) + " reward: " + str(cumulated_reward))
            observation, reward, done, info = env.step(action)

            # Add to overall reward
            cumulated_reward += reward

            # Adjust learning vars
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            # Discretize new observation
            yaw, speed, lateral = observation
            state = build_state([to_bin(yaw, angular_z_bins),
                             to_bin(speed, linear_x_bins),
                             to_bin(lateral, linear_y_bins)])
            
            if done:
                reward = -200

            action = learner.move(state, reward)

            if done:
                last_time_steps = np.append(last_time_steps, [int(step + 1)])
                break

        rospy.logwarn("Episode " + str(episode) + " complete.")
        raw_input("Press enter to begin the next episode")

    l = last_time_steps.tolist()
    rospy.logwarn("Overall score: {0.4f}".format(last_time_steps.mean()))

    env.close()

