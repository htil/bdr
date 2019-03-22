#!/usr/bin/env python

from gym import wrappers, spaces
import gym
import qlearn
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
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logwarn("Gym environment launched")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('bebop_bdr_qlearn')
    outdir = pkg_path + '/training_results'

    # Q-learning input parameters
    alpha = 0.5
    gamma = 0.7
    epsilon = 0.9
    
    epsilon_discount = 0.999
    num_episodes = 1000
    num_steps = 1000
    running_step: 0.06

    # Discretize the action space
    num_bins_angular_z = 21
    num_bins_linear_x  = 11
    num_bins_linear_y  = 21

    angular_z_bins = pd.cut([-1, 1], bins=num_bins_angular_z, retbin=True)[1][1:-1]
    linear_x_bins  = pd.cut([0, 1],  bins=num_bins_linear_x,  retbin=True)[1][1:-1]
    linear_y_bins  = pd.cut([-1, 1], bins=num_bins_linear_y,  retbin=True)[1][1:-1]
    
    num_possible_states = num_bins_angular_z * num_bins_linear_x * num_bins_linear_y

    # Initialize the algorithm
    qlearn = qlearn.QLearn(actions=num_possible_states, alpha=alpha, gamma=gamma, epsilon=epsilon)

    start_time = time.time()
    last_time_steps = np.ndarray(0)
    highest_reward = 0

	for episode in range(num_episodes):
        rospy.logwarn("############### START EPISODE " + str(episode) + " ###############")
        
        observation = env.reset()
        yaw, speed, lateral = observation

        state = build_state([to_bin(yaw, num_bins_angular_z),
                             to_bin(speed, num_bins_linear_x),
                             to_bin(lateral, num_bins_linear_y)])

        cumulated_reward = 0

        for step in range(num_steps):
             # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            rospy.logwarn("Episode " + str(episode) + "Step " + str(step) + ":" + action)

            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)

            # Add to overall reward
            cumulated_reward += reward

            # Adjust learning vars
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward
            
            if qlearn.epsilon > 0.05:
                qlearn.epsilon *= epsilon_discount

            # Discretize new observation
            yaw, speed, lateral = observation
            state = build_state([to_bin(yaw, num_bins_angular_z),
                             to_bin(speed, num_bins_linear_x),
                             to_bin(lateral, num_bins_linear_y)])

            if done:
                last_time_steps = np.append(last_time_steps, [int(step + 1)])
                reward = -200
                qlearn.learn(state, action, reward, nextState)
                break
            else:
                qlearn.learn(state, action, reward, nextState)
                state = nextState

        rospy.logwarn("Episode " + str(episode) + " complete.")
		raw_input("Press enter to begin the next episode")

    l = last_time_steps.tolist()
    rospy.logwarn("Overall score: {0.4f}".format(last_time_steps.mean()))

    env.close()

