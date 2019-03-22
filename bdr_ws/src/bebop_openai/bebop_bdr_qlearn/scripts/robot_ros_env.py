import numpy as np
import rospy
import gym
from gym.utils import seeding

class RobotRosEnv(gym.Env):
    def __init__(self):
        self.episode_num = 0
        self.cumulated_episode_reward = 0
        rospy.logwarn("ROS Environment initialized\n")

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        #self._set_action(action)
        obs = self._get_obs()
        done = self._is_done(obs)
        info = {}
        reward = self._compute_reward(obs, done)
        self.cumulated_episode_reward += reward
        return obs, reward, done, info

    def reset(self):
	    #self.takeoff()
        obs = self._get_obs()
        return obs

    def close(self):
        rospy.logwarn("Closing ROS Environment")
        rospy.signal_shutdown("Closing ROS Environment")

    def _update_episode(self):
        self.episode_num += 1
        self.cumulated_episode_reward = 0

    def _get_obs(self):
        raise NotImplementedError()

    def _set_action(self, action):
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        raise NotImplementedError()

    def _is_done(self, observations):
        raise NotImplementedError()