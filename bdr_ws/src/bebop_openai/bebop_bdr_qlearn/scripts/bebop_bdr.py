import rospy
import numpy as np
from gym import spaces
import bebop_env
import cv2
import time

from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion
from pynput.keyboard import Key, Listener

timestep_limit_per_episode = 10000

register(
        id='BebopBdr-v0',
        entry_point='bebop_bdr:Bebop2BdrEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class Bebop2BdrEnv(bebop_env.Bebop2Env):

    def on_press(self, key):
        try:
            if key == Key.space:
                self.done = True

        except AttributeError:
            pass

    def __init__(self):
        self.cumulated_reward = 0
        self.cumulated_steps = 0
        self.done = False
        self.listener = Listener(on_press=self.on_press)
        self.listener.start()
        super(Bebop2BdrEnv, self).__init__()      

    def _set_action(self, action):
        self.move(action)

    def _get_obs(self):
        h, w = self.image.shape[:2]
        wall = w//5

        r1 = np.sum(self.image[0:h, 0*wall:1*wall] == 255)
        r2 = np.sum(self.image[0:h, 1*wall:2*wall] == 255)
        r3 = np.sum(self.image[0:h, 2*wall:3*wall] == 255)
        r4 = np.sum(self.image[0:h, 3*wall:4*wall] == 255)
        r5 = np.sum(self.image[0:h, 4*wall:5*wall] == 255)
        
        region = max(r1, r2, r3, r4, r5)
        self.last_error = self.error

        if region == r1:
            self.error = -2
            state = 0
        elif region == r2:
            self.error = -1
            state = 1
        elif region == r3:
            self.error = 0
            state = 2
        elif region == r4:
            self.error = 1
            state = 3
        else:
            self.error = 2
            state = 4

        return state

    def _is_done(self, observations):
        return self.done

    def _compute_reward(self, observations, done):
        img = self.image
        height, width = img.shape[:2]
        hh = height/2
        ww = width/2

        center = np.sum(img[hh-80:hh+120, ww-75:ww+75] == 255)
        print(center)

        if center > 0:
            reward = 1

        self.cumulated_reward += reward
        self.cumulated_steps += 1

        rospy.logdebug("reward = " + str(reward))
        rospy.logdebug("cumulated_reward = " + str(self.cumulated_reward))
        rospy.logdebug("cumulated_steps = " + str(self.cumulated_steps))
        
        return reward
