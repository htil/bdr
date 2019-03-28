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
        flat = self.camera_image_scaled.flatten()
        state = int(''.join(["0" if x == 0 else "1" for x in flat]), 2)
        return state

    def _is_done(self, observations):
        return self.done

    def _compute_reward(self, observations, done):
        img = self.camera_image_raw
        height, width = img.shape[:2]
        hh = height/2
        ww = width/2

        center = np.sum(img[hh-80:hh+120, ww-75:ww+75] == 255)
        print(center)
        # check center of image
        if center> 0:
            reward = center // 10
        else:
            reward = -50

        self.cumulated_reward += reward
        self.cumulated_steps += 1

        rospy.logdebug("reward = " + str(reward))
        rospy.logdebug("cumulated_reward = " + str(self.cumulated_reward))
        rospy.logdebug("cumulated_steps = " + str(self.cumulated_steps))
        
        return reward
