import rospy
import numpy as np
from gym import spaces
import bebop_env

from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

timestep_limit_per_episode = 10000

register(
        id='BebopBdr-v0',
        entry_point='bebop_bdr:Bebop2BdrEnv',
        timestep_limit=timestep_limit_per_episode,
    )

class Bebop2BdrEnv(bebop_env.Bebop2Env):
    def __init__(self):
        super(Bebop2BdrEnv, self).__init__()      

    def _set_action(self, action):
        print(action)
        #self.move(action)

    def _get_obs(self):
        return self.yaw, self.speed, self.lateral, self.img

    def _is_done(self, observations):
        yaw, speed, lateral, img = observations   
        num_white_pixels = np.sum(img == 255)

        if num_white_pixels < 50:
            return True
        else:
            return False

    def _compute_reward(self, observations, done):
        img, speed = observations 
        height, width = img.shape[:2]
        reward = 0.0

        # check center of image
        if img[height/2, width/2] == 255:
            reward += 10
        else:
            reward -= 10

        if speed > 5:
            reward += 2

        self.cumulated_reward += reward
        self.cumulated_steps += 1

        rospy.logdebug("reward = " + str(reward))
        rospy.logdebug("cumulated_reward = " + str(self.cumulated_reward))
        rospy.logdebug("cumulated_steps = " + str(self.cumulated_steps))
        
        return reward
