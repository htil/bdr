import rospy
import numpy
from gym import spaces
from openai_ros.robot_envs import parrotdrone_env
from gym.envs.registration import register
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

timestep_limit_per_episode = 10000

register(
        id='BebopBdr-v0',
        entry_point='openai_ros:task_envs.bebop.bebop_bdr.Bebop2Env',
        timestep_limit=timestep_limit_per_episode,
    )

class Bebop2BdrEnv(bebop2_env.Bebop2Env):
    def __init__(self):
        # set variables
        self.cumulated_steps = 0.0

        # init the drone
        super(Bebop2Env, self).__init__()

    def _set_init_pose(self):
        self.land()
        return True

    def _init_env_variables(self):
        self.takeoff()
        self.cumulated_reward = 0.0        

    def _set_action(self, action):
        # send velocity command
        self.move(action[0], action[1])
        rospy.logdebug("action set: " + str(action) + "=>")

    def _get_obs(self):
        return self.camera_image_raw, self.speed

    def _is_done(self, observations):
        img, speed = observations   
        num_white_pixels = np.sum(img == 255)

        if num_white_pixels < 50:
            episode_done = True
        else:
            episode_done = False

        if episode_done:
            rospy.logerr("episode_done: " + str(episode_done) + "=>")
        else:
            rospy.logwarn("episode_done: "+str(episode_done) + "=>")

        return episode_done

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