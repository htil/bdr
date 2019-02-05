import gym
from gym import error, spaces, utils
from gym.utils import seeding

class BdrEnv(gym.Env):
   metadata = {'render.modes': ['human']}

   def __init__(self):
      self.action_space = spaces.Tuple((spaces.Box(low=1.0,  high=3.0, shape=1),
                                        spaces.Box(low=-1.0, high=1.0, shape=1)))

   def step(self, action):
       """
      Returns: obj, reward, done, info : tuple
      obj (object): an env-specific object representing
                    the observation of the environment

      reward (float): reward amount from previous action

      done (bool): whether it is time to reset environment

      info (dict): debug diagnostics
      """

      self.take_action(action)
      self.status = self.env.step()
      
      reward = self.get_reward()
      obj = self.env.getState()
      done = self.status

      return obj, reward, done, {}

   def take_action(self, action):

   def get_reward(self, action):

   def reset(self):

   def render(self, mode='human', close=False):


