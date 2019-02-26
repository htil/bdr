import logging
from gym.envs.registration import register
logger = logging.getLogger(__name__)

register(
    id='GazeboBrainDroneRaceCamera-v0'
    entry_point='gym_gazebo.envs.bebop:GazeboBrainDroneRaceCameraEnv',
)

# completed: lcb 02/26/19 11:47:00 AM