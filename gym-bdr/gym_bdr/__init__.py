from gym.envs.registration import register

register(
    id='bdr-v0',
    entry_point='gym_bdr.envs:BdrEnv',
)

