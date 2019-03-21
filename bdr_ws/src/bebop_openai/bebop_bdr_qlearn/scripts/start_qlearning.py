#!/usr/bin/env python

import gym
import numpy as np
import time
import qlearn
from gym import wrappers, spaces
import rospy
import rospkg
import bebop_bdr


if __name__ == '__main__':
    rospy.init_node('bebop_bdr_qlearn', anonymous=True, log_level=rospy.WARN)

    # Create the Gym environment
    env = gym.make('BebopBdr-v0')
    rospy.logwarn("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('bebop_bdr_qlearn')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    rospy.logwarn("Monitor Wrapper started")

    last_time_steps = np.ndarray(0)

    # Loads parameters from the ROS param server
    Alpha = rospy.get_param("/bebop/alpha")
    Epsilon = rospy.get_param("/bebop/epsilon")
    Gamma = rospy.get_param("/bebop/gamma")
    epsilon_discount = rospy.get_param("/bebop/epsilon_discount")
    nepisodes = rospy.get_param("/bebop/nepisodes")
    nsteps = rospy.get_param("/bebop/nsteps")
    running_step = rospy.get_param("/bebop/running_step")

    # Initializes the algorithm that we are going to use for learning
    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           alpha=Alpha, gamma=Gamma, epsilon=Epsilon)
    initial_epsilon = qlearn.epsilon

    start_time = time.time()
    highest_reward = 0

	for x in range(nepisodes):
        rospy.logwarn("############### START EPISODE " + str(x) + " ###############")

        cumulated_reward = 0
        done = False

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # Initialize the environment and get first state of the robot
        observation = env.reset()
        state = ''.join(map(str, observation))

        for i in range(nsteps):
            rospy.logwarn("############### START STEP " + str(i) + " ###############")
    
            action = qlearn.chooseAction(state)
            rospy.logwarn("Next action is:%d", action)

            # Execute the action in the environment and get feedback
            observation, reward, done, info = env.step(action)

            rospy.logwarn(str(observation) + " " + str(reward))

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            nextState = ''.join(map(str, observation))

            # Make the algorithm learn based on the results
            rospy.logwarn("# state we were=>" + str(state))
            rospy.logwarn("# action that we took=>" + str(action))
            rospy.logwarn("# reward that action gave=>" + str(reward))
            rospy.logwarn("# episode cumulated_reward=>" + str(cumulated_reward))
            rospy.logwarn("# State in which we will start next step=>" + str(nextState))

            qlearn.learn(state, action, reward, nextState)

            if not (done):
                rospy.logwarn("NOT DONE")
                state = nextState
            else:
                rospy.logwarn("DONE")
                last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                break

            rospy.logwarn("############### END STEP " + str(i) + " ###############")
            
			#raw_input("Next Step...PRESS KEY")
            # rospy.sleep(2.0)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        rospy.logerr(("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
            round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s)))


    env.close()

