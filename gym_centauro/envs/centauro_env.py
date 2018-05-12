import gym, math
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np
from gym_centauro.envs.vrep_plugin import vrep

lua_script_name = 'world_visual'
observation_channel = 44 + 3 + 3 #map_pixel*map_pixel + 25  # 60 x 60 + 8  2*3 + 2 + 2
action_channel = 24

n_step_num = 300

class CentauroEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(observation_channel,))
        self.action_space = spaces.Box(-1, 1, shape = (action_channel,))
        self.max_h_v = 0

    def seed(self, seed=20000):
        print(seed)
        self.port_num = seed 
        self.connect_vrep()

    def convert_state(self, robot_state, target_state, rot_vel):
        state = np.asarray(robot_state[:20])
        state = np.append(state, robot_state[24:-1])
        # state = np.append(state, robot_state[-4:-1])
        # state = np.append(state, rot_vel)
        # observation = self.terrain_map.flatten()
        # target_info = state[3:5]
        # robot_info = state[-3:-1]

        # print(target_info, robot_info)

        # state = np.append(observation, target_info)
        state = np.append(state, target_state[:3])
        state = np.append(state, robot_state[-1])
        return state

    def reset(self):
        # print('reset')
        _, _, state, _, _ = self.call_sim_function(lua_script_name, 'reset')        
        self.dist_pre = state[-21]
        self.action_pre = np.zeros(24)
        self.nsteps = 0

        return state[:-21]

    def step(self, action):
        self.nsteps += 1

        if isinstance(action, np.int32) or isinstance(action, int) or isinstance(action, np.int64):
            if action == -1:
                action = [0,0,0,0,0]
            else:          
                action = action_list[action]

        # print(action)
        a = np.zeros(24)
        a[-len(action):] = action
        _, _, robot_state, _, found_pose = self.call_sim_function(lua_script_name, 'step', a)
        reward, is_finish, info = self.compute_reward(robot_state, action, found_pose)
        
        # target_state, rot_vel, reward_long, is_finish, info = self.check_state(a, found_pose)
        # time.sleep(0.2)

        # if info == 'goal':
        #     a = np.zeros(24)
        #     _, _, robot_state, _, found_pose = self.call_sim_function(lua_script_name, 'step', a)
        #     time.sleep(1)
        #     target_state, rot_vel, reward_long, is_finish, info = self.check_state(a, found_pose)

        # state_ = self.convert_state(robot_state[:-16], target_state, rot_vel)
        return robot_state[:-21], reward, is_finish, {"info" : info}


    def compute_reward(self, robot_state, action, found_pose):
        # state_diff = np.sum(abs(robot_state[24] - self.state_pre[24]))/math.pi
        # _, _, g_pose, _, _ = self.call_sim_function(lua_script_name, 'get_robot_position')
        # _, _, acc, _, _ = self.call_sim_function('Accelerometer', 'get_acc_data')
        # _, _, floor_dist, _, _ = self.call_sim_function(lua_script_name, 'get_minimum_floor_dist')
        # sum_acc = np.sum(acc)

        # _, _, joint_force, _, _ = self.call_sim_function(lua_script_name, 'get_joint_force')
        # _, _, l_v, _, _ = self.call_sim_function(lua_script_name, 'get_robot_velocity')
        
        l_v = robot_state[-4:-1]
        h_v = np.sqrt(l_v[0]*l_v[0] + l_v[1]*l_v[1])
        v_v = abs(l_v[2])

        projection = robot_state[-1]
        # if projection < 0:
        #     projection = 0

        # print(projection)

        if h_v > self.max_h_v:
            self.max_h_v = h_v

        # print(self.max_h_v, h_v, v_v)

        joint_force = np.asarray(robot_state[-20:-4])
        # print(joint_force.max(), joint_force)
        dist = robot_state[-21]
        target_reward = -(dist - self.dist_pre)/0.02

        info = 'unfinish'
        is_finish = False
        alive = 1
        terminal_r = 0
        # print(dist, g_pose[-1])
        # action = np.asarray(action)
        action_cost = np.abs(action - self.action_pre).mean() * -0.5

        self.dist_pre = dist
        self.action_pre = action

        if found_pose == bytearray(b"b"):       # when wheel leave the ground
            # is_finish = True
            alive = 1
            info = 'bad_pose'
            
        if found_pose == bytearray(b"a"):       # when collision or no pose can be found
            is_finish = True
            terminal_r = -10
            info = 'fall'
        #     self.goal_cound += 1
        #     if self.goal_cound > 10:
        #         is_finish = True
        # else:
        #     self.goal_cound = 0

        if found_pose == bytearray(b"c"):       # when collision or no pose can be found
            is_finish = True
            terminal_r = -10
            info = 'crash'

        # # print('dist', dist, target_state)
        if dist < 0.1 and info == 'unfinish': # and diff_l < 0.02: g_pose[-1] < 0.1 
            is_finish = True
            target_reward = 1
            # terminal_r = 100
            info = 'goal_r'
        #     reward = +1
        #     # reward_long = REWARD_GOAL/5
        #     # self.goal_cound += 1
        #     # if self.goal_cound > 10:
        #     #     info = 'goal'
        #     #     reward_long = REWARD_GOAL*10
        #     #     is_finish = True
        # # else:
        # #     self.goal_cound = 0


        if found_pose == bytearray(b"o"): 
        # if abs(g_pose[0]) > 0.1 or g_pose[1] < -0.1: # or (robot_state[2] < 0 and abs(robot_state[1]) > 0.1): # out of boundary
        # if abs(robot_state[1]) > 0.15 or robot_state[2] < -0.6:
            is_finish = True
            info = 'out'

        if self.nsteps > n_step_num:
            is_finish = True
            info = 'no_step'

        # print('rewards', self.nsteps, reward, target_reward, np.sum(joint_force)*0.02)

        if target_reward < -1:
            target_reward = -1
        if target_reward > 1:
            target_reward = 1

        # reward = target_reward - np.sum(joint_force)*0.02 #action_cost
        # final_reward = alive * 0.05 + h_v * 0.2 * projection*4 + terminal_r #+ alive
        # final_reward = terminal_r - joint_force.max()*0.1
        reward_alive = alive + terminal_r
        reward_target = target_reward + terminal_r
        return [reward_alive, reward_target], is_finish, info


    def render(self, mode='human', close=False):
        return 0


    def compute_dist(self, x, y):
        return math.sqrt(x*x + y*y)

    #################### server ###############################
    def restart_simulator(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        while True:
            vrep.simxGetIntegerSignal(self.clientID,lua_script_name,vrep.simx_opmode_blocking)
            e = vrep.simxGetInMessageInfo(self.clientID,vrep.simx_headeroffset_server_state)
            not_stopped = e[1] & 1

            if not not_stopped:
                break
        e = vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)

    def connect_vrep(self):
        clientID = vrep.simxStart('127.0.0.1', self.port_num, True, True, 5000, 1)
        if clientID != -1:
            print ('Connected to remote API server with port: ', self.port_num)
        else:
            print ('Failed connecting to remote API server with port: ', self.port_num)
        self.clientID = clientID
        self.restart_simulator()

    def disconnect_vrep(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)
        time.sleep(1)
        vrep.simxFinish(self.clientID)
        print ('Program ended')

    def call_sim_function(self, object_name, function_name, input_floats=[]):
        inputInts = []
        inputFloats = input_floats
        inputStrings = []
        inputBuffer = bytearray()
        res,retInts,retFloats,retStrings,retBuffer = vrep.simxCallScriptFunction(self.clientID, object_name,vrep.sim_scripttype_childscript,
                    function_name, inputInts, inputFloats, inputStrings,inputBuffer, vrep.simx_opmode_blocking)

        # print 'function call: ', self.clientID
        return res, retInts, retFloats, retStrings, retBuffer
