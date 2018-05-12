local lfs = require("lfs")
full_path = lfs.currentdir()
path = ";"..string.sub(full_path, 1, -5).."gym_centauro/gym_centauro/envs/?.lua"
print(path)
package.path=package.path .. path

require("common_functions")
require("get_set")

_obs_mode = 'random' --'near'
_bound_x = 1.2
_bound_y = 1.2

_target_dist = 2

function start()
    _floor_hd = simGetObjectHandle('floor')
    _collection_wheel_hd = simGetCollectionHandle('wheels')
    _wheel_hds = simGetCollectionObjects(_collection_wheel_hd)

    _base_hd = simGetObjectHandle('world_visual')
    _fake_robot_hd = simGetObjectHandle('fake_robot')
    _robot_hd = simGetObjectHandle('centauro')
    _target_hd = simGetObjectHandle('target')
    _joint_hds = get_joint_hds(24)
    _mode = simGetModelProperty(_robot_hd)

    _start_pos = simGetObjectPosition(_base_hd, -1)
    _start_ori = simGetObjectOrientation(_base_hd,-1)
    _start_joint_values = get_joint_values(_joint_hds)

    _start_t_pos = simGetObjectPosition(_target_hd, -1)
    _start_t_pos[3] = _start_t_pos[3] + 0.35
    _start_t_ori = simGetObjectOrientation(_target_hd,-1)

    _collection_hd = simGetCollectionHandle('obstacle_all')
    _collection_robot_hd = simGetCollectionHandle('centauro_mesh')

    _obstacles_hds = simGetCollectionObjects(_collection_hd)

    _obstacle_dynamic_collection = simGetCollectionHandle('obstacle_dynamic')
    _obstacle_dynamic_hds = simGetCollectionObjects(_obstacle_dynamic_collection)

    _obs_hds = check_avaiable_obstacle_hds()

    _start_xyz = {}
    _start_ori = {}
    local objects=simGetObjectsInTree(_base_hd,sim_handle_all,0)
    for i=1,#objects,1 do
        _start_xyz[i] = simGetObjectPosition(objects[i], _base_hd)
        _start_ori[i] = simGetObjectOrientation(objects[i], _base_hd)    
    end

    _tra_robot_g_pose = {}
end

function check_avaiable_obstacle_hds()
    hds = {}
    for i=1, #_obstacle_dynamic_hds, 1 do 
        local obs_pos_global = simGetObjectPosition(_obstacle_dynamic_hds[i], -1)
        
        local x = math.abs(obs_pos_global[1])
        local y = math.abs(obs_pos_global[2])

        if x < 2.5 and y < 2.5 then   
            hds[#hds + 1] = _obstacle_dynamic_hds[i]
        end
    end
    return hds
end

function get_minimum_floor_dist(inInts,inFloats,inStrings,inBuffer)
    local threshold = 0.1
    local res, data = simCheckDistance(_collection_robot_hd, _floor_hd, threshold)
    if data == nil then 
        dist = threshold
    else 
        dist = data[7]
    end
    -- print(dist)    
    return {}, {threshold - dist}, {}, ''
end

function reset(inInts,inFloats,inStrings,inBuffer)
    _tra_robot_g_pose = {}

    target_pos = {}
    target_pos[1] = 0
    target_pos[2] = math.random()*_target_dist
    target_pos[3] = _start_t_pos[3]
    simSetObjectPosition(_target_hd, -1, target_pos)
    ori = simGetObjectOrientation(_target_hd, -1)
    ori[3] = math.random() * math.pi*2
    simSetObjectOrientation(_target_hd, -1, ori)

    robot_pos = {}
    robot_pos[1] = 0
    robot_pos[2] = 0
    robot_pos[3] = _start_pos[3]
    simSetObjectPosition(_base_hd, -1, robot_pos)

    reset_joint(_joint_hds)

    local objects=simGetObjectsInTree(_base_hd,sim_handle_all,0)
    for i=1,#objects,1 do
        simSetObjectPosition(objects[i], _base_hd, _start_xyz[i])
        simSetObjectOrientation(objects[i], _base_hd, _start_ori[i])        
        simResetDynamicObject(objects[i])
    end

    local init_wheel_angle = (math.random()-0.5)*2 * math.pi 
    for i=17, 20, 1 do
        simSetJointPosition(_joint_hds[i], init_wheel_angle)
        simSetJointTargetPosition(_joint_hds[i], init_wheel_angle)
    end

    pos = simGetObjectPosition(_base_hd, -1)
    ori = simGetObjectOrientation(_base_hd, -1)
    ori[3] = math.random() * math.pi*2
    simSetObjectOrientation(_base_hd, -1, ori)

    _, state, _, _ = get_robot_state()
    local robot_loc  = simGetObjectPosition(_robot_hd, -1)
    local robot_ori  = simGetObjectOrientation(_robot_hd, -1)
    local robot_state = {robot_loc, robot_ori}
    -- print(robot_loc[1], robot_loc[2], robot_loc[3])
    _tra_robot_g_pose[#_tra_robot_g_pose+1] = robot_state
        
    -- print(joint_pose[#joint_pose])
    return {}, state, {}, ''
end

function step(inInts,actions,inStrings,inBuffer)
    -- global_counter = global_counter + 1
    -- if global_counter > 500 then 
    --     global_counter = 0
    --     target_pos[1] = (math.random()-0.5)*2*_target_dist
    --     target_pos[2] = (math.random()-0.5)*2*_target_dist
    --     target_pos[3] = _start_t_pos[3]
    --     simSetObjectPosition(_target_hd, -1, target_pos)        
    -- end
    local body_min_p = -45*math.pi/180
    local body_max_p = math.pi 

    local reach_limit = false

    local robot_joints = get_joint_values(_joint_hds)
    for i=1, 16, 1 do
        local hd = _joint_hds[i]
        local t_p = robot_joints[i] + actions[i]*5*math.pi/180
        if t_p < body_min_p or t_p > body_max_p then
            reach_limit = true
        end
        simSetJointTargetPosition(hd, t_p)
    end

    local a_min_p = -160*math.pi/180
    local a_max_p = 320*math.pi/180 
    for i=17, 20, 1 do
        local t_p = robot_joints[i] + actions[i]*5*math.pi/180
        if t_p < a_min_p or t_p > a_max_p then
            reach_limit = true
        end        
        simSetJointTargetPosition(_joint_hds[i], t_p)
    end         
    for i=21, 24, 1 do
        simSetJointTargetVelocity(_joint_hds[i], math.pi*actions[i]*400/180)
        --simSetJointForce(hd, 100)
    end 

    
    -- -- move robot base
    
    -- rotate_robot(_joint_hds, actions[#actions-2])
    --move_robot(_joint_hds, actions[#actions-1], actions[#actions])
    -- _, v = simGetObjectFloatParameter(_joint_hds[17], 2012)
    -- print('velocity', v)

    -- check collision
    local _, data = simCheckDistance(_collection_robot_hd, _collection_hd, 0.05)
    if data ~= nil then 
        res = 'c'
    else
        res = 't'
    end

    -- wheel leave the ground
    for i=1, #_wheel_hds, 1 do
        local result, data = simCheckDistance(_wheel_hds[i], _floor_hd, 0.05)
        if data == nil  then
            res = 'b'
            break
        end
    end
    
    if reach_limit then
        res = 'b'
    end

    -- touch the ground
    local result, data = simCheckDistance(_collection_robot_hd, _floor_hd, 0.05)
    -- print(data[1], data[2], data[3], data[4], data[5], data[6], data[7])
    if data ~= nil then 
        res = 'a'
    end 

    _, state, _, _ = get_robot_state()

    local robot_loc  = simGetObjectPosition(_robot_hd, -1)
    local robot_ori  = simGetObjectOrientation(_robot_hd, -1)
    local robot_state = {robot_loc, robot_ori}
    -- print(robot_loc[1], robot_loc[2], robot_loc[3])
    _tra_robot_g_pose[#_tra_robot_g_pose+1] = robot_state
    
    if math.abs(robot_loc[1])> 2.3 or math.abs(robot_loc[2])> 2.3 then
        res = 'o'
    end

    return {}, state, {}, res
end

function get_h_dist_to_target()
    local target_pos =simGetObjectPosition(_target_hd, -1)
    local robot_pos =simGetObjectPosition(_robot_hd, -1)

    local diff_x = math.abs(robot_pos[1] - target_pos[1])
    local diff_y = math.abs(robot_pos[2] - target_pos[2])
    local dist = math.sqrt(diff_x*diff_x + diff_y*diff_y)

    return dist
end


function get_target_state(inInts,inFloats,inStrings,inBuffer)
    local target_pos =simGetObjectPosition(_target_hd, _robot_hd)
    local robot_pose_in_target =simGetObjectPosition(_robot_hd, _target_hd)
    -- local target_g_pos =simGetObjectPosition(_target_hd, -1)
    return {}, {target_pos[1], target_pos[2], target_pos[3], robot_pose_in_target[1], robot_pose_in_target[2]}, {}, ''
end

function get_robot_velocity(inInts,inFloats,inStrings,inBuffer)
    local l_v, a_v = simGetObjectVelocity(_robot_hd)
    -- local target_g_pos =simGetObjectPosition(_target_hd, -1)
    return {}, {l_v[1], l_v[2], l_v[3]}, {}, ''
end

function get_joint_force(inInts,inFloats,inStrings,inBuffer)
    local joint_force = {}
    -- for i=1, 16, 4 do 
        -- local j_group = {}
        -- if simGetJointForce(_joint_hds[1]) == nil then 
        --     j_group = {0, 0, 0, 0}
        -- else
        --     j_group[1] = math.abs(simGetJointForce(_joint_hds[i]))
        --     j_group[2] = math.abs(simGetJointForce(_joint_hds[i+1]))
        --     j_group[3] = math.abs(simGetJointForce(_joint_hds[i+2]))
        --     j_group[4] = math.abs(simGetJointForce(_joint_hds[i+3]))
        -- end 
        
        -- local j_force_avg = (j_group[1] + j_group[2] + j_group[3] + j_group[4])/4
        -- -- print(j_force_avg, j_group[1], j_group[2], j_group[3], j_group[4])
        -- joint_force[#joint_force + 1] = math.abs(j_group[1] - j_force_avg)
        -- joint_force[#joint_force + 1] = math.abs(j_group[2] - j_force_avg)
        -- joint_force[#joint_force + 1] = math.abs(j_group[3] - j_force_avg)
        -- joint_force[#joint_force + 1] = math.abs(j_group[4] - j_force_avg)
    -- end
    for i=1, 16, 1 do 
        joint_force[#joint_force + 1] = math.abs(simGetJointForce(_joint_hds[i]))
    end    
    return {}, joint_force, {}, ''
end

function get_robot_state(inInts,inFloats,inStrings,inBuffer)  
    local joint_p_v = get_joint_pos_vel(_joint_hds)
    robot_ori = simGetObjectOrientation(_robot_hd, -1)

    local robot_state = {}
    for i=1, 20, 1 do 
        robot_state[#robot_state+1] = joint_p_v[i]
    end
    for i=25, #joint_p_v, 1 do 
        robot_state[#robot_state+1] = joint_p_v[i]
    end

    robot_state[#robot_state+1] = robot_ori[1]
    robot_state[#robot_state+1] = robot_ori[2]
    robot_state[#robot_state+1] = robot_ori[3]    

    local target_pos =simGetObjectPosition(_target_hd, _robot_hd)
    local dist_to_target = math.sqrt(target_pos[1]*target_pos[1] + target_pos[2]*target_pos[2] + target_pos[3]*target_pos[3])

    robot_state[#robot_state+1] = target_pos[1]
    robot_state[#robot_state+1] = target_pos[2]
    robot_state[#robot_state+1] = target_pos[3]

    robot_state[#robot_state+1] = dist_to_target

    -- local dist_to_target = get_h_dist_to_target()
    -- joint_pose[#joint_pose + 1] = dist_to_target

    for i=1, 16, 1 do 
        if simGetJointForce(_joint_hds[1]) == nil then 
            robot_state[#robot_state + 1] = 0
        else 
            robot_state[#robot_state + 1] = math.abs(simGetJointForce(_joint_hds[i]))
        end
    end

    local l_v, a_v = simGetObjectVelocity(_robot_hd)
    robot_state[#robot_state + 1] = l_v[1]
    robot_state[#robot_state + 1] = l_v[2]
    robot_state[#robot_state + 1] = l_v[3]


    -- compute projection of current velocity to target direction
    local robot_pos_g = simGetObjectPosition(_robot_hd, -1)
    local target_pos_g = simGetObjectPosition(_target_hd, -1)

    local t_vec = {target_pos_g[1]-robot_pos_g[1], target_pos_g[2]-robot_pos_g[2]}
    local corss = l_v[1]*t_vec[1] + l_v[2]*t_vec[2]
    local t_dist = math.sqrt(t_vec[1]*t_vec[1] + t_vec[2]*t_vec[2])
    local projection = corss/t_dist 

    robot_state[#robot_state + 1] = projection

    return {}, robot_state, {}, ''
end


function check_collision(robot_pos, robot_ori, target_pos, target_ori)
    -- return 0, 0
    simSetObjectPosition(_fake_robot_hd,-1,robot_pos)
    simSetObjectOrientation(_fake_robot_hd, -1, robot_ori)
    local res_robot = simCheckCollision(_fake_robot_hd, _collection_hd)

    simSetObjectPosition(_fake_robot_hd,-1,target_pos)
    simSetObjectOrientation(_fake_robot_hd, -1, target_ori)
    local res_target = simCheckCollision(_fake_robot_hd, _collection_hd)
    return res_robot, res_target
end

initialized = false
global_counter = 0

start()
-- _current_ep = convert_current_ep() 

-- for i=1, #_joint_hds, 1 do
-- local hd = _joint_hds[5]
-- local hd=simGetObjectHandle('j_wheel_2')
-- simSetJointTargetVelocity(hd, -100)
-- simSetJointForce(hd, 1)
-- simSwitchThread()
-- end 

-- geting camera data
-- if (sim_call_type==sim_childscriptcall_sensing) then 
--     r=simGetVisionSensorImage(depthCam)
--     resolution = simGetVisionSensorResolution(depthCam)
--     print(#r)
--     print(r[100000], r[100001], r[100002], r[600000], r[600001], r[600002])
-- end

while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
    -- do something in here
    -- simSwitchThread()
end

