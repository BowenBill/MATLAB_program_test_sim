function [res, Handle_Gripper] = get_Handle_Gripper(vrep, clientID)
%% 获取UR5机械臂所需句柄
% 关节
joint = zeros(7,1);
[res,joint(1,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint1',vrep.simx_opmode_blocking);%simx_opmode_blocking:可以理解为这个操作执行一次，但是没有执行完就要等，所以速度慢，但是会保证通信完成
[res,joint(2,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint2',vrep.simx_opmode_blocking);
[res,joint(3,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint3',vrep.simx_opmode_blocking);%simx_opmode_blocking:可以理解为这个操作执行一次，但是没有执行完就要等，所以速度慢，但是会保证通信完成
[res,joint(4,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint4',vrep.simx_opmode_blocking);
[res,joint(5,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint5',vrep.simx_opmode_blocking);%simx_opmode_blocking:可以理解为这个操作执行一次，但是没有执行完就要等，所以速度慢，但是会保证通信完成
[res,joint(6,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint6',vrep.simx_opmode_blocking);
[res,joint(7,1)] = vrep.simxGetObjectHandle(clientID,'/gripper_joint7',vrep.simx_opmode_blocking);
% 点
[res,base] = vrep.simxGetObjectHandle(clientID,'/Dummy_base',vrep.simx_opmode_blocking);
[res,effector] = vrep.simxGetObjectHandle(clientID,'/Dummy_end_effector',vrep.simx_opmode_blocking);
% [res,measure] = vrep.simxGetObjectHandle(clientID,'/Dummy_measure',vrep.simx_opmode_blocking);
% 输出
Handle_Gripper.joints = joint;
Handle_Gripper.base = base;
Handle_Gripper.effector = effector;
% Handle_Gripper.measure = measure;
end