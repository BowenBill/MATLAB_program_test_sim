%% 该程序通过Vrep-MATLAB联合仿真来得到干涉仪测量距离d
% 用时: 102s
clc;
clear;
close all;
%% 准备
[vrep, clientID] = get_vrep_connect();
% 获取句柄
[res, Handle_Gripper] = get_Handle_Gripper(vrep, clientID);
 vrchk(vrep, res,true);
[res, Handle_Interferometric] = vrep.simxGetObjectHandle(clientID,'/Dummy_interferometric',vrep.simx_opmode_blocking);
 vrchk(vrep, res,true);
[res, Handle_Mirror] = vrep.simxGetObjectHandle(clientID,'/Dummy_mirror_origion',vrep.simx_opmode_blocking);
 vrchk(vrep, res,true);
% 设置0位姿
res = set_joint_positions(vrep, clientID, Handle_Gripper.joints, zeros(7,1));
%% 开始仿真
vrep_Start(vrep, clientID);
%% 运动+数据记录
% 外参标定数据
tic
data_list_extrinsic_parameter = get_extrinsic_parameter(vrep,clientID,Handle_Mirror, Handle_Interferometric,Handle_Gripper);
pause(0.1);
% 内参标定数据
data_list_intrinsic_parameter = get_intrinsic_parameter(vrep,clientID,Handle_Mirror, Handle_Interferometric,Handle_Gripper);
pause(2);
toc
%% 结束仿真
vrep_Finish(vrep, clientID);
% 回到0位姿
[vrep, clientID] = get_vrep_connect();
ret = set_joint_positions(vrep, clientID, Handle_Gripper.joints, zeros(7,1));
vrep_Finish(vrep, clientID); 
%% 处理数据
data_intrinsic = data_list_intrinsic_parameter;
data_extrinsic = data_list_extrinsic_parameter;
data_intrinsic(1:4,:) = data_intrinsic(1:4,:) * 1000;
data_extrinsic(1:4,:) = data_extrinsic(1:4,:) * 1000;
data_intrinsic = data_intrinsic';
data_extrinsic = data_extrinsic';
data = [data_intrinsic;data_extrinsic];
% 删除多余变量
clear data_intrinsic data_extrinsic data_list_intrinsic_parameter data_list_extrinsic_parameter;
clear clienyID Handle_Mirror Handle_Gripper Handle_Interferometric ret res vrep;