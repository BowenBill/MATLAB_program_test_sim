%% 机器人内参标定
function data_list = get_intrinsic_parameter(vrep,clientID,Handle_Mirror, Handle_Interferometric,Handle_Gripper)
% 选取8个姿态
orientation = [0 0  0  0  6 6  6 6;...
               0 0  6  6  0 0  6 6;...
               0 45 0  45 0 45 0 45]*pi/180;
% 选取3个位置
position = [50 -50 0   0   0   0;...
            0   0  50 -50  0   0;...
            0   0  0   0   50 -50;...
            0   0  0   0   50 -50]/1000;
d_list = zeros(8,48);      % 共48组数据
num_list = 1;
for i = 1 : 1: 6  % position
    % 选定位置
    thetalist = zeros(7,1);
    thetalist(1:4) = position(1:4,i);
    % 选定角度
    for j = 1:1:8  % orientation
        % 变量初始化
        new_data = zeros(8,1);
        % 第一次 选定角度，移动，测量d，记录数据
        thetalist(5:7) = orientation(1:3,j);
        % 第一次 移动 + 测量d + 记录数据
        res = set_joint_positions(vrep, clientID, Handle_Gripper.joints, thetalist);
        pause(0.1);
        d = get_distance_d(vrep,clientID,Handle_Mirror,Handle_Interferometric,Handle_Gripper.base);
        new_data(1:7) = thetalist;
        new_data(8) = d;
        d_list(:, num_list) = new_data;
        num_list = num_list +1;
        pause(0.1);
    end
end
data_list = d_list;
end