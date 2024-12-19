%% 该代码用于标定外参，得到90组数据
function data_list = get_extrinsic_parameter(vrep,clientID,Handle_Mirror, Handle_Interferometric,Handle_Gripper)

% 得到电机关节运动距离（角度）与d的关系式
% 先在工作空间内选择15个点
Point_list_15 = zeros(7,15); % 每一列对应7个电机所移动/旋转的距离/角度
Point_list_15(1,:) = [0 50 -50 0 0 0 0 -50 50 50 -50 -50 50 50 -50]/1000;
Point_list_15(2,:) = [0 0 0 50 -50 0 0 -50 -50 50 50 -50 -50 50 50]/1000;
Point_list_15(3,:) = [0 0 0 0 0 50 -50 50 50 50 50 -50 -50 -50 -50]/1000;
Point_list_15(4,:) = Point_list_15(3,:);
% Point_list_15(4,:) = [0 0 0 0 0 -50 50 -50 -50 -50 -50 50 50 50 50]/1000;
 % L5，L6[-12 12deg]; L7[-45 45deg]
angle_list = zeros(3,6);
angle_list(1,:)= [-12 12 0 0 0 0]*pi/180;
angle_list(2,:)= [0 0 -12 12 0 0]*pi/180;
angle_list(3,:)= [0 0 0 0 -45 45]*pi/180;
% d_list中距离单位是mm    角度为deg
% d_list中每一列代表的是距离+关节角度
d_list = zeros(8,90);      % 共90组数据
num_list = 1;
for i = 1 : 1: 15
    % 选定位置
    thetalist = zeros(7,1);
    thetalist(1:4) = Point_list_15(1:4,i);
    % 选定角度
    for j = 1:2:5
        % 变量初始化
        new_data = zeros(8,1);
        % 第一次 选定角度，移动，测量d，记录数据
        thetalist(5:7) = angle_list(1:3,j);
        % 第一次 移动 + 测量d + 记录数据
        res = set_joint_positions(vrep, clientID, Handle_Gripper.joints, thetalist);
        pause(0.1);
        d = get_distance_d(vrep,clientID,Handle_Mirror,Handle_Interferometric,Handle_Gripper.base);
        new_data(1:7) = thetalist;
        new_data(8) = d;
        d_list(:, num_list) = new_data;
        num_list = num_list +1;
        pause(0.1);

        % 第二次 选定角度，移动，测量d，记录数据
        % 初始化
        new_data = zeros(8,1);
        thetalist(5:7) = angle_list(1:3,j+1);
        % 第二次 移动 + 测量d + 记录数据
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


