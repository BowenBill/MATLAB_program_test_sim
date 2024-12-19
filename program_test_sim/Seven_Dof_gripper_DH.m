%% 对象7Dof_gripper
% a = [0 15 17.82 0 159.08 0 0 ];
% alpha = [-90 90 90 -160 -98.65 90 90] * pi/180;
% d = [0 0 -49.27 -31.94 1.23 0 0];
% theta = [-90 -10 90 -30.38 -4.82 90 0] * pi/180;
clc
clear
%% 根据末端执行器读出来的矩阵
pos_w_T_e_estimate_test1 = [-0.13785 0 (+0.06848-0.1)] * 1000;
ori_w_T_e_estimate_test1 = [-180 -60 0] * pi/180;
w_T_e_estimate_test1 = MobXYZ(pos_w_T_e_estimate_test1)* EulerZYX(ori_w_T_e_estimate_test1);
%% 根据DH参数算出来的
