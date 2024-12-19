%% 初始值的计算和验算
clc
clear
% 带入的是Vrep仿真的位姿：变量名_test1
% 带入的是论文中写的公式：变量名_test2
% 都要带入：变量名_test1
% 计算误差：ERROR_变量名_test
%% 对象干涉仪: Dummy_interferometric 
% 变量: x_wf y_wf z_wf beta_wf theta_wf
% x:-0.325 y:+0.025 z:+0.075-0.1 (m)
% a:+20    b:+80    g:0       (deg)
pos_w_T_f_test1 = [-0.325 0.025 (+0.075-0.1)] * 1000;
beta_wf_test2  = +20 * pi/180;
theta_wf_test2 = +80 * pi/180;
% 文章中是Rx和Rz,应该是Rx和Ry
w_T_f_test1 = MobXYZ(pos_w_T_f_test1) * ROT('X', beta_wf_test2) * ROT('Y', theta_wf_test2);
ori_w_T_f_test1 = [20 80 0]*pi/180;
w_T_f_test2 = MobXYZ(pos_w_T_f_test1) * EulerZYX(ori_w_T_f_test1);
ERROR_w_T_f_test = w_T_f_test1 - w_T_f_test2;
% disp(ERROR_w_T_f_test);
% 误差为0,该参数正确
%% 对象镜面: Dummy_mirror_origion
% 变量: z_et beta_et theta_et
% 镜面相对执行器坐标系e_T_t Dummy_end_effector
% Rx = +20 Ry = +10 (deg)
% z_et = 0.020
% beta_et_real = 20 * pi/180;
% theta_et_real = 10 * pi/180;
% z_et_real = 0.020 * 1000;
% 验证公式: 
% w_T_t = w_T_e * e_T_t
% 根据Vrep中算出的的e_T_t
% w_T_e_test1 
pos_w_T_e_test1 = [-0.14701 +0.00191 (+0.0732-0.1)] * 1000;
ori_w_T_e_test1 = [-172 -61 +176] * pi/180;
w_T_e_test1 = MobXYZ(pos_w_T_e_test1)* EulerZYX(ori_w_T_e_test1);
% w_T_t_test1
pos_w_T_t_test1 = [-0.1645 +0.00326 (+0.0636-0.1)] * 1000;
ori_w_T_t_test1 = [140.384 -61.882 135.40] *pi/180;
w_T_t_test1 = MobXYZ(pos_w_T_t_test1) * EulerZYX(ori_w_T_t_test1);
% e_T_t_test1 = inv(w_T_e) * w_T_t;
e_T_t_test1 = w_T_e_test1\w_T_t_test1;
% e_T_t_test2
beta_et_test2 = 20 * pi/180;
theta_et_test2 = 10 * pi/180;
z_et_test2 = 0.020 * 1000;
e_T_t_test2 = MOB('Z',20) * ROT('X', beta_et_test2) * ROT('Y', theta_et_test2);
% w_T_t_test2
w_T_t_test2 = w_T_e_test1 * e_T_t_test2;
% w_T_t_test1 & w_T_t_test2 误差
ERROR_e_T_t_test = e_T_t_test1 - e_T_t_test2;
ERROR_w_T_t_test12 = w_T_t_test1 - w_T_t_test2;
% w_T_t = w_T_e * e_T_t
% 误差源于Vrep小数点位数不够，模型正确
%% 对象7Dof_gripper
% 坐标系中的Z轴要和模型中坐标系中的Z轴重合
% a = [0 15 17.82 0 159.08 0 0 ];
% alpha = [-90 90 90 -160 -98.65 90 90] * pi/180;
% d = [0 0 -49.27 -31.94 1.23 0 0];
% theta = [-90 -10 90 -30.38 -4.82 90 0] * pi/180;
% 变量名_estimate 为估计值，是sw模型上的值
a_estimate = [0 15 17.82 0 159.08 0 0 ];
alpha_estimate = [-90 90 90 -160 -98.65 90 90] * pi/180;
d_estimate = [0 0 -49.27 -31.94 1.23 0 0];
theta_estimate = [-90 -10 90 -30.38 -4.82 90 0] * pi/180;
w_T_e_estimate_test2 = eye(4);
for i = 1 :  7
    w_T_e_estimate_test2 = w_T_e_estimate_test2 * ROT('X',alpha_estimate(i)) * MOB('X',a_estimate(i))...
        * ROT('Z',theta_estimate(i))* MOB('Z',d_estimate(i));
end
% Vrep 标准模型中读取的值
pos_w_T_e_estimate_test1 = [-0.13785 0 (+0.06848-0.1)] * 1000;
ori_w_T_e_estimate_test1 = [180 -60 180] * pi/180;
w_T_e_estimate_test1 = MobXYZ(pos_w_T_e_estimate_test1)* EulerZYX(ori_w_T_e_estimate_test1);
ERROR_w_T_e_estimate_test = w_T_e_estimate_test1 - w_T_e_estimate_test2;
% 误差 位置误差小于0.01mm,源于sw读取DH模型参数的进度为0.01mm
% 误差 方向误差小于0.01rad
% disp(ERROR_w_T_e_estimate_test)
init_val_list = init_val.';