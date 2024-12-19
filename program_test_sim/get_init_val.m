%% 模型的初值
% 在formula_derivation中
% 变量名_init      为初值
% 变量名_real 为真值
function init_val = get_init_val()
%% 机器人DH参数初值
% a = [0 15 17.82 0 159.08 0 0 ];
% alpha = [-90 90 90 -160 -98.65 90 90] * pi/180;
% d = [0 0 -49.27 -31.94 1.23 0 0];
% theta = [-90 -10 90 -30.38 -4.82 90 0] * pi/180;
a_0_init = 0; a_1_init = 15;     a_2_init = 17.82; 
a_3_init = 0; a_4_init = 159.08; a_5_init = 0; a_6_init = 0;

alpha_0_init = -90 * pi/180; alpha_1_init = 90 * pi/180;     alpha_2_init = 90 * pi/180;
alpha_3_init = -160 * pi/180;  alpha_4_init = -98.65 * pi/180;  alpha_5_init = 90 * pi/180;
alpha_6_init = 90 * pi/180;

d_1_init_init = 0; d_2_init_init = 0; d_3_init_init = -49.27; d_4_init_init = -31.94;
d_5_init = 1.23;   d_6_init = 0;      d_7_init = 0;

theta_1_init = -90* pi/180;     theta_2_init = -10* pi/180;         theta_3_init = 90* pi/180; 
theta_4_init = -30.38* pi/180;   theta_5_init_init = -4.82* pi/180;   theta_6_init_init = 90* pi/180;
theta_7_init_init = 0* pi/180;
%% 对象干涉仪: Dummy_interferometric 
% 变量: x_wf y_wf z_wf beta_wf theta_wf
% x:-0.325 y:+0.025 z:+0.075-0.1 (m)
% a:+20    b:+80    g:0       (deg)
% x_wf_real = -0.325 * 1000;
% y_wf_real = +0.025 * 1000;
% z_wf_real = (0.075-0.1) * 1000;
% beta_wf_real  = +20 * pi/180;
% theta_wf_real = +80 * pi/180;
% 初值: 适当人为增加误差
x_wf_init = -0.325 * 1000;
y_wf_init = +0.025 * 1000;
z_wf_init = (0.075-0.1) * 1000;
beta_wf_init  = +20 * pi/180;
theta_wf_init = +80 * pi/180;
%% 对象镜面: Dummy_mirror_origion
% 变量: z_et beta_et theta_et
% 镜面相对执行器坐标系e_T_t
% Rx = +20 Ry = +10 (deg)
% z_et = -0.020
% beta_et_real = 20 * pi/180;
% theta_et_real = 10 * pi/180;
% z_et_real = 0.020 * 1000;
beta_et_init = 20 * pi/180;
theta_et_init = 10 * pi/180;
z_et_init = 0.020 * 1000;
%% 初值的list
init_val = [x_wf_init y_wf_init z_wf_init beta_wf_init theta_wf_init z_et_init beta_et_init theta_et_init ...
                alpha_0_init alpha_1_init alpha_2_init alpha_3_init alpha_4_init alpha_5_init alpha_6_init ...
                a_0_init a_1_init a_2_init a_3_init a_4_init a_5_init a_6_init ...
                theta_1_init theta_2_init theta_3_init theta_4_init theta_5_init_init theta_6_init_init theta_7_init_init ...
                d_1_init_init d_2_init_init d_3_init_init d_4_init_init d_5_init d_6_init d_7_init];
end
%% 机器人DH参数真值
% 根据Vrep模型得出的真值







%% 误差分析




