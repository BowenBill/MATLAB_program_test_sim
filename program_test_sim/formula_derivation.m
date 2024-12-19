%% 公式推导
clc
%% 要推理的内容：w_T_f  e_T_t   w_T_e   w_T_t   L
syms x_wf y_wf z_wf beta_wf theta_wf ...   % 光纤相对世界坐标系w_T_f
     z_et beta_et theta_et...              % 镜面相对执行器坐标系e_T_t
     d_1 d_2 d_3 d_4 theta_5 theta_6 theta_7... % 机器人P副移动距离和R副转动角度，变量
     alpha_0 alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6...% 下面都是常量，通过带入上述变量计算
     a_0 a_1 a_2 a_3 a_4 a_5 a_6...% 单位mm
     theta_1 theta_2 theta_3 theta_4 theta_5_init theta_6_init theta_7_init...% 单位rad
     d_1_init d_2_init d_3_init d_4_init d_5 d_6 d_7...% 单位mm 以上为了计算w_T_e
     temp_sym real
% x_wf y_wf z_wf是光纤在世界坐标系的位置，beta_wf theta_wf是R_xw和R_zw
% 文章中是Rx和Rz,应该是Rx和Ry
w_T_f = MOB('X',x_wf) * MOB('Y',y_wf) * MOB('Z',z_wf) * ROT('X', beta_wf) * ROT('Y', theta_wf);

% z_et beta_et theta_et是关于e_T_t的参数
e_T_t = MOB('Z',z_et) * ROT('X', beta_et) * ROT('Y', theta_et);

% 关于微装配机器人的DH参数
% i-1_T_i = Rx(alpha_i-1) * Dx(a_i-1) * Rz(theta_i) * Dz(d_i)
a = [a_0 a_1 a_2 a_3 a_4 a_5 a_6];
alpha = [alpha_0 alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6];
d = [d_1_init+d_1 d_2_init+d_2 d_3_init+d_3 d_4_init+d_4 d_5 d_6 d_7];
theta = [theta_1 theta_2 theta_3 theta_4 theta_5_init+theta_5 theta_6_init+theta_6 theta_7_init+theta_7];
w_T_e = eye(4);
for i = 1 :  7
    w_T_e = w_T_e * ROT('X',alpha(i)) * MOB('X',a(i))...
        * ROT('Z',theta(i))* MOB('Z',d(i));
end
%% 构建计算L的等式
% 参数a_wt b_wt c_wt d_wt x_wt y_wt z_wt赋值
w_T_t = w_T_e * e_T_t;
a_wt = w_T_t(1,3); b_wt = w_T_t(2,3); c_wt = w_T_t(3,3);
x_wt = w_T_t(1,4); y_wt = w_T_t(2,4); z_wt = w_T_t(3,4);
d_wt = -a_wt * x_wt - b_wt * y_wt - c_wt * z_wt;
% a_wf = w_T_f(1,3); b_wf = w_T_f(2,3); c_wf = w_T_f(3,3);
a_wf = w_T_f(1,3);b_wf = w_T_f(2,3);c_wf = w_T_f(3,3);
% 方程 将该表达式放入Word文档后查找所有变量，发现所有变量都存在的
L_expr = -(a_wt * x_wf + b_wt * y_wf +c_wt*z_wf +d_wt)/...
    (a_wt * a_wf + b_wt * b_wf +c_wt * c_wf);
%% 数据带入构成雅可比矩阵求数值解
% 将仿真得到的 d_1 d_2 d_3 d_4 theta_5 theta_6 theta_7 L_expr 带入，得到Jacobians_equ
% 初始化
Jacobians_equ = zeros(36,36);
% 变量列表，用于记录变量
variable_list = [x_wf y_wf z_wf beta_wf theta_wf z_et beta_et theta_et ...                  % 1 - 8
                alpha_0 alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6 ...                 % 9 - 15
                a_0 a_1 a_2 a_3 a_4 a_5 a_6 ...                                             % 16 - 22
                theta_1 theta_2 theta_3 theta_4 theta_5_init theta_6_init theta_7_init ...  % 23 - 29
                d_1_init d_2_init d_3_init d_4_init d_5 d_6 d_7];                           % 30 - 36
% 使用for循环+diff函数构建多组方程，得到Jacobians_equ 公式：res = pinv(Jacobians_equ)*L
% 直接获取L数组
L = data(:,8);
% 开始迭代的初始值，后面要改
iterations = 1000;                      % 暂定的迭代次数
variable_val = zeros(36, iterations+1); % 用于记录每次迭代的值
init_val = get_init_val();              % 迭代的初始值
variable_val(:,1) = init_val;           % 对init_val进行赋值
iterations_count = 1;                   % 当前第iterations_count次迭代
% 用for循环构建多组方程，得到Jacobians_equ，通过迭代计算，时间是个问题，循环一遍要800秒左右，
% for i = 1:size(data, 1)
tic % 开始计时
for i = 1:36
    % 从data中提取第i组数据
    d_1_val = data(i, 1); d_2_val = data(i, 2); d_3_val = data(i, 3); d_4_val = data(i, 4);
    theta_5_val = data(i, 5); theta_6_val = data(i, 6); theta_7_val = data(i, 7);
    L_val = data(i, 8);
    % 将数据代入L的方程
    L_expr_sub = subs(L_expr, {d_1, d_2, d_3, d_4, theta_5, theta_6, theta_7, L_expr},...
        {d_1_val, d_2_val, d_3_val, d_4_val, theta_5_val, theta_6_val, theta_7_val, L_val});
    % 对for j循环所用的参数初始化
    temp_var = 0;
    temp_diff = 0;
    variable_list_temp = variable_list;
    % 对L_expr_sub求偏导，共36个变量，用variable_list记录，用for循环求解
    for j = 1 : 36 
        % for j 循环用于求出当前的雅可比矩阵
        % 先带入除偏导项的数值(通过删去该矩阵这一列来实现)，后求偏导，再带入偏导项的数值，得到一个常数
        temp_j = j;                                        % 偏导项为第 j 项
        temp_sym = variable_list(j);                       % 偏导项
        temp_var = variable_val(iterations_count,j);       % 偏导项数值
        % 将第j个变量和第j个变量所对应的数值删去，得到1个变量矩阵和1个数值矩阵
        % 删去第j个变量的变量矩阵temp_syms
        temp_syms_list = variable_list;
        temp_syms_list(:,j) = [];
        % 删去第j个变量的数值矩阵temp_val_list
        temp_val_list = variable_val(:,iterations_count)'; 
        temp_val_list(:,j) = [];
        L_expr_sub_j = subs(L_expr_sub,temp_syms_list,temp_val_list); % 第一次带入数据，简化计算
        temp_diff = diff(L_expr_sub_j,temp_sym);         % 求偏导
        temp_diff = subs(temp_diff,temp_sym,temp_var);   % 第二次带入数据，得到雅可比矩阵
        temp_diff = double(temp_diff);                   % 转换为双精度类型
        Jacobians_equ(i,j) = temp_diff;                  % 用Jacobians_equ(i,j)接受结果  
        temp_diff = 0;                                   % 将temp_diff初始化
    end
    % 将方程添加到方程组中 这步暂时没必要
end
toc % 计时结束
% L_36 = L(1:36);
% sol = pinv(Jacobians_equ)*L_36; % 方程的解