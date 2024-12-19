%% Vrep中实际的DH参数
clc
clear
% 各个Link的初始坐标xyz abg
alpha_estimate = [-90 90 90 -160 -98.65 90 90] * pi/180;
a_estimate = [0 15 17.82 0 159.08 0 0 ];
theta_estimate = [-90 -10 90 -30.38 -4.82 90 0] * pi/180;
d_estimate = [0 0 -49.27 -31.94 1.23 0 0];
% DH参数公式
%   [           cos(theta),           -sin(theta),           0,             a]
%   [cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha)]
%   [sin(alpha)*sin(theta), sin(alpha)*cos(theta),  cos(alpha),  d*cos(alpha)]
%   [                    0,                     0,           0,             1]

% 在Vrep中，只有z轴的姿态是准确的，坐标(x,y,z)表示矩阵初始状态下的位置
% Base - Link1 DH参数     理论上base与link1在init的时候重合
% Base
% x:0.000; y:0.000; z:0.100
% a:0.00;  b:0.00;  g:0.00
% Link1
% x:0.002; y:0.001; z:0.1002
% a:-90.50;  b:0.30;  g:-89.80
pos_Link1 = [0.002 0.001 0.1002-0.1] * 1000;
ori_Link1 = [-90.5 0.3 -89.80] * pi/180;
w_T_Link1 = MobXYZ(pos_Link1) * EulerZYX(ori_Link1);
[alpha_1,a_1,theta_1,d_1] = get_DH_para( w_T_Link1 );
w_T_Link1_test = ROT('X',alpha_1) * MOB('X',a_1)...
          * ROT('Z',theta_1)* MOB('Z',d_1);
ERROR_w1 = w_T_Link1_test-w_T_Link1;

% Link1 - Link2 DH参数 目标: Link1_T_Link2
% Link2
% x:0.00506; y:0.00212; z:0.11568
% a:-53.023;  b:-89.622;  g:-52.549
% Link3
% x:0.00312; y:0.00068; z:0.13379
% a:99.625;  b:-0.276;  g:179.926
% STEP 1: Link2 & Link3 Vrep中的位姿矩阵
pos_Link2 = [0.00506 0.00212 0.11568-0.1] * 1000;
ori_Link2 = [-53.023 -89.622 -52.549] * pi/180;
w_T_Link2 = MobXYZ(pos_Link2) * EulerZYX(ori_Link2);
pos_Link3 = [0.00312 0.00068 0.13379-0.1] * 1000;
ori_Link3 = [99.625 -0.276 179.926] * pi/180;
w_T_Link3 = MobXYZ(pos_Link3) * EulerZYX(ori_Link3);
% STEP 2: Link2 z轴坐标向量 相对base
Link2_z = [w_T_Link2(1,3) w_T_Link2(2,3) w_T_Link2(3,3)];
% STEP 3: Link3 z轴坐标向量 相对base
Link3_z = [w_T_Link3(1,3) w_T_Link3(2,3) w_T_Link3(3,3)];
% STEP 4: Link2 x轴坐标求解
% 求解Link2_x
Link2_x = cross(Link2_z,Link3_z);
% 单位化 相对于base坐标系
Link2_x = Link2_x/(Link2_x(1)^2 + Link2_x(2)^2 + Link2_x(3)^2)^0.5;
% STEP 5: 求解Link2_y 叉乘
Link2_y = cross(Link2_z,Link2_x);
% 构造 w_T_Link2 矩阵
w_T_Link2 = get_w_T_Link(Link2_x, Link2_y, Link2_z, pos_Link2);
% STEP 6: 求解Link1_T_Link2, 并带入得到DH参数
Link1_T_Link2 = w_T_Link1\w_T_Link2;
[alpha_2,a_2,theta_2,d_2] = get_DH_para( Link1_T_Link2 );
Link1_T_Link2_test = ROT('X',alpha_2) * MOB('X',a_2)...
          * ROT('Z',theta_2)* MOB('Z',d_2);
ERROR_12 = Link1_T_Link2_test-Link1_T_Link2;

% Link2 - Link3 DH参数 目标: Link2_T_Link3
% Link3
% x:0.00312; y:0.00068; z:0.13379
% a:99.625;  b:-0.276;  g:179.926
% Link4(这个坐标轴的方向是不准的，x轴与Link4 & Link5的z轴 垂直)
% x:-0.00486; y:0.00382; z:0.15001
% a:-100.876;  b:0.732;  g:+0.256
% Link2_T_Link3_estimate = ROT('X',alpha_estimate(3)) * MOB('X',a_estimate(3))...
%         * ROT('Z',theta_estimate(3))* MOB('Z',d_estimate(3));
pos_Link4 = [-0.00486 0.00382 0.15001-0.1] * 1000;
ori_Link4 = [-100.876 0.732 +0.256] * pi/180;
w_T_Link4 = MobXYZ(pos_Link4) * EulerZYX(ori_Link4);
Link3_z = [w_T_Link3(1,3) w_T_Link3(2,3) w_T_Link3(3,3)];
Link4_z = [w_T_Link4(1,3) w_T_Link4(2,3) w_T_Link4(3,3)];
Link3_x = cross(Link3_z,Link4_z);
Link3_x = -Link3_x/(Link3_x(1)^2 + Link3_x(2)^2 + Link3_x(3)^2)^0.5;
Link3_y = cross(Link3_z,Link3_x);
w_T_Link3 = get_w_T_Link(Link3_x, Link3_y, Link3_z, pos_Link3);
Link2_T_Link3 = w_T_Link2\w_T_Link3;
[alpha_3,a_3,theta_3,d_3] = get_DH_para( Link2_T_Link3 );

% Link3 - Link4 DH参数 目标: Link3_T_Link4
% Link5
% x:-0.14307; y:0.00523; z:0.06925
% a:-0.717;  b:-29.931;  g:+177.815
% Link3_T_Link4_estimate = ROT('X',alpha_estimate(4)) * MOB('X',a_estimate(4))...
%          * ROT('Z',theta_estimate(4))* MOB('Z',d_estimate(4));
pos_Link5 = [-0.14307 0.00523 0.06925-0.1] * 1000;
ori_Link5 = [-0.717 -29.931 +177.815] * pi/180;
w_T_Link5 = MobXYZ(pos_Link5) * EulerZYX(ori_Link5);

Link4_z = [w_T_Link4(1,3) w_T_Link4(2,3) w_T_Link4(3,3)];
Link5_z = [w_T_Link5(1,3) w_T_Link5(2,3) w_T_Link5(3,3)];
Link4_x = cross(Link4_z,Link5_z);
Link4_x = -Link4_x/(Link4_x(1)^2 + Link4_x(2)^2 + Link4_x(3)^2)^0.5;
Link4_y = cross(Link4_z,Link4_x);
w_T_Link4 = get_w_T_Link(Link4_x, Link4_y, Link4_z, pos_Link4);

Link3_T_Link4 = w_T_Link3\w_T_Link4;
[alpha_4,a_4,theta_4,d_4] = get_DH_para( Link3_T_Link4 );
Link3_T_Link4_test = ROT('X',alpha_4) * MOB('X',a_4)...
          * ROT('Z',theta_4)* MOB('Z',d_4);
ERROR_34 = Link3_T_Link4_test-Link3_T_Link4;

% Link4 - Link5 DH参数 目标: Link4_T_Link5
% Link6
% x:-0.14323; y:0.00615; z:0.06799
% a:-91.981;  b:3.822;  g:-117.844
% Link4_T_Link5_estimate = ROT('X',alpha_estimate(5)) * MOB('X',a_estimate(5))...
%         * ROT('Z',theta_estimate(5))* MOB('Z',d_estimate(5));
pos_Link6 = [-0.14323 0.00615 0.06799-0.1] * 1000;
ori_Link6 = [-91.981 3.822 -117.844] * pi/180;
w_T_Link6 = MobXYZ(pos_Link6) * EulerZYX(ori_Link6);

Link5_z = [w_T_Link5(1,3) w_T_Link5(2,3) w_T_Link5(3,3)];
Link6_z = [w_T_Link6(1,3) w_T_Link6(2,3) w_T_Link6(3,3)];
Link5_x = cross(Link5_z,Link6_z);
Link5_x = Link5_x/(Link5_x(1)^2 + Link5_x(2)^2 + Link5_x(3)^2)^0.5;
Link5_y = cross(Link5_z,Link5_x);
w_T_Link5 = get_w_T_Link(Link5_x, Link5_y, Link5_z, pos_Link5);

Link4_T_Link5 = w_T_Link4\w_T_Link5;
[alpha_5,a_5,theta_5,d_5] = get_DH_para( Link4_T_Link5 );

% Link5 - Link6 DH参数 目标: Link5_T_Link6
% Link7
% x:-0.14438; y:0.00654; z:0.0685
% a:-176.90;  b:-62.041;  g:-173.223
pos_Link7 = [-0.14438 0.00654 0.0685-0.1] * 1000;
ori_Link7 = [-176.90 -62.041 -173.223] * pi/180;
w_T_Link7 = MobXYZ(pos_Link7) * EulerZYX(ori_Link7);

Link6_z = [w_T_Link6(1,3) w_T_Link6(2,3) w_T_Link6(3,3)];
Link7_z = [w_T_Link7(1,3) w_T_Link7(2,3) w_T_Link7(3,3)];
Link6_x = cross(Link6_z,Link7_z);
Link6_x = Link6_x/(Link6_x(1)^2 + Link6_x(2)^2 + Link6_x(3)^2)^0.5;
Link6_y = cross(Link6_z,Link6_x);
w_T_Link6 = get_w_T_Link(Link6_x, Link6_y, Link6_z, pos_Link6);

Link5_T_Link6 = w_T_Link5\w_T_Link6;
[alpha_6,a_6,theta_6,d_6] = get_DH_para( Link5_T_Link6 );

% Link6 - Link7 DH参数 目标: Link6_T_Link7
% w_T_Link7 是准确的
Link6_T_Link7 = w_T_Link6\w_T_Link7;
[alpha_7,a_7,theta_7,d_7] = get_DH_para( Link6_T_Link7 );

%% DH参数汇总
alpha = [alpha_1 alpha_2 alpha_3 alpha_4 alpha_5 alpha_6 alpha_7];
a = [a_1 a_2 a_3 a_4 a_5 a_6 a_7];
theta = [theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7];
d = [d_1 d_2 d_3 d_4 d_5 d_6 d_7];
% 验证
w_T_e = eye(4);
for i = 1 : 4
    w_T_e = w_T_e * ROT('X',alpha(i)) * MOB('X',a(i))...
        * ROT('Z',theta(i))* MOB('Z',d(i));
end
ERROR = w_T_e - w_T_Link4;
% disp(ERROR)

