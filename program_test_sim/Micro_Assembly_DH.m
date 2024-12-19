% Micro_Assembly DH参数
% 机器人的初始参数4P3R机器人
% link类 R = Link([theta,d,a,alpha])关节角、连杆偏距、连杆长度、连杆转角
% d = [0 0 49.27 31.94]
% theta = [4.82*pi/180 -pi/2 0]
% L(1)用于和世界坐标系换算，不是机器人的joint
L(1) = Link([0 0 0 -pi/2],'modefied');

L(2) = Link([0 0 15 pi/2],'modefied');
L(2).qlim = [0 100];
L(2).offset = 0-50;
L(2).jointtype = 'P';

L(3)= Link([-10*pi/180 0 17.82 pi/2],'modefied');
L(3).qlim = [0 100];
L(3).jointtype = 'P';
L(3).offset = 0-50;

L(4)= Link([pi/2 49.27 0 -160*pi/180],'modefied');
L(4).offset = 49.27-50; 
L(4).qlim = [0 100];
L(4).jointtype = 'P';

L(5)= Link([-30.38*pi/180 0 159.08 -81.35*pi/180],'modefied');        % L(5).jointtype = 'P';
L(5).jointtype = 'P';
L(5).qlim = [0 100];
L(5).offset = -31.94-50;

L(6) = Link([0 1.23 0 pi/2],'modefied');
L(6).offset = -5.04*pi/180;
L(6).qlim = [-12*pi/180 12*pi/180];

L(7) = Link([90 0 0 pi/2],'modefied');
L(7).offset = pi/2;
L(7).qlim = [-12*pi/180 12*pi/180];

L(8) = Link([0 0 0 0],'modefied');
L(8).qlim = [-pi/4 pi/4];
Micro_Assembly = SerialLink([L(1),L(2),L(3),L(4),L(5),L(6),L(7),L(8)]);
% Micro_Assembly = SerialLink([L(1),L(2),L(3),L(4),L(5)]);
Micro_Assembly.name = 'MicroAssembly';
Micro_Assembly.teach  % 用于调试


