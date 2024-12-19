function [ rot_mat ] = EulerXYZ( theta )
%% X-Y-Z固定角 用于MATLAB的读取
    rot_mat = ROT('Z',theta(3)) * ROT('Y',theta(2)) * ROT('X',theta(1));
end