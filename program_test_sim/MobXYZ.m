function [ rot_mat ] = MobXYZ( pos )
%% MohamedRaslan .. Muhammed Mohhie
    rot_mat = MOB('X',pos(1)) * MOB('Y',pos(2)) * MOB('Z',pos(3));
end