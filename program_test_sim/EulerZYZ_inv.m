function [ theta ] = EulerZYZ_inv( rot_mat )
%	旋转矩阵可以如下表式:
%	R = Rz(theta(5) * Ry(theta(6) * Rz(theta(7)
%   theta - inverse eulerzyz angle. 1*3 vector
    theta = [0,0,0];
    % 如果sin(theta(2))为0
    if abs(rot_mat(3,3)) < eps && rot_mat(3,3) < 0
        theta(2) = 2 * pi;
        theta(1) = 0;
        theta(3) = atan2(rot_mat(1,2), -rot_mat(1,1));
    elseif abs(rot_mat(3,3)) < eps && rot_mat(3,3) > 0
        theta(2) = 0;
        theta(1) = 0;
        theta(3) = atan2(-rot_mat(1,2), rot_mat(1,1));
    else % 正常的情况下
        theta(2) = atan2(sqrt(rot_mat(3,1).^2+rot_mat(3,2).^2),rot_mat(3,3));
        theta(1) = atan2(rot_mat(2,3)/sin(theta(2)),rot_mat(1,3)/sin(theta(2)));
        theta(3) = atan2(rot_mat(3,2)/sin(theta(2)),-rot_mat(3,1)/sin(theta(2)));
    end
end

