function [alpha,a,theta,d] = get_DH_para( MAT_DH )
% 奇异点没有排除，请谨慎使用该函数
% DH参数公式
%   [           cos(theta),           -sin(theta),           0,             a]
%   [cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha)]
%   [sin(alpha)*sin(theta), sin(alpha)*cos(theta),  cos(alpha),  d*cos(alpha)]
%   [                    0,                     0,           0,             1]

% 求解a
a = MAT_DH(1,4);

% 求解theta
if      abs(MAT_DH(1,1)) < 0.05 && MAT_DH(1,2) < 0       % 关于theta接近90°的情况，用泰勒展开近似
    theta = pi/2 - MAT_DH(1,1);
elseif  abs(MAT_DH(1,1)) < 0.05 && MAT_DH(1,2) > 0       % 关于theta接近-90°的情况，用泰勒展开近似
    theta = MAT_DH(1,1) - pi/2;
else
    theta = atan2(-MAT_DH(1,2),MAT_DH(1,1));        % atan2(sin(theta),cos(theta))
end

% 求解d和alpha 分段计算
% abs_d = (MAT_DH(2,4)^2 + MAT_DH(3,4)^2)^0.5;
if abs(MAT_DH(3,3)) < 0.1 && MAT_DH(2,3) < 0        % 关于alpha接近90°的情况，sin(alpha)>0 用泰勒展开近似
    alpha = pi/2 - MAT_DH(3,3);                     % 
    d = MAT_DH(2,4)/MAT_DH(2,3);     % 若d为正，-d*sin(alpha)为负数，d与-d*sin(alpha)符号相反
elseif abs(MAT_DH(3,3)) < 0.1 && MAT_DH(2,3) > 0    % 关于alpha接近-90°的情况，sin(alpha)<0 用泰勒展开近似
    alpha = MAT_DH(3,3) - pi/2;                     %
    d = MAT_DH(2,4)/MAT_DH(2,3);      % 若d为正，-d*sin(alpha)为正数，d与-d*sin(alpha)符号相同
else
    alpha = atan2(-MAT_DH(2,3),MAT_DH(3,3));        % atan2(sin(alpha),cos(alpha))
    d = MAT_DH(2,4)/MAT_DH(2,3);
end


end