%% 计算距离d的函数
% 输入Handle_Mirror,Handle_Interferometric,relativToRef，输出距离d
function d = get_distance_d(vrep,clientID,Handle_Mirror,Handle_Interferometric,relativToRef)
%% STEP1 读取干涉仪和镜面相对于base的位姿
% 获取位置和方向的句柄 镜子和干涉仪
% target_position表示Vrep中xyz的坐标
% target_orientation表示Vrep中abg
    res = vrep.simxGetObjectPosition(clientID,Handle_Mirror,relativToRef,...
               vrep.simx_opmode_streaming);
 vrchk(vrep, res,true);
    res = vrep.simxGetObjectOrientation(clientID,Handle_Mirror,relativToRef,...
               vrep.simx_opmode_streaming);
 vrchk(vrep, res,true);
    res = vrep.simxGetObjectPosition(clientID,Handle_Interferometric,relativToRef,...
               vrep.simx_opmode_streaming);
 vrchk(vrep, res,true);
    res = vrep.simxGetObjectOrientation(clientID,Handle_Interferometric,relativToRef,...
               vrep.simx_opmode_streaming);
  vrchk(vrep, res,true);

% 相对坐标系是base坐标系  读取Handle_Mirror Handle_Interferometric的位姿
    [res , TargPos_Mirror]= vrep.simxGetObjectPosition(clientID,Handle_Mirror,...
                          relativToRef,vrep.simx_opmode_blocking);
    vrchk(vrep, res,true);
    [res , Targtheta_Mirror]= vrep.simxGetObjectOrientation(clientID,Handle_Mirror,...
                            relativToRef,vrep.simx_opmode_blocking);
    vrchk(vrep, res,true);
    [res , TargPos_Interferometric]= vrep.simxGetObjectPosition(clientID,Handle_Interferometric,...
                          relativToRef,vrep.simx_opmode_blocking);
    vrchk(vrep, res,true);
    [res , Targtheta_Interferometric]= vrep.simxGetObjectOrientation(clientID,Handle_Interferometric,...
                            relativToRef,vrep.simx_opmode_blocking);
    vrchk(vrep, res,true);
%% STEP2 将读取的位姿带入公式得到距离d
% 将读取的位姿转换为位姿矩阵w_T_t & w_T_f

% 初始化，将target_position转化为mm为单位（Vrep中为m），target_orientation默认为rad为单位
TargPos_Mirror = TargPos_Mirror * 1000;
TargPos_Interferometric = TargPos_Interferometric * 1000;

% 计算位姿矩阵w_T_t & w_T_f
        w_T_t=EulerZYX(Targtheta_Mirror);
        w_T_t(1:3, 4)= TargPos_Mirror; % XYZ
        w_T_t=double(w_T_t);  % 将w_T_t转化为双精度类型，得到更加精确的结果

        w_T_f=EulerZYX(Targtheta_Interferometric);
        w_T_f(1:3, 4)= TargPos_Interferometric; % XYZ
        w_T_f=double(w_T_f);  % 将w_T_f转化为双精度类型，得到更加精确的结果

% 读取矩阵中的数字并且带入公式求出d
a_wt = w_T_t(1,3);b_wt = w_T_t(2,3);c_wt = w_T_t(3,3);
x_wt = w_T_t(1,4);y_wt = w_T_t(2,4);z_wt = w_T_t(3,4);

a_wf = w_T_f(1,3);b_wf = w_T_f(2,3);c_wf = w_T_f(3,3);
x_wf = w_T_f(1,4);y_wf = w_T_f(2,4);z_wf = w_T_f(3,4);
d_wt = -a_wt * x_wt -b_wt*y_wt -c_wt*z_wt;

d = -(a_wt*x_wf +b_wt*y_wf + c_wt *z_wf +d_wt)/(a_wt*a_wf +b_wt *b_wf +c_wt *c_wf);%输出单位为mm
end
