function res = set_joint_positions(vrep, clientID, Handle_joints, positions)
%% 设置关节角位置
    for i = 1 : 7
        res = vrep.simxSetJointPosition(clientID, Handle_joints(i), positions(i), vrep.simx_opmode_blocking);
    end
end