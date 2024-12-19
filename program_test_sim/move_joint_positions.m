% 移动机器人各关节move_joint_positions
function res = move_joint_positions(vrep, clientID, Handle_joints, positions)
%% 设置关节角位置
% vrep.simxSetJointTargetPosition(clientID, Handle_gripper_joint(1,1), 0, vrep.simx_opmode_oneshot_wait);
    for i = 1 : 7
        res = vrep.simxSetJointPosition(clientID, Handle_joints(i), positions(i), vrep.simx_opmode_blocking);
    end
end