function vrep_Finish(vrep, clientID)
%% 结束vrep仿真
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);% simx_opmode_oneshot_wait: 表示我把数据发出去之后就开始执行后面的语句，不管你发完了没
vrep.simxFinish(clientID);% close the connection to V-REP:
vrep.delete(); % call the destructor!
end