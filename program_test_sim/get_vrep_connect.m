function [vrep, clientID] = get_vrep_connect()
%% Á¬½ÓvrepÈí¼þ
vrep=remApi('remoteApi');
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID < 0)
    disp('Failed connecting to remote API server'); 
    return;
else
   disp('Succeed connecting to remote API server');  
end
end

