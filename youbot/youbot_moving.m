function youbot_moving()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||本程序示例如何使用youbot机器人的底盘自由度及状态机软件架构的使用方法|||||||||||||||||||||||||||||||||||
%***********************************************************************************************************************

%###############################################################固定程序创建VREP和matlab的连接并远程启动###################################################################
    disp('Program started');
    %加载remoteApiProto.m文件 
    vrep = remApi('remoteApi');
    %关闭所有VREP和matlab的连接
    vrep.simxFinish(-1);
    %创建新的VREP和matlab的连接，返回状态为id，如果id<0代表创建连接失败
    id = vrep.simxStart('127.0.0.1', 19997, true, true, 2000, 5);
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    % 确保程序意外中断后系统自动清除VREP和matlab的连接数据
    cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%由matlab远程启动VREP的仿真
    vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);
%###############################################################################初始化程序变量###############################################################################
    % 初始化youbot机器人获取所有相关句柄 .
    h = youbot_init(vrep, id);
   % 初始化Hokuyo激光雷达获取所有相关句柄 
    h = youbot_hokuyo_init(vrep, h);
    % 暂定2s保证初始化过程完成 
    pause(2);

    % Youbot constants
    timestep = .05;

    forwBackVel = 0;
    leftRightVel = 0;
    rotVel = 0;
    prevOri = 0; 
    prevLoc = 0;

    % Make sure everything is settled before we start. 
    pause(2);
    
    % 初始化状态机
    fsm = 'cst_forwBack';

    %% Start the demo. 
    while true
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
    
        % 获取移动机器人相对于世界坐标系的位置和姿态 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);

        %机器人前后移动状态机. 
        if strcmp(fsm, 'cst_forwBack');
            
            %前后速度
            forwBackVel = -1;
            
            % 使得机器人逼近Y=-6.5的位置. 
            if youbotPos(2) + 6.5 < .001
                forwBackVel = 0;
                fsm = 'lin_forwBack';
            end
        %    
        elseif strcmp(fsm, 'lin_forwBack');
            %控制机器人前进运动速度与Y=-4.5位置相关
            forwBackVel = - 2 * (youbotPos(2) + 4.5);
            
            % 使得机器人逼近Y=-4.5的位置. 
            if abs(youbotPos(2) + 4.5) < .001
                forwBackVel = 0;
                fsm = 'LeftRight';
            end
            
        elseif strcmp(fsm, 'LeftRight');
            % 控制机器人前进运动速度与X=-4.5位置相关
            leftRightVel = - 2 * (youbotPos(1) + 4.5);
            
            % 使得机器人逼近X=-4.5的位置.
            if abs(youbotPos(1) + 4.5) < .001
                leftRightVel = 0;
                fsm = 'Rot';
            end
         %旋转状态机   
        elseif strcmp(fsm, 'Rot');
            % Rotate. 
            rotVel = angdiff(- pi / 2, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff(- pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'finished';
            end
            
        elseif strcmp(fsm, 'finished'),
            pause(3);
            break
            
        else
            error(sprintf('Unknown state %s.', fsm));
        end

        % Update wheel velocities for each loop
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

        % Make sure that we do not go faster that the simulator. 
        elapsed = toc;
        timeleft = timestep - elapsed;
        if (timeleft > 0),
          pause(min(timeleft, .01));
        end
    end

end % main function
   
