function youbot_arm()
%***********************************************************************************************************************
%|||||||||||||||||||||||||||||||||||||||本程序示例如何使用youbot机器人的机械臂及状态机软件架构的使用方法|||||||||||||||||||||||||||||||||||
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

    %定义机械臂关节空间运动范围
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.308996796608;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.5707963705063 ];

    % 获取机械臂末端tip坐标系的位置
    [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    vrchk(vrep, res, true);
    % 初始化状态机 
    fsm = 'noIK';
    

    %[res,Y_pos]=vrep.simxGetObjectPosition(id, h.Yellow_rec, h.armRef, vrep.simx_opmode_buffer); vrchk(vrep, res, true);


%###########################################################################################机器人有限状态机部分######################################################################
    while true
       %程序开启计时器（使用matlab的tic和toc机制，确保matlab周期与vrep基本一致）
        tic % See end of loop to see why it's useful. 
        
        if vrep.simxGetConnectionId(id) == -1
          error('Lost connection to remote API.');
        end
        
        %机械臂单关节空间控制状态机
        if strcmp(fsm, 'noIK')
            % Define each angle of the robot arm.
            chooseAngle = [90*pi/180, 30*pi/180, 54*pi/180, 20*pi/180, 0*pi/180];
            
            % Apply the value to each articulation.
            for i = 1:5
                res = vrep.simxSetJointTargetPosition(id, h.armJoints(i), chooseAngle(i), vrep.simx_opmode_oneshot);
                vrchk(vrep, res, true);
            end
            
            % Wait until the robot arm is in position.
            pause(5);
            fsm = 'useIK';
            
        % 机械臂笛卡尔空间控制状态机
        elseif strcmp(fsm, 'useIK')
            % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % 调整机械臂控制模式为2（笛卡尔空间模式）（默认0：关节空间控制模式）
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            
            % Set the new position wwanted for the gripper.
            tpos = [1.8002 , 3.5182 , 0.2151];
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(5);
            fsm = 'rotGrip';

        % 单关节模式控制机械臂末端手抓旋转状态机
        elseif strcmp(fsm, 'rotGrip')
            % Remove the inverse kinematics (IK) mode.
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 0, vrep.simx_opmode_oneshot_wait);
            % Set the new gripper angle to "0".
            res = vrep.simxSetJointTargetPosition(id, h.armJoints(5), 0, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(5);
            fsm = 'down';
        %机械臂单关节空间控制状态机
        elseif strcmp(fsm, 'down')
             % Get the arm position. 
            [res, tpos] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
            vrchk(vrep, res, true);
            
            % 调整机械臂控制模式为2（笛卡尔空间模式）（默认0：关节空间控制模式）
            res = vrep.simxSetIntegerSignal(id, 'km_mode', 2, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res, true);
            
            % Set the new position wwanted for the gripper.
            tpos = [1.8002 , 3.5182 , 0.0151];
            res = vrep.simxSetObjectPosition(id, h.ptarget, h.armRef, tpos, vrep.simx_opmode_oneshot);
            vrchk(vrep, res, true);
            
            % Wait long enough so that the tip is at the right position and go on to the next state. 
            pause(5);
            fsm = 'grasp';
            
        % 手抓闭合控制状态机
        elseif strcmp(fsm, 'grasp')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 0, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            
            pause(3);
            fsm = 'release';
        
        % 手抓张开控制状态机
        elseif strcmp(fsm, 'release')
            res = vrep.simxSetIntegerSignal(id, 'gripper_open', 1, vrep.simx_opmode_oneshot_wait);
            vrchk(vrep, res);
            pause(5);
            fsm = 'finished';
   
        elseif strcmp(fsm, 'finished')
            %% Demo done: exit the function. 
            pause(3);
            break;
        else
            error('Unknown state %s.', fsm);
        end

        % Make sure that we do not go faster that the simulator (each iteration must take 50 ms.)
        elapsed = toc;
        timeleft = timestep - elapsed;
        if timeleft > 0
            pause(min(timeleft, .01));
        end
    end

end % main function
   
