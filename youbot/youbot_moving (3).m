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
    fsm = 'Rot1';

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
                %旋转状态机   
        if strcmp(fsm, 'Rot1');
            % Rotate. 
            rotVel = angdiff( pi / 2, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( pi / 2, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack1';
            end

        


        %机器人前后移动状态机. 
        %elseif strcmp(fsm, 'cst_forwBack');
            
            %前后速度
            %forwBackVel = -1;
            
            % 使得机器人逼近Y=-6.5的位置. 
            %if youbotPos(2) + 5.3 < .001
            %    forwBackVel = 0;
             %   fsm = 'lin_forwBack';
            %end
        %    
        elseif strcmp(fsm, 'lin_forwBack1');
            %控制机器人前进运动速度与Y=-3位置相关
            forwBackVel = 2 * (youbotPos(1) + 3);
            %forwBackVel = -2 
         
            
            % 使得机器人逼近Y=-3的位置. 
            if abs(youbotPos(1) + 3) < .001
                forwBackVel = 0;
                fsm = 'LeftRight1';
            end
            
        elseif strcmp(fsm, 'LeftRight1');
            % 控制机器人前进运动速度与X=-5.3位置相关
            leftRightVel = - 2 * (youbotPos(2) + 5.3);
            
            % 使得机器人逼近X=-5.3的位置.
            if abs(youbotPos(2) + 5.3) < .001
                leftRightVel = 0;
                fsm = 'LeftRight2';
            end

        elseif strcmp(fsm, 'LeftRight2');
            % 控制机器人前进运动速度与X=-2.85位置相关
            leftRightVel = - 0.5 * (youbotPos(2) + 3);
            
            % 使得机器人逼近X=-2.85的位置.
            if abs(youbotPos(2) + 3) < .001
                leftRightVel = 0;
                fsm = 'Rot2';
            end
        elseif strcmp(fsm, 'Rot2');
            % Rotate. 
            rotVel = angdiff( 91*pi/180, youbotEuler(3));
            
            % Stop when the robot is at an angle close to -pi/2. 
            if abs(angdiff( 91*pi/180, youbotEuler(3))) < .1 / 180 * pi
                rotVel = 0;
                fsm = 'lin_forwBack2';
            end
        elseif strcmp(fsm, 'lin_forwBack2');
            %控制机器人前进运动速度与Y=-6.4位置相关
            forwBackVel = 1 * (youbotPos(1) + 6.2);
         
            
            % 使得机器人逼近Y=-6.4的位置. 
            if abs(youbotPos(1) + 6.2) < .001
                forwBackVel = 0;
                fsm = 'nav';
            end
        elseif strcmp(fsm, 'nav');
    global cellsize;
    cellsize=0.25

%###################################基于地图的导航部分####################################
  %设定导航目标
  load('fmap.mat');
  goal = [ ij(5.3), ij(-2.5) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %计算导航轨迹TRAJ
  %获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
   % 获取移动机器人相对于世界坐标系的位置和姿态 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);  
  traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  traj = [traj; [xy(goal(1)) xy(goal(2))]];
  traj = traj (2:end,:)
  %导航相关的变量初始化代码
  num=1;
  %初始化有限状态机变量  
  i='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%####################################################机器人有限状态机部分######################################################################
%开始有限状态机
	while true,
	%获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
	 % 获取移动机器人相对于世界坐标系的位置和姿态 
        [res, youbotPos] = vrep.simxGetObjectPosition(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);
        [res, youbotEuler] = vrep.simxGetObjectOrientation(id, h.ref, -1, vrep.simx_opmode_buffer);
        vrchk(vrep, res, true);  
	
	%获取新的目标点
	target(1)=traj(num,1);
	target(2)=traj(num,2);
  
	%计算目标点在机器人本体坐标下的坐标
        %目标点在世界坐标系的坐标
        P_ow=[target(1);target(2);1];
        %移动机器人固连坐标系在世界坐标下的矩阵表示
		T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
        %计算目标点在移动机器人固连坐标系的坐标
		P_oc=inv(T_cw)*P_ow; 

	%转向状态机
     if strcmp(i, 'rotate'),
	%转向状态相关代码，计算移动机器人期望转向速度
        %angl=atan2(P_oc(2),P_oc(1));
        if P_oc(1)>0;
             angl=atan(P_oc(2)/P_oc(1))-pi/2;
        end
        
        if  P_oc(1)<0;
            angl=atan(P_oc(2)/P_oc(1))+pi/2;
        end
        
        if P_oc(1)==0
            angl=0;
        end
	    rotVel =5*angl;
        i = 'drive';
	%前进状态机
     elseif strcmp(i, 'drive'),
	%前进状态相关代码，计算移动机器人期望前进速度
        forwBackVel =0.5*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
        i = 'rotate';
        if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
             i='finished';
        end
        
	%结束状态机
     elseif strcmp(i, 'finished'),
	%结束状态机相关代码
        num=num+1;
        i='rotate';
        if target==traj(end,:)
            break;
        end
     end
     end
  % 更新移动机器人控制状态
   h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);
        %elseif strcmp(fsm, 'finished'),
            %pause(3);
            %break
            
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
   
