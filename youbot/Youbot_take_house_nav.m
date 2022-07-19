function PX3_control_point_house_nav()
%###############################################################固定程序创建VREP和matlab的连接并远程启动###################################################################
	disp('Program started');
	%加载remoteApiProto.m文件
	vrep=remApi('remoteApi'); 
	%关闭所有VREP和matlab的连接
	vrep.simxFinish(-1); 
	%创建新的VREP和matlab的连接，返回状态为id，如果id<0代表创建连接失败
	id=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
	if id < 0,
		disp('Failed connecting to remote API server. Exiting.');
		vrep.delete();
		return;
	end
	fprintf('Connection %d to remote API server open.\n', id);
	% Make sure we close the connexion whenever the script is interrupted.
	cleanupObj = onCleanup(@() cleanup_vrep(vrep, id));
	%由matlab远程启动VREP的仿真
	res = vrep.simxStartSimulation(id, vrep.simx_opmode_oneshot_wait);

%#################################################初始化程序变量###############################################################################
    % 初始化youbot机器人获取所有相关句柄 .
    h = youbot_init(vrep, id);
   % 初始化Hokuyo激光雷达获取所有相关句柄 
    h = youbot_hokuyo_init(vrep, h);
	%小车轮子的半径
	r = 0.0975;
	%小车轮距半径
	b = 0.1655;
	%小车前进速度和转弯速度
	forwBackVel = 1;
	rotVel = 0;
    global cellsize;
    cellsize=0.25
%###################################基于地图的导航部分####################################
  %设定导航目标
  load('fmap.mat');
  
  goal = [ ij(-2.725), ij(-2.675) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %计算导航轨迹TRAJ
  %获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
  %[res youbotPos]=vrep.simxGetObjectPosition(h.id, h.ref,-1,vrep.simx_opmode_buffer);

  %[res youbotEuler]=vrep.simxGetObjectOrientation(h.id, h.ref,-1,vrep.simx_opmode_buffer);
  %traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  %traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  %traj = [traj; [xy(goal(1)) xy(goal(2))]];
  %traj = traj (2:end,:)
  %导航相关的变量初始化代码
  num=1;
  %初始化有限状态机变量  
  fsm='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
  leftRightVel=0;
%####################################################机器人有限状态机部分######################################################################
%开始有限状态机
	while true,
	%获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
[res youbotPos]=vrep.simxGetObjectPosition(h.id, h.ref,-1,vrep.simx_opmode_buffer);

[res youbotEuler]=vrep.simxGetObjectOrientation(h.id, h.ref,-1,vrep.simx_opmode_buffer);

	
	%获取新的目标点
%	target(1)=traj(num,1);
%	target(2)=traj(num,2);
  target(1)=-3;
	%计算目标点在机器人本体坐标下的坐标
        %目标点在世界坐标系的坐标
       % P_ow=[target(1);target(2);1];
        %移动机器人固连坐标系在世界坐标下的矩阵表示
		%T_cw=se2(youbotPos(1),youbotPos(2),youbotEuler(3));
        %计算目标点在移动机器人固连坐标系的坐标
		%P_oc=inv(T_cw)*P_ow; 
        
	%转向状态机
     if strcmp(fsm, 'rotate')
	%转向状态相关代码，计算移动机器人期望转向速度
  %if P_oc(1)>0
  %angl=-pi/2+atan2(P_oc(2),P_oc(1));
  %elseif P_oc(1)<0
  %angl=pi/2+atan2(P_oc(2),P_oc(1));  
  %elseif P_oc(1)==0
            %angl=0;
  %end
     angl=-90*pi/180;
       % angl=-atan2(P_oc(1),P_oc(2));
	    rotVel =5*angl;
        if ((youbotEuler(3))<(-90*pi/180))
        fsm = 'drivex';
        rotVel=0;
        end
	%前进状态机
     elseif strcmp(fsm, 'drivex')
	%前进状态相关代码，计算移动机器人期望前进速度
    
        forwBackVel =-1;
        %fsm = 'rotate';
        if (youbotPos(1)<-3)
             fsm='drivey';
             forwBackVel=0;
        end
     elseif  strcmp(fsm, 'drivey') 
        leftRightVel =1;
        %fsm = 'rotate';
        if (youbotPos(2)<-5.325)
             fsm='finished';
             leftRightVel=0;  
        end
	%结束状态机
     elseif strcmp(fsm, 'finished')
	%结束状态机相关代码
        %num=num+1;
        fsm='rotate';
        %if target==traj(end,:)
            break;
        %end
     end

  % 更新移动机器人控制状态   
        h = youbot_drive(vrep, h, forwBackVel, leftRightVel, rotVel);

	%绘制坐标图
	%if 1,
		
		%[X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
		%plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');
		%axis equal;
		%axis([-7.8 7.8 -7.8 7.8]);
	%end
	drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end

disp('Program ended');
%endfunction
end



