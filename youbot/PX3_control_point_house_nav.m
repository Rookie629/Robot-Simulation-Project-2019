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
	%初始化机器人相关变量返回给h结构体
	h = PX3_init(vrep, id);
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
  
  goal = [ ij(5), ij(-2) ];
  dx = Dstar(double(fmap), 'quiet');
  dx.plan([goal(1); goal(2)]);
  %计算导航轨迹TRAJ
  %获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
  [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
  [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
  traj = dx.path([ij(youbotPos(1)) ; ij(youbotPos(2))]);
  traj = [ xy(traj(:,1)) xy(traj(:,2)) ];
  traj = [traj; [xy(goal(1)) xy(goal(2))]];
  traj = traj (2:end,:)
  %导航相关的变量初始化代码
  num=1;
  %初始化有限状态机变量  
  fsm='rotate';
  res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);

%####################################################机器人有限状态机部分######################################################################
%开始有限状态机
	while true,
	%获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
	[res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
	[res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);  
	
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
     if strcmp(fsm, 'rotate'),
	%转向状态相关代码，计算移动机器人期望转向速度
        angl=atan2(P_oc(2),P_oc(1));
	    rotVel =5*angl;
        fsm = 'drive';
	%前进状态机
     elseif strcmp(fsm, 'drive'),
	%前进状态相关代码，计算移动机器人期望前进速度
        forwBackVel =0.5*sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2);
        fsm = 'rotate';
        if sqrt((youbotPos(1)-target(1))^2+(youbotPos(2)-target(2))^2)<0.1
             fsm='finished';
        end
        
	%结束状态机
     elseif strcmp(fsm, 'finished'),
	%结束状态机相关代码
        num=num+1;
        fsm='rotate';
        if target==traj(end,:)
            break;
        end
     end

  % 更新移动机器人控制状态
	res = vrep.simxPauseCommunication(h.id, true); vrchk(vrep, res);
		%双轮差动型移动机器人驱动方程，计算双轮转速
		vLeft = 1*(forwBackVel - b * rotVel) / r;
		vRight = 1*(forwBackVel + b * rotVel) / r;
		%向vrep发送双轮转速
		vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
		vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
	res = vrep.simxPauseCommunication(h.id, false); vrchk(vrep, res);
	%绘制坐标图
	if 1,
		
		[X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
		plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og',target(1), target(2), '*b');
		axis equal;
		axis([-7.8 7.8 -7.8 7.8]);
	end
	drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end

disp('Program ended');
%endfunction
end



