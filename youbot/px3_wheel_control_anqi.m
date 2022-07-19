function px3_wheel_control()
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
	%初始化句柄控制变量
	h = PX3_init(vrep, id);
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);

%####################################################机器人有限状态机部分######################################################################
	%开始有限状态机
	%fsm=XXXX
	while true,
	%请在此处补充状态机相关代码
    %获取移动机器人本体固连坐标系相对于世界坐标系的位置和姿态
    [res youbotPos] = vrep.simxGetObjectPosition(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res);
    [res youbotEuler] = vrep.simxGetObjectOrientation(h.id, h.Pioneer_p3dx, -1,vrep.simx_opmode_buffer); vrchk(vrep, res); 
    youbotPos
    youbotEuler
	%补充移动机器人驱动模型建模代码
    r=0.0975;
    b=0.1655;
    forwBackVel=0.1;
    rotVel=0.1;
	vLeft=(forwBackVel-b*rotVel)/r;
    vRight=(forwBackVel+b*rotVel)/r;
	%向vrep发送双轮转速
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(1),vLeft,vrep.simx_opmode_oneshot); vrchk(vrep, res);
    vrep.simxSetJointTargetVelocity(h.id, h.wheelJoints(2),vRight,vrep.simx_opmode_oneshot); vrchk(vrep, res);
    if 1,
    cellsize=0.25
    [X,Y] = meshgrid((-7.5+cellsize/2):cellsize:(7.5-cellsize/2),(-7.5+cellsize/2):cellsize:(7.5-cellsize/2));
    plot( youbotPos(1), youbotPos(2), 'ob',7.5, 0, 'or', 0, 7.5, 'og');
    axis equal;
    axis([-7.8 7.8 -7.8 7.8]);
  end
  drawnow;
	vrep.simxSynchronousTrigger(id);
	%end while
	end
%endfunction
disp('Program ended');
end



