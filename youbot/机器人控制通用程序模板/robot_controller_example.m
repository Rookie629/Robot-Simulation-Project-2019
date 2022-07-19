function robot_controller_example()
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
	%h=robot_init(vrep, id);
	%fsm=XXXX
	%开启VREP和matlab的同步机制
	res = vrep.simxSynchronous(h.id, true); vrchk(vrep, res);
%#############################################机器人有限状态机部分######################################################################
	%开始有限状态机
	while true,
	%请在此处补充状态机相关代码
	

	
	
	 %触发同步
	 vrep.simxSynchronousTrigger(id);
	%end while
	end
%输出程序结束
disp('Program ended');
%endfunction
end



